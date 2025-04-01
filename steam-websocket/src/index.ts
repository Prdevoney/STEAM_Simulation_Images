import * as WebSocket from 'ws';
import * as pty from 'node-pty';
import * as os from 'os';
import * as path from 'path';
import * as fs from 'fs';

const wss = new WebSocket.Server({ port: 8002 });

console.log('Started web socket server on port 8002'); 
// Determine the shell of the os, even though we know the conatiner is going to be Ubuntu (bash) 
const shell = os.platform() === 'win32' ? 'powershell.exe' : 'bash';

// Start script for each potential simulation 
const simulation_script_map: Record<string, string> = {
  "test_world_image": "ign gazebo -r /root/gaz_worlds_files/turtlebot3_baylands.sdf -s", // test_world_image
  "1LjViNIEB14XNArQtwaP": "ign gazebo -r - v 4 /usr/share/ignition/ignition-gazebo6/worlds/linear_battery_demo.sdf -s", // mod2_ros_intro
  "neOI52gdX1HInFQgE8Mp": "ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/joint_trajectory_controller.sdf -s", // mod3_robot_arm
  "bWSwj8u9RfeRd69jDkQ1": "ign gazebo -v 4 -r /root/gaz_worlds_files/tugbot_warehouse.sdf -s", // mod4_tugbot
  "hfQiob6b3V4WwvgcHyTf": "ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/multicopter_velocity_control.sdf -s", // mod5_drone
};

const sim_key = "simulation_key"; 
const gen_key = "general_key";

// Map to keep track of interactive terminal sessions 
const terminals = new Map(); 

// Generate a new pseudo terminal with node pty 
function spawnTerm() {
  const term = pty.spawn(shell, [], {
    name: 'xterm-color',
    cols: 80,
    rows: 30,
    cwd: process.cwd(), 
  });
  return term; 
}

function executeScript (data: any, term: any) {
  // Get the temp dir 
  const tempDir = os.tmpdir(); 

  // Create temp python file
  const tempFile = path.join(tempDir, 'temp_script.py');

  // Add 'python_script' to the temp python file
  fs.writeFileSync(tempFile, data.python_script);

  // Make python script executable 
  fs.chmodSync(tempFile, '755');

  // Execute the python script
  term.write(`python3 ${tempFile}\r`);
  console.log(`ran: python3 ${tempFile}`);
}

function executeStartScript (data: any, term: any) {
  const start_script = simulation_script_map[data.module_id]; 
  console.log(start_script); 
  if (data.command === "restart") {
    if (terminals.has(gen_key)) {
      const gen_term = terminals.get(gen_key);
      gen_term.write("\x03\r");
    }
    term.write("\x03\r");
    setTimeout(() => {
      term.write(`${start_script}\r`);
    },1000); // Wait for 1 second before executing the script
  }
  else if (data.command === "start") {
    console.log('Starting simulation');
    term.write(`${start_script}\r`);
  }
}

function interactiveInput (data: any, term: any) {
  term.write(`${data.interactive_input}\r`);
}

function termOutput (term: any, ws: WebSocket) {
  term.onData((output: any) => {
    // Skip output if it contains the command or the directory path
    if (output.includes("python3") || output.includes(process.cwd()) || output.includes("root")) {
      return;
    }
    console.log('Output: %s', output);
    ws.send(output);
  });
}

wss.on('connection', (ws: WebSocket) => {

  ws.send('Connected to TypeScript WebSocket Server on port 8002');

  ws.on('message', (data: WebSocket.Data) => {
    try {
      console.log('Payload: %s', data);
      const message = JSON.parse(data.toString());
      
      if (message.term_type === "simulation_terminal") {
        try {
          if (!message.command){
            console.log('No command provided');
            return;
          }
          // The simulation has already been created 
          if (message.command === "restart"){
            // get the terminal where the simulation is running
            const term = terminals.get(sim_key); 
            // execute restart command
            executeStartScript(message, term);
          } else if (message.command === "start") {
            if (!message.module_id){
              console.log('No module_id provided');
              return; 
            }
            // create new terminal 
            const term = spawnTerm();
            terminals.set(sim_key, term);
            executeStartScript(message, term);
            termOutput(term, ws);
          }
        
        } catch (e) {
          console.error('Error: %s', e);
        }
      } else if (message.term_type === "general_terminal") {
          try {
            // First call for gen_terminal, create new terminal and run script 
            if (!message.interactive_input) {
              if (!message.python_script) {
                console.log('No python script provided');
                return; 
              }
              if (!terminals.has(gen_key)) {
                // create new terminal 
                const term = spawnTerm(); 
                console.log('Created new terminal for general terminal');
                // Logic for saving terminal in map with the message.question_id as the key
                terminals.set(gen_key, term);
                // Call function executeScript to run the recieved python script 
                executeScript(message, term); 
                // Call function to output the terminal output to the websocket
                termOutput(term, ws);
              } else {
                // If the terminal already exists, get it from the map
                const term = terminals.get(gen_key);
                console.log("got the existing gen_terminal");
                // Call function executeScript to run the recieved python script 
                executeScript(message, term); 
                // Call function to output the terminal output to the websocket
                termOutput(term, ws);
              }
             
            } else {
              if (!terminals.has(gen_key)) {
                console.log('No terminal found for interactive input');
                return; 
              }
              const term = terminals.get(gen_key);
              interactiveInput(message, term);
            }
          } catch (e) {
            console.error('Error: %s', e);
          }
      } else {
        console.log("No term_type provided");
        ws.send('No term_type provided');
        return; 
      }
    } catch (e) {
      console.error('Error: %s', e);
    }
   
  });

});
