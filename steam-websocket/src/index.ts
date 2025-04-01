import * as WebSocket from 'ws';
import * as pty from 'node-pty';
import * as os from 'os';
import * as path from 'path';
import * as fs from 'fs';
import { exec } from 'child_process';

const wss = new WebSocket.Server({ port: 8002 });

console.log('Started web socket server on port 8002'); 
// Determine the shell of the os, even though we know the conatiner is going to be Ubuntu (bash) 
const shell = os.platform() === 'win32' ? 'powershell.exe' : 'bash';

// Start script for each potential simulation 
const simulation_script_map: Record<string, string> = {
  "test_world_image": "ign gazebo -r gaz_worlds_files/turtlebot3_baylands.sdf", // test_world_image
  "1LjViNIEB14XNArQtwaP": "", // mod2_ros_intro
  "neOI52gdX1HInFQgE8Mp": "", // mod3_robot_arm
  "bWSwj8u9RfeRd69jDkQ1": "", // mod4_tugbot
  "hfQiob6b3V4WwvgcHyTf":"", // mod5_drone
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

  // clear terminal of anything that may be running 
  term.write("\x03\r");

  // Execute the python script
  term.write(`python3 ${tempFile}\r`);
}

function executeStartScript (data: any, term: any) {
  const script = simulation_script_map[data.module_id]; 
  if (data.command === "restart") {
    if (terminals.has(gen_key)) {
      const gen_term = terminals.get(gen_key);
      gen_term.write("\x03\r");
    }
    term.write("\x03\r");
    term.write(`${script}\r`);
  }
  else if (data.command === "start") {
    term.write("\x03\r");
    term.write(`${script}\r`);
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
                // Logic for saving terminal in map with the message.question_id as the key
                terminals.set(gen_key, term);
                // Call function executeScript to run the recieved python script 
                executeScript(message, term); 
                // Call function to output the terminal output to the websocket
                termOutput(term, ws);
              } else {
                // If the terminal already exists, get it from the map
                const term = terminals.get(gen_key);
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
