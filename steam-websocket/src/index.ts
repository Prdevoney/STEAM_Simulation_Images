import * as WebSocket from 'ws';
import * as pty from 'node-pty';
import * as os from 'os';
import * as path from 'path';
import * as fs from 'fs';

const wss = new WebSocket.Server({ port: 4000 });

console.log('Started web socket server on port 4000'); 
// Determine the shell of the os, even though we know the conatiner is going to be Ubuntu (bash) 
const shell = os.platform() === 'win32' ? 'powershell.exe' : 'bash';

// Map to keep track of interactive terminal sessions 
const terminals = new Map(); 

// Generate a new pseudo terminal with node pty 
const spawnTerm = () => {
  const term = pty.spawn(shell, [], {
    name: 'xterm-color',
    cols: 80,
    rows: 30,
    cwd: '/mnt/c/Users/PRDev/OneDrive - University of Central Florida/UCF/UCF Spring 2025/COP_4935/STEAM_Simulation_Images/python_scripts', 
  });
  return term; 
};

const executeScript = (script: any, term: any) => {
  // Logic for executing the python script
  


  return 'Python script executed successfully';
};

wss.on('connection', (ws: WebSocket) => {

  ws.send('Connected to TypeScript WebSocket Server on port 4000');

  ws.on('message', (data: WebSocket.Data) => {
    try {
      console.log('Payload: %s', data);
      const message = JSON.parse(data.toString());

      // Check if python script is provided
      if (!message.python_script) {
        console.log('No python script provided');
        ws.send('No python script provided');
        return;
      }
      
      if (message.term_type === "gen_terminal") {
        try {
          // create new terminal 
          const term = spawnTerm();
          // Call function to execute the python script
          const result = executeScript(message.python_script, term);
          // kill terminal after script is ran
          term.kill();
          return result;
        } catch (e) {
          console.error('Error: %s', e);
          ws.send('Error executing python script');
        }
      } 
      else if (message.term_type === "persitant_terminal") {
        
      }
      else if (message.term_type === "interactive_terminal") {
        
        // First call for interactive_terminal, create new terminal and run script 
        if (!message.interactive_input) {
          // create new terminal 
          const term = spawnTerm(); 

          // Logic for saving terminal in map with the message.question_id as the key
          terminals.set(message.question_id, term);

          // Call function executeScript to run the recieved python script 
          const response = executeScript(message.python_script, term); 
        }
      } else {
        console.log("No term_type provided");
        ws.send('No term_type provided');
        return; 
      }
      terminals.delete(message.question_id);
    } catch (e) {
      console.error('Error: %s', e);
    }
   
  });

});
