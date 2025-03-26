import * as WebSocket from 'ws';
import * as pty from 'node-pty';
import * as os from 'os';
import * as path from 'path';
import * as fs from 'fs';

const wss = new WebSocket.Server({ port: 8002 });

console.log('Started web socket server on port 8002'); 
// Determine the shell of the os, even though we know the conatiner is going to be Ubuntu (bash) 
const shell = os.platform() === 'win32' ? 'powershell.exe' : 'bash';

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

      // Check if question_id is provided
      if (!message.question_id) {
        console.log('No question_id provided');
      }
      
      if (message.term_type === "gen_terminal") {
        try {
          // Check if python script is provided
          if (!message.python_script) {
            console.log('No python script provided');
            return; 
          }
          // create new terminal 
          const term = spawnTerm();
          // Call function to execute the python script
          executeScript(message, term);
          // Set up listener for this terminal
          termOutput(term, ws);
        
        } catch (e) {
          console.error('Error: %s', e);
        }
      } else if (message.term_type === "interactive_terminal") {
          try {
            // First call for interactive_terminal, create new terminal and run script 
            if (!message.interactive_input) {
              if (!message.python_script) {
                console.log('No python script provided');
                return; 
              }
              // create new terminal 
              const term = spawnTerm(); 

              // Logic for saving terminal in map with the message.question_id as the key
              terminals.set(message.question_id, term);

              // Call function executeScript to run the recieved python script 
              executeScript(message, term); 
              // Call function to output the terminal output to the websocket
              termOutput(term, ws);
            
            } else {
              const term = terminals.get(message.question_id);
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
