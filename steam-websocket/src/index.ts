import * as WebSocket from 'ws';
import * as pty from 'node-pty';
import * as os from 'os';
import * as path from 'path';
import * as fs from 'fs';

const wss = new WebSocket.Server({ port: 4001 });

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
    cwd: process.cwd(), 
  });
  return term; 
};

const executeScript = (data: any, term: any) => {
  // Get the temp dir 
  const tempDir = os.tmpdir(); 

  // Create temp python file
  const tempFile = path.join(tempDir, 'temp_script.py');

  // Add 'python_script' to the temp python file
  fs.writeFileSync(tempFile, data.python_script);

  // Execute the python script
  term.write(`python3 ${tempFile}\r`);
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
      }
      // Check if question_id is provided
      if (!message.question_id) {
        console.log('No question_id provided');
      }
      
      if (message.term_type === "gen_terminal") {
        try {
          // create new terminal 
          const term = spawnTerm();
          // Call function to execute the python script
          const tempFile = executeScript(message, term);

          term.onData((output: any) => {
            // Skip output if it contains the command or the directory path
            if (output.includes("python3") || output.includes(process.cwd())) {
              return;
            }
            console.log('Output: %s', output);
            ws.send(output);
          });
        
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
