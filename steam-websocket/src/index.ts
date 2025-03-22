import * as WebSocket from 'ws';
import * as pty from 'node-pty';
import * as os from 'os';

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

const executeScript = (message: any) => {
  // Logic for executing the python script
  return 'Python script executed successfully';
};

wss.on('connection', (ws: WebSocket) => {

  ws.send('Connected to TypeScript WebSocket Server on port 4000');

  ws.on('message', (data: WebSocket.Data) => {
    try {
      console.log('Payload: %s', data);
      const message = JSON.parse(data.toString());

      // Question type is free response 
      if (message.question_type === 'frq') {
        console.log('Received FRQ question');
        
        if (message.term_type === "gen_terminal") {
          try {
            // create new terminal 
            const term = spawnTerm();
            
            // Call function to execute the python script
            executeScript(message.python_script);

            term.onData((output: any) => {
              console.log('Output: %s', output);
            }); 
            // kill terminal after script is ran
            term.kill();
          } catch (e) {

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
            const response = executeScript(message); 
          } else {
            // Logic for saving terminal in map with the message.question_id as the key
            const term = terminals.get(message.question_id);
            // Call function to execute input in the proper interactive terminal

          }
          
        }
          
        terminals.delete(message.question_id);


      } else if (message.question_type === 'mcq') {
        console.log('Received MCQ question');

        if (message.term_type === "active_terminal") {
          
          // First call for interactive_terminal, create new terminal and run script 
          if (!message.interactive_input) {
            // create new terminal 
            const term = spawnTerm();

            // Logic for saving terminal in map with the message.question_id as the key
            terminals.set(message.question_id, term);
  
            // Call function executeScript to run the recieved python script 
            const response = executeScript(message); 
          } else {
            // Logic for saving terminal in map with the message.question_id as the key
            const term = terminals.get(message.question_id);
            // Call function to execute input in the proper interactive terminal

          }
          
        } else if (message.term_type === "gen_terminal") {
          try {
            // create new terminal 
            const term = spawnTerm();
            // Execute script in the general terminal
            term.write(`python3 ${message.python_script}`);

            term.onData((output: any) => {
              console.log('Output: %s', output);
            }); 
            // kill terminal after script is ran
            term.kill();
          } catch (e) {

          }
        }
      }

    } catch (e) {
      console.error('Error: %s', e);
    }
   
  });

});
