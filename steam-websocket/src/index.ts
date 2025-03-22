import * as WebSocket from 'ws';
import * as pty from 'node-pty';
import * as os from 'os';

const wss = new WebSocket.Server({ port: 4000 });

console.log('Started web socket server on port 4000'); 
// Determine the shell of the os, even though we know the conatiner is going to be Ubuntu (bash) 
const shell = os.platform() === 'win32' ? 'powershell.exe' : 'bash';

const terminals = new Map(); 

wss.on('connection', (ws: WebSocket) => {

  ws.send('Connected to TypeScript WebSocket Server on port 4000');

  ws.on('message', (data: WebSocket.Data) => {
    try {
      console.log('Payload: %s', data);
      const message = JSON.parse(data.toString());

      // Question type is free response 
      if (message.question_type === 'frq') {
        console.log('Received FRQ question');
        
        if (message.term_type === "interactive_terminal") {
          
          // create a new terminal object 
          if (!message.interactive_input) {
            var term = pty.spawn(shell, [], {
              name: 'xterm-color',
              cols: 80,
              rows: 30
            });
  
            // Logic for saving terminal in map with the message.question_id as the key
            terminals.set(message.question_id, term);
  
            // Call function executeScript to run the recieved python script 
            //const response = executeScript(message); 
          } else {
            // Logic for saving terminal in map with the message.question_id as the key
            const term = terminals.get(message.question_id);
            // Call function to execute input in the proper interactive terminal
          }
          
        } else if (message.term_type === "gen_terminal") {
          var term = pty.spawn(shell, [], {
            name: 'xterm-color',
            cols: 80,
            rows: 30
          });
          // Call function executeScript to run the recieved python script
          //const response = executeScript(message);
        }

        term.onData((output: any) => {
          ws.send(JSON.stringify({
            type: 'received output',
            id: id,
            data: output
          }));
        });

        term.onExit(({ exitCode }) => {
          ws.send(JSON.stringify({
            type: 'exit',
            id: id,
            code: exitCode
          }));
          
          terminals.delete(id);
        });

        term.write(message.data + '\r');

      } 

    } catch (e) {
      console.error('Error: %s', e);
    }
   
  });

});
