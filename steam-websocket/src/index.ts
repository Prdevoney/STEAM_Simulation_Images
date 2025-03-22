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

      if (message.type === "command") {
        const id = Date.now().toString(); 
        console.log(`Running ${message.type}: ${message.data}`);

        var term = pty.spawn(shell, [], {
          name: 'xterm-color',
          cols: 80,
          rows: 30
        });
        
        terminals.set(id, term);
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

      } else if (message.type === 'input' && message.id) {
        // Find the terminal for this ID
        const term = terminals.get(message.id);
        if (term) {
          // Send the input to the terminal
          term.write(message.data);
        }
      }

    } catch (e) {
      console.error('Error: %s', e);
    }
   
  });

});
