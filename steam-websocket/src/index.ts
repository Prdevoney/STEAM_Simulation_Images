import * as WebSocket from 'ws';

const wss = new WebSocket.Server({ port: 4000 });

wss.on('connection', (ws: WebSocket) => {
  ws.on('message', (message: WebSocket.Data) => {
    console.log('received: %s', message);
  });

  ws.send('Connected to TypeScript WebSocket Server');
});

console.log('WebSocket server started on port 4000');