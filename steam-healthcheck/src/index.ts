import * as http from 'http';

// Configuration
const PORT = 7002;
const HOST = '0.0.0.0'; // Listen on all interfaces

// Create HTTP server
const healthServer = http.createServer((req, res) => {
  // Set response headers
  res.writeHead(200, {
    'Content-Type': 'text/plain',
    'Cache-Control': 'no-store'
  });
  
  console.log('healthy');
  res.end('healthy');
});

// Start the server
healthServer.listen(PORT, HOST, () => {
  console.log(`health check server running on ${PORT}`);
});

// Handle server errors
healthServer.on('error', (error) => {
  console.error('Health check server error:', error);
});

// Handle shutdown
process.on('SIGTERM', () => {
  console.log('SIGTERM received, shutting down health check server');
  healthServer.close(() => {
    console.log('Health check server closed');
  });
});