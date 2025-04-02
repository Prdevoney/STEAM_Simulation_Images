# Web Socket Doc 
The web socket is used to receive messages from the client and execute the proper commands in the users container then send a response back to the client. 

## Requests Structure: 
**This is how requests need to be structured when sent to the web socket. Not all fields are required.**<br>
`ws://<gateway_ip_address>/<user_id>/<module_id>/command`

```
JSON
{
    "term_type": "<terminal_type>",
    "module_id": "<module_id>", 
    "command": "<command>", 
    "python_script": "<Monaco editor content>",
    "interactive_input": "<any valid input>",
}
```

## Fields: 

### **`term_type:`** (required) This is to know what terminal the process must be ran in and how the logic should be handled
<table border="1">
  <tr>
    <th>"term_type" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>simulation_terminal</code></td>
    <td>Use this if you are starting or restarting a simulation</td>
  </tr>
  <tr>
    <td><code>general_terminal</code></td>
    <td>Use this if you are executing users code</td>
  </tr>
  
</table>

### **`module_id`:** (required) Unique string so we can keep track of the module 
<table border="1">
  <tr>
    <th>"module_id" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>Unique string</code></td>
    <td>The unique ID used to identify each module</td>
  </tr>

</table>

### **`command:`** This is to start or restart a simulation
<table border="1">
  <tr>
    <th>"command" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>start</code></td>
    <td>Use this if you starting the simulation for the first time this current module</td>
  </tr>
  <tr>
    <td><code>restart</code></td>
    <td>Use this if you are restarting (reseting) the simulation</td>
  </tr>
  
</table>

### **`python_script:`** This is the python script that will be executed on the server 
<table border="1">
  <tr>
    <th>"python_script" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>Monaco editor content</code></td>
    <td>python script written by the user in the Monaco editor, just JSON.stringify() <br> the content in the Monaco editor and send it in the json directly</td>
  </tr>
</table>

### **`interactive_input:`** This is the input that the user gives to an interactive prompt  
*required only on subsequent calls to server when user is promped for input and `"term_type": "general_terminal"`* <br>
*you don't need to include the `python_script` on calls that include `interactive_input`*
<table border="1">
  <tr>
    <th>"interactive_input" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>Any valid user input</code></td>
    <td>This only needs to be included when the user is responding <br>to an interactive prompt, like arrow key strokes</td>
  </tr>
</table>


## Sample Calls: 

### Starting simulation: 
```
{
	"term_type": "simulation_terminal",
	"module_id": "test_world_image", 
	"command": "start", 
}
```

### Restarting simulation: 
```
{
  "term_type": "simulation_terminal", 
  "module_id": "test_world_image", 
  "command": "restart", 
}
```

### Executing user code: 
```
{
  "term_type": "general_terminal",
  "module_id": "test_world_image", 
  "python_script": "<monaco editor content>", 
}
```

### Sending user input to answer python_script prompt: 
```
{
  "term_type": "general_terminal",
  "module_id": "test_world_image", 
  "python_script": "", 
  "interactive_input": "<any valid user input>", 
}
```