# Web Socket Doc 
The web socket is used to receive messages from the client and execute the proper commands in the users container then send a response back to the client. 

## Requests Structure: 
**This is how requests need to be structured when sent to the web socket, not all fields are required**
```
JSON
{
    "term_type": "<terminal_type>",
    "interactive_input": "<any valid input>",
    "python_script": "<Monaco editor content>",
}
```
## Fields: 

### **term_type:** (required) This is to know what terminal the process must be ran in and how the logic should be handled
<table border="1">
  <tr>
    <th>"term_type" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>gen_terminal</code></td>
    <td>Use for questions that do not require any user <br> interaction after the question is answerd</td>
  </tr>
  <tr>
    <td><code>persistant_terminal</code></td>
    <td>Use this if the terminal is persistant, meaning after the user submited their answer the terminal continues to <br>stream data to the client. We will use the question id to keep track of the terminal session. 
  </tr>
  <tr>
    <td><code>interactive_terminal</code></td>
    <td>Use this if the terminal prompts the user for input after the question has been answered and the script has been run.<br> We will use the question id to keep track of the terminal session. This can be expanded later to<br> type of interaction needed but for our use now with interactivity all we need is arrow key strokes</td>
  </tr>
  
</table>

### **interactive_input:** This is the input that the user gives to an interactive prompt  
*required only on subsequent calls to server when:  `"term_type": "interactive_terminal"`*
<table border="1">
  <tr>
    <th>"interactive_input" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>Any valid user input</code></td>
    <td>This only needs to be included when the user is responding to an interactive prompt, <br>like arrow key strokes</td>
  </tr>
</table>

### **python_script:** This is the python script that will be executed on the server 
*required if:  `"question_type": "frq"`*
<table border="1">
  <tr>
    <th>"python_script" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>&lt;python_script&gt;.py</code></td>
    <td>python script written by the user in the Monaco editor, just send <br> the content in the Monaco editor as a string in the JSON</td>
  </tr>
</table>
