# Web Socket Doc 
The web socket is used to receive messages from the client and execute the proper commands in the users container then send a response back to the client. 

## Requests Structure: 
**This is how requests need to be structured when sent to the web socket. Not all fields are required.**
```
JSON
{
    "question_id": "<id>",
    "term_type": "<terminal_type>",
    "python_script": "<Monaco editor content>",
    "interactive_input": "<any valid input>",
}
```
## Fields: 

### **question_id:** (required) Unique string so we can keep track of the question that the request is coming from 
<table border="1">
  <tr>
    <th>"question_id" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>Unique string</code></td>
    <td>Unique string for the question, use the same ID if multiple calls are made from the same question.<br> To be clear each question will have its own unique ID</td>
  </tr>

</table>

### **term_type:** (required) This is to know what terminal the process must be ran in and how the logic should be handled
<table border="1">
  <tr>
    <th>"term_type" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>gen_terminal</code></td>
    <td>Use for questions that do not require any user <br> interaction after the code has been submitted</td>
  </tr>
  <tr>
    <td><code>interactive_terminal</code></td>
    <td>Use this if the terminal prompts the user for input after the question has been answered and the script has been run.<br> We will use the question id to keep track of the terminal session. This can be expanded later to<br> type of interaction needed but for our use now with interactivity all we need is arrow key strokes</td>
  </tr>
  
</table>

### **python_script:** (required) This is the python script that will be executed on the server 
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

### **interactive_input:** This is the input that the user gives to an interactive prompt  
*required only on subsequent calls to server when:  `"term_type": "interactive_terminal"`* <br>
*you don't need to include `python_script` on subsequent calls*
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
