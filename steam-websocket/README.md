# Web Socket Doc 
The web socket is used to receive messages from the client and execute the proper commands in the users container then send a response back to the client. 

## Requests Structure: 
**This is how requests need to be structured when sent to the web socket, not all fields are required**
```
JSON
{
    "question_id": "<question_id>",
    "question_type": "<type>",
    "python_script": <python_script.py>,
    "term_type": "<terminal_type>",
    "interactive_input": "<any valid input>"
}
```
## Fields: 
### **question_id**: (required) This is the id of the question that is being answered. It tells us what python script to run. 
<table border="1">
  <tr>
    <th>"question_id" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>the current question id</code></td>
    <td>This is so we know what question was just answered. We can use this id for multiple things<br> such as executing a python script on the server for a multiple choise question or <br>keeping track of a terminal for an interactive question</td>
  </tr>
</table>

### **question_type:** (required) this is the service that you are looking for 
<table border="1">
  <tr>
    <th>"question_type" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>frq</code></td>
    <td>This is for when a user creates their own code through <br>the monaco editor and it needs to be executed </td>
  </tr>
  <tr>
    <td><code>multi</code></td>
    <td>This is for the regular multiple choice questions, no code needs to be uploaded here <br>because the python script will already be on the server and ready to execute</td>
  </tr>
</table>

### **python_script:** This is the shell command that is going to be executed in the containers os 
*required if:  `"question_type": "frq"`*
<table border="1">
  <tr>
    <th>"python_script" possible values</th>
    <th>what it's for</th>
  </tr>
  <tr>
    <td><code>&lt;python_script&gt;.py</code></td>
    <td>python script written by the user in the Monaco editor, encode the<br> file in the frontend and send it to the users container to be executed</td>
  </tr>
</table>

### **term_type:** (required) This is to know what terminal the process must be ran in and whether it is persistant or not
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
    <td><code>active_terminal</code></td>
    <td>Use this if the terminal is persistant meaning after the user submited their answer they will <br>either be prompted for interaction or continue to recieve data from the server. We will use <br>the question id to keep track of the terminal session. This can be expanded later to type of<br> interaction needed but for our use now with interactivity all we need is arrow key strokes</td>
  </tr>
</table>

### **interactive_input:** This is the input that the user gives to an interactive prompt  
*required only on subsequent calls when:  `"term_type": "active_terminal"`*
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