# Imports for flask server 
from flask import Flask, request, jsonify

# Imports to execute code and work with file system
import subprocess
import os

app = Flask(__name__)

@app.route("/")
def index():
    return "Flask server is running!"

# Route definition 
@app.route("/execute", methods=['POST'])
def execute_code():

    # Check if the request has a file
    if 'file' not in request.files: 
        return jsonify({"error": "No file provided"}), 400

    file = request.files['file']
    if file.filename == '': 
        return jsonify({"error": "No file selected"}), 400
    
    try: 
        file.save('temp_code.py')
        subprocess.run(['chmod','+x', 'temp_code.py'], check=True)
        result = subprocess.run(
            ['python', 'temp_code.py'],
             capture_output=True,
             text=True,
             timeout=30
        )
        os.remove('temp_code.py')
        return jsonify({
            'stdout': result.stdout,
            'stderr': result.stderr,
            'returncode': result.returncode
        })
    except Exception as e:
        if os.path.exists('temp_code.py'):
            os.remove('temp_code.py')
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)