from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route("/")
def home():
    return jsonify({'message':'server is runningg'})

@app.route("/test")
def test():
    return jsonify({'message':'addition is done'})

if __name__ == "__main__":
    app.run(host='127.0.0.1', port=5000, debug=True)