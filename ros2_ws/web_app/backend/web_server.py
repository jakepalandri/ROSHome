from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import json
import os

app = Flask(__name__, static_folder='frontend')
CORS(app)

@app.route('/add-command', methods=['POST'])
def add_command():
    print("Adding new command...")
    data = request.get_json()
    command = data.get('command')
    action = data.get('action')

    # Append new command to JSON file
    commands_file = os.path.join(os.path.dirname(__file__), '../../assets/json/commands.json')
    with open(commands_file, 'r+') as f:
        commands = json.load(f)
        commands[command] = action
        f.seek(0)
        json.dump(commands, f, indent=4)
    
    return jsonify({"status": "success"})

@app.route('/')
def serve_index():
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/<path:path>')
def serve_static(path):
    return send_from_directory(app.static_folder, path)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
