from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import json
import os

app = Flask(__name__, static_folder='frontend')
CORS(app)
COMMANDS_FILE = '../../assets/json/commands.json'

# Load commands from JSON file
def load_commands():
    with open(COMMANDS_FILE, 'r') as file:
        return json.load(file)

# Save commands to JSON file
def save_commands(commands):
    with open(COMMANDS_FILE, 'w') as file:
        json.dump(commands, file, indent=4)

# Endpoint to get commands
@app.route('/api/commands', methods=['GET'])
def get_commands():
    commands = load_commands()
    return jsonify(commands)

# Endpoint to delete a command
@app.route('/api/commands/<command>', methods=['DELETE'])
def delete_command(command):
    commands = load_commands()
    if command in commands:
        del commands[command]
        save_commands(commands)
        return jsonify({"message": f"Device type '{command}' removed successfully"}), 200
    else:
        return jsonify({"error": "Command not found"}), 404

@app.route('/add-command', methods=['POST'])
def add_command():
    data = request.get_json()
    deviceType = data.get('deviceType')
    commandsList = data.get('commands')

    commands = load_commands()
    commands[deviceType.lower()] = commandsList
    save_commands(commands)

    return jsonify({"status": "success"})

@app.route('/')
def serve_index():
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/<path:path>')
def serve_static(path):
    return send_from_directory(app.static_folder, path)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
