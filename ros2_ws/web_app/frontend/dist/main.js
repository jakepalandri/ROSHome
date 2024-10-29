"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g;
    return g = { next: verb(0), "throw": verb(1), "return": verb(2) }, typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (_) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
// Variables
var backendURL = "http://192.168.0.100:5000"; // on lab router network
// const backendURL = "http://192.168.0.186:5000"; // on home network
var toggleButton = document.getElementById("toggleButton");
var addCommandsDiv = document.getElementById("addCommandsDiv");
var viewRemoveCommandsDiv = document.getElementById("viewRemoveCommandsDiv");
var commandList = document.getElementById("commandList");
var commandInput = document.getElementById("command");
var actionInput = document.getElementById("action");
loadCommands();
toggleButton.addEventListener("click", function () { return __awaiter(void 0, void 0, void 0, function () {
    var addCommandsVisible;
    return __generator(this, function (_a) {
        switch (_a.label) {
            case 0:
                addCommandsVisible = addCommandsDiv.style.display === "block";
                addCommandsDiv.style.display = addCommandsVisible ? "none" : "block";
                viewRemoveCommandsDiv.style.display = addCommandsVisible ? "block" : "none";
                // Update button text
                toggleButton.textContent = addCommandsVisible ? "Add Commands" : "View Commands";
                commandInput.value = "";
                actionInput.value = "";
                return [4 /*yield*/, loadCommands()];
            case 1:
                _a.sent();
                return [2 /*return*/];
        }
    });
}); });
// Load commands from backend and display them in the list
function loadCommands() {
    return __awaiter(this, void 0, void 0, function () {
        var response, commands_1, error_1;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    _a.trys.push([0, 3, , 4]);
                    return [4 /*yield*/, fetch(backendURL + "/api/commands")];
                case 1:
                    response = _a.sent();
                    if (!response.ok)
                        throw new Error("Error fetching commands: " + response.statusText);
                    return [4 /*yield*/, response.json()];
                case 2:
                    commands_1 = _a.sent();
                    commandList.innerHTML = ''; // Clear current list
                    Object.keys(commands_1).forEach(function (command) {
                        var row = document.createElement("tr");
                        // Create and set command key cell
                        var keyCell = document.createElement("td");
                        keyCell.textContent = command;
                        row.appendChild(keyCell);
                        // Create and set command value cell
                        var valueCell = document.createElement("td");
                        valueCell.textContent = commands_1[command];
                        if (command.includes("that"))
                            valueCell.textContent = "[gesture]_" + commands_1[command];
                        row.appendChild(valueCell);
                        // Create actions cell with delete button
                        var actionsCell = document.createElement("td");
                        var deleteButton = document.createElement("button");
                        deleteButton.className = "btn btn-danger btn-sm";
                        deleteButton.textContent = "Delete";
                        deleteButton.onclick = function () { return deleteCommand(command); };
                        actionsCell.appendChild(deleteButton);
                        row.appendChild(actionsCell);
                        // Append the row to the command list
                        commandList.appendChild(row);
                    });
                    return [3 /*break*/, 4];
                case 3:
                    error_1 = _a.sent();
                    console.error(error_1);
                    commandList.innerHTML = "<li class=\"list-group-item text-danger bg-dark text-light\">Failed to load commands</li>";
                    return [3 /*break*/, 4];
                case 4: return [2 /*return*/];
            }
        });
    });
}
// Function to delete a command
function deleteCommand(commandKey) {
    return __awaiter(this, void 0, void 0, function () {
        var response, error_2;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    _a.trys.push([0, 3, , 4]);
                    return [4 /*yield*/, fetch(backendURL + "/api/commands/" + commandKey, {
                            method: 'DELETE',
                        })];
                case 1:
                    response = _a.sent();
                    if (!response.ok)
                        throw new Error("Failed to delete command: " + response.statusText);
                    // Reload commands after successful deletion
                    return [4 /*yield*/, loadCommands()];
                case 2:
                    // Reload commands after successful deletion
                    _a.sent();
                    return [3 /*break*/, 4];
                case 3:
                    error_2 = _a.sent();
                    console.error(error_2);
                    alert("Error deleting command: " + commandKey);
                    return [3 /*break*/, 4];
                case 4: return [2 /*return*/];
            }
        });
    });
}
// Add command form submission
document.getElementById("commandForm").addEventListener("submit", function (event) { return __awaiter(void 0, void 0, void 0, function () {
    var command, action, response, error_3;
    return __generator(this, function (_a) {
        switch (_a.label) {
            case 0:
                event.preventDefault();
                command = document.getElementById("command").value;
                action = document.getElementById("action").value;
                _a.label = 1;
            case 1:
                _a.trys.push([1, 6, , 7]);
                return [4 /*yield*/, fetch(backendURL + "/add-command", {
                        method: "POST",
                        headers: {
                            "Content-Type": "application/json"
                        },
                        body: JSON.stringify({ command: command, action: action })
                    })];
            case 2:
                response = _a.sent();
                if (!response.ok) return [3 /*break*/, 4];
                document.getElementById("message").innerText = "Command added successfully!";
                commandInput.value = "";
                actionInput.value = "";
                return [4 /*yield*/, loadCommands()];
            case 3:
                _a.sent();
                return [3 /*break*/, 5];
            case 4:
                document.getElementById("message").innerText = "Failed to add command.";
                _a.label = 5;
            case 5: return [3 /*break*/, 7];
            case 6:
                error_3 = _a.sent();
                document.getElementById("message").innerText = "Error: Could not connect to server.";
                return [3 /*break*/, 7];
            case 7: return [2 /*return*/];
        }
    });
}); });
