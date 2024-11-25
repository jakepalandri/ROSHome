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
// const backendURL = "http://192.168.0.162:5000"; // on home network
var toggleButton = document.getElementById("toggleButton");
var addCommandsDiv = document.getElementById("addCommandsDiv");
var viewCommandsDiv = document.getElementById("viewCommandsDiv");
var commandForm = document.getElementById("commandForm");
var inputContainer = document.getElementById("inputContainer");
var deviceTypeInput = document.getElementById("deviceType");
var firstCommandInput = document.getElementById("command0");
var addButton = document.getElementById("addButton");
var commandList = document.getElementById("commandList");
var submissionText = document.getElementById("submission");
var submissionButton = document.getElementById("submit");
var message = document.getElementById("message");
var inputCount = 1;
toggleButton.addEventListener("click", function () { return __awaiter(void 0, void 0, void 0, function () {
    var addCommandsVisible;
    return __generator(this, function (_a) {
        switch (_a.label) {
            case 0:
                addCommandsVisible = addCommandsDiv.style.display === "block";
                addCommandsDiv.style.display = addCommandsVisible ? "none" : "block";
                viewCommandsDiv.style.display = addCommandsVisible ? "block" : "none";
                // Update button text
                toggleButton.textContent = addCommandsVisible ? "Add Commands" : "View Commands";
                message.innerText = "";
                return [4 /*yield*/, loadCommands()];
            case 1:
                _a.sent();
                return [2 /*return*/];
        }
    });
}); });
// Load commands from backend and display them in the list
var loadCommands = function () { return __awaiter(void 0, void 0, void 0, function () {
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
                    var valuesCell = document.createElement("td");
                    commands_1[command].forEach(function (value) {
                        valuesCell.innerHTML += value + "<br/>";
                    });
                    row.appendChild(valuesCell);
                    // Create actions cell with delete button
                    var actionsCell = document.createElement("td");
                    var modifyButton = document.createElement("button");
                    modifyButton.className = "btn btn-primary btn-sm modify-button";
                    modifyButton.textContent = "Modify";
                    modifyButton.onclick = function () { return modifyCommand(command, commands_1[command]); };
                    actionsCell.appendChild(modifyButton);
                    var deleteButton = document.createElement("button");
                    deleteButton.className = "btn btn-danger btn-sm delete-button";
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
}); };
// Reset input fields
var resetInputs = function () {
    deviceTypeInput.value = "";
    inputContainer.innerHTML = "\n    <label for=\"command0\" class=\"form-label\" style=\"margin-top: 10px;\">\n        <h5>\n            Commands:\n        </h5>\n    </label>\n    <input type=\"text\" id=\"command0\" name=\"command0\" class=\"form-control bg-dark text-light\" placeholder=\"For example: turn on\" required>";
    inputCount = 1;
    firstCommandInput = document.getElementById("command0");
    submissionText.innerHTML = "";
    inputContainer.appendChild(firstCommandInput);
};
// Function to modify a command
var modifyCommand = function (device, commands) {
    toggleButton.click();
    resetInputs();
    deviceTypeInput.value = device;
    commands.forEach(function (command) {
        var input = document.getElementById("command" + (inputCount - 1));
        input.value = command;
        addInputField();
    });
    setSubmissionText();
};
// Function to delete a command
var deleteCommand = function (commandKey) { return __awaiter(void 0, void 0, void 0, function () {
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
}); };
// Add command form submission
commandForm.addEventListener("submit", function (event) { return __awaiter(void 0, void 0, void 0, function () {
    var deviceType, commands, inputs, response, error_3;
    return __generator(this, function (_a) {
        switch (_a.label) {
            case 0:
                event.preventDefault();
                deviceType = deviceTypeInput.value;
                commands = [];
                inputs = document.querySelectorAll("input");
                inputs.forEach(function (input) {
                    if (input.id === "deviceType" || input.value.trim() === "")
                        return;
                    commands.push(input.value.trim());
                });
                _a.label = 1;
            case 1:
                _a.trys.push([1, 6, , 7]);
                return [4 /*yield*/, fetch(backendURL + "/add-command", {
                        method: "POST",
                        headers: {
                            "Content-Type": "application/json"
                        },
                        body: JSON.stringify({ deviceType: deviceType, commands: commands })
                    })];
            case 2:
                response = _a.sent();
                if (!response.ok) return [3 /*break*/, 4];
                message.innerText = "Commands set successfully!";
                resetInputs();
                return [4 /*yield*/, loadCommands()];
            case 3:
                _a.sent();
                return [3 /*break*/, 5];
            case 4:
                message.innerText = "Failed to add command.";
                _a.label = 5;
            case 5: return [3 /*break*/, 7];
            case 6:
                error_3 = _a.sent();
                message.innerText = "Error: Could not connect to server.";
                return [3 /*break*/, 7];
            case 7: return [2 /*return*/];
        }
    });
}); });
// Set the submission text
var setSubmissionText = function () {
    var inputs = document.querySelectorAll("input");
    if (deviceTypeInput.value.trim() === "" || firstCommandInput.value.trim() === "")
        return;
    submissionText.innerHTML = "<h3>Individual commands:</h3>";
    submissionText.innerHTML += "<h6>The keyword <i>that</i> indicates a gesture is expected</h6>";
    var count = 1;
    inputs.forEach(function (input) {
        if (input.id === "deviceType")
            return;
        if (input.value.trim() !== "") {
            submissionText.innerHTML += "<p><strong>Command " + count + ":</strong> " + input.value + " that " + deviceTypeInput.value + "</p>";
        }
        count++;
    });
    submissionText.innerHTML += "<h3>Commands for all " + deviceTypeInput.value + "s:</h3>";
    count = 1;
    inputs.forEach(function (input) {
        if (input.id === "deviceType")
            return;
        if (input.value.trim() !== "") {
            submissionText.innerHTML += "<p><strong>Command " + count + ":</strong> " + input.value + " all " + deviceTypeInput.value + "s</p>";
        }
        count++;
    });
};
// Add a new input field
var addInputField = function () {
    var previousInput = document.getElementById("command" + (inputCount - 1));
    if (previousInput.value.trim() === "")
        return;
    var newInput = document.createElement("input");
    newInput.type = "text";
    newInput.classList.add("form-control");
    newInput.id = "command" + inputCount;
    newInput.name = "command" + inputCount;
    newInput.addEventListener("input", setSubmissionText);
    newInput.classList.add("form-control", "bg-dark", "text-light");
    if (inputCount === 1)
        newInput.placeholder = "For example: turn off";
    if (inputCount === 2)
        newInput.placeholder = "For example: turn up";
    if (inputCount === 3)
        newInput.placeholder = "For example: turn down";
    inputContainer.appendChild(newInput);
    inputCount++;
};
firstCommandInput.addEventListener("input", setSubmissionText);
deviceTypeInput.addEventListener("input", setSubmissionText);
addButton.addEventListener("click", addInputField);
// run these on load
loadCommands();
setSubmissionText();
