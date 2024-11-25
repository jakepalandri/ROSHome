// Variables
// CUSTOMISABLE
const backendURL = "http://192.168.0.100:5000"; // on lab router network
// const backendURL = "http://192.168.0.162:5000"; // on home network
const toggleButton = document.getElementById("toggleButton") as HTMLButtonElement;
const addCommandsDiv = document.getElementById("addCommandsDiv") as HTMLDivElement;
const viewCommandsDiv = document.getElementById("viewCommandsDiv") as HTMLDivElement;
const commandForm = document.getElementById("commandForm") as HTMLFormElement;
const inputContainer = document.getElementById("inputContainer") as HTMLDivElement;
const deviceTypeInput = document.getElementById("deviceType") as HTMLInputElement;
let   firstCommandInput = document.getElementById("command0") as HTMLInputElement;
const addButton = document.getElementById("addButton") as HTMLButtonElement;
const commandList = document.getElementById("commandList") as HTMLTableElement;
const submissionText = document.getElementById("submission") as HTMLParagraphElement;
const submissionButton = document.getElementById("submit") as HTMLButtonElement;
const message = document.getElementById("message") as HTMLParagraphElement;
let inputCount = 1;

toggleButton.addEventListener("click", async () => {
    // Toggle visibility of divs
    const addCommandsVisible = addCommandsDiv.style.display === "block";
    addCommandsDiv.style.display = addCommandsVisible ? "none" : "block";
    viewCommandsDiv.style.display = addCommandsVisible ? "block" : "none";

    // Update button text
    toggleButton.textContent = addCommandsVisible ? "Add Commands": "View Commands";
    message.innerText = "";
    await loadCommands();
});

// Load commands from backend and display them in the list
const loadCommands = async () => {
    try {
        const response = await fetch(`${backendURL}/api/commands`);
        if (!response.ok) throw new Error(`Error fetching commands: ${response.statusText}`);

        const commands = await response.json();
        commandList.innerHTML = ''; // Clear current list
        Object.keys(commands).forEach(command => {
            const row = document.createElement("tr");
      
            // Create and set command key cell
            const keyCell = document.createElement("td");
            keyCell.textContent = command;
            row.appendChild(keyCell);
      
            // Create and set command value cell
            const valuesCell = document.createElement("td");

            commands[command].forEach((value: string) => {
                valuesCell.innerHTML += `${value}<br/>`;
            });
            row.appendChild(valuesCell);
      
            // Create actions cell with delete button
            const actionsCell = document.createElement("td");

            const modifyButton = document.createElement("button");
            modifyButton.className = "btn btn-primary btn-sm modify-button";
            modifyButton.textContent = "Modify";
            modifyButton.onclick = () => modifyCommand(command, commands[command]);
            actionsCell.appendChild(modifyButton);

            const deleteButton = document.createElement("button");
            deleteButton.className = "btn btn-danger btn-sm delete-button";
            deleteButton.textContent = "Delete";
            deleteButton.onclick = () => deleteCommand(command);
            actionsCell.appendChild(deleteButton);
            row.appendChild(actionsCell);
      
            // Append the row to the command list
            commandList.appendChild(row);
        });
    } catch (error) {
        console.error(error);
        commandList.innerHTML = `<li class="list-group-item text-danger bg-dark text-light">Failed to load commands</li>`;
    }
}

// Reset input fields
const resetInputs = () => {
    deviceTypeInput.value = "";
    inputContainer.innerHTML = `
    <label for="command0" class="form-label" style="margin-top: 10px;">
        <h5>
            Commands:
        </h5>
    </label>
    <input type="text" id="command0" name="command0" class="form-control bg-dark text-light" placeholder="For example: turn on" required>`;
    inputCount = 1;
    firstCommandInput = document.getElementById("command0") as HTMLInputElement;
    submissionText.innerHTML = "";
    inputContainer.appendChild(firstCommandInput);
}

// Function to modify a command
const modifyCommand = (device: string, commands: string[]) => {
    toggleButton.click();
    resetInputs();
    deviceTypeInput.value = device;
    
    commands.forEach(command => {
        const input = document.getElementById(`command${inputCount - 1}`) as HTMLInputElement;
        input.value = command;
        addInputField();
    });
    setSubmissionText();
}

// Function to delete a command
const deleteCommand = async (commandKey: string): Promise<void> => {
    try {
        const response = await fetch(`${backendURL}/api/commands/${commandKey}`, {
            method: 'DELETE',
        });
        if (!response.ok) throw new Error(`Failed to delete command: ${response.statusText}`);

        // Reload commands after successful deletion
        await loadCommands();
    } catch (error) {
        console.error(error);
        alert(`Error deleting command: ${commandKey}`);
    }
}

// Add command form submission
commandForm.addEventListener("submit", async (event) => {
    event.preventDefault();

    const deviceType = deviceTypeInput.value;
    const commands: string[] = [];

    const inputs = document.querySelectorAll("input");
    inputs.forEach((input: HTMLInputElement) => {
        if (input.id === "deviceType" || input.value.trim() === "") return;
        commands.push(input.value.trim());
    });

    try {
        const response = await fetch(`${backendURL}/add-command`, {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ deviceType, commands })
        });

        if (response.ok) {
            message.innerText = "Commands set successfully!";
            resetInputs();
            await loadCommands();
        } else {
            message.innerText = "Failed to add command.";
        }
    } catch (error) {
        message.innerText = "Error: Could not connect to server.";
    }
});

// Set the submission text
const setSubmissionText = () => {
    const inputs = document.querySelectorAll("input");
    if (deviceTypeInput.value.trim() === "" || firstCommandInput.value.trim() === "") return;
    
    submissionText.innerHTML = "<h3>Individual commands:</h3>";
    submissionText.innerHTML += "<h6>The keyword <i>that</i> indicates a gesture is expected</h6>";
    let count = 1;
    inputs.forEach((input: HTMLInputElement) => {
        if (input.id === "deviceType") return;
        if (input.value.trim() !== "") {
            submissionText.innerHTML += `<p><strong>Command ${count}:</strong> ${input.value} that ${deviceTypeInput.value}</p>`;
        }
        count++;
    });
    

    submissionText.innerHTML += `<h3>Commands for all ${deviceTypeInput.value}s:</h3>`;
    count = 1;
    inputs.forEach((input: HTMLInputElement) => {
        if (input.id === "deviceType") return;
        if (input.value.trim() !== "") {
            submissionText.innerHTML += `<p><strong>Command ${count}:</strong> ${input.value} all ${deviceTypeInput.value}s</p>`;
        }
        count++;
    }); 
};

// Add a new input field
const addInputField = () => {
    const previousInput = document.getElementById(`command${inputCount - 1}`) as HTMLInputElement;
    if (previousInput.value.trim() === "") return;

    const newInput = document.createElement("input");
    newInput.type = "text";
    newInput.classList.add("form-control");
    newInput.id = `command${inputCount}`;
    newInput.name = `command${inputCount}`;
    newInput.addEventListener("input", setSubmissionText);
    newInput.classList.add("form-control", "bg-dark", "text-light");
    if (inputCount === 1) newInput.placeholder = "For example: turn off";
    if (inputCount === 2) newInput.placeholder = "For example: turn up";
    if (inputCount === 3) newInput.placeholder = "For example: turn down";
    inputContainer.appendChild(newInput);
    inputCount++;
};

firstCommandInput.addEventListener("input", setSubmissionText);
deviceTypeInput.addEventListener("input", setSubmissionText);
addButton.addEventListener("click", addInputField);

// run these on load
loadCommands();
setSubmissionText();
