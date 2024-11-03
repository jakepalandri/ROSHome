// Variables
// const backendURL = "http://192.168.0.100:5000"; // on lab router network
const backendURL = "http://192.168.0.162:5000"; // on home network
const toggleButton = document.getElementById("toggleButton") as HTMLButtonElement;
const addCommandsDiv = document.getElementById("addCommandsDiv") as HTMLDivElement;
const viewRemoveCommandsDiv = document.getElementById("viewRemoveCommandsDiv") as HTMLDivElement;
const thatKeyword = document.getElementById("thatKeyword") as HTMLParagraphElement;
const commandForm = document.getElementById("commandForm") as HTMLFormElement;
const commandList = document.getElementById("commandList") as HTMLTableElement;
const commandInput = document.getElementById("command") as HTMLInputElement;
const actionInput = document.getElementById("action") as HTMLInputElement;
const submissionText = document.getElementById("submission") as HTMLParagraphElement;
const submissionButton = document.getElementById("submit") as HTMLButtonElement;
const message = document.getElementById("message") as HTMLParagraphElement;

toggleButton.addEventListener("click", async () => {
    // Toggle visibility of divs
    const addCommandsVisible = addCommandsDiv.style.display === "block";
    addCommandsDiv.style.display = addCommandsVisible ? "none" : "block";
    viewRemoveCommandsDiv.style.display = addCommandsVisible ? "block" : "none";

    // Update button text
    toggleButton.textContent = addCommandsVisible ? "Add Commands": "View Commands";
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
            const valueCell = document.createElement("td");
            valueCell.textContent = commands[command];
            if (command.includes("that")) valueCell.textContent = `[gesture]_${commands[command]}`;
            row.appendChild(valueCell);
      
            // Create actions cell with delete button
            const actionsCell = document.createElement("td");
            const deleteButton = document.createElement("button");
            deleteButton.className = "btn btn-danger btn-sm";
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

// Set submission text and enable/disable submission button to confirm values to the user
const setSubmissionText = () => {
    if (commandInput.value !== "" && actionInput.value !== "") {
        submissionText.innerHTML = `Command: ${commandInput.value} <br>`;
        if (commandInput.value.includes("that")) {
            submissionText.innerHTML += `Action: [gesture]_${actionInput.value}`;
        }
        else {
            submissionText.innerHTML += `Action: ${actionInput.value}`;
        }
        submissionButton.disabled = false;
    }
    else {
        submissionText.innerHTML = "";
        submissionButton.disabled = true;
    }
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

    const command = commandInput.value;
    const action = actionInput.value;

    try {
        const response = await fetch(`${backendURL}/add-command`, {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ command, action })
        });

        if (response.ok) {
            message.innerText = "Command added successfully!";
            commandInput.value = "";
            actionInput.value = "";
            await loadCommands();
        } else {
            message.innerText = "Failed to add command.";
        }
    } catch (error) {
        message.innerText = "Error: Could not connect to server.";
    }
});

commandInput.addEventListener("input", setSubmissionText);
actionInput.addEventListener("input", setSubmissionText);

commandInput.addEventListener("input", () => {
    if (commandInput.value.includes("that")) {
        thatKeyword.style.color = "lightgreen";
    }
    else {
        thatKeyword.style.color = "white";
    }
});

// run these on load
loadCommands();
setSubmissionText();