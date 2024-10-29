// Variables
const backendURL = "http://192.168.0.100:5000"; // on lab router network
// const backendURL = "http://192.168.0.186:5000"; // on home network
const toggleButton = document.getElementById("toggleButton")!;
const addCommandsDiv = document.getElementById("addCommandsDiv")!;
const viewRemoveCommandsDiv = document.getElementById("viewRemoveCommandsDiv")!;
const commandList = document.getElementById("commandList")!;
const commandInput = document.getElementById("command") as HTMLInputElement;
const actionInput = document.getElementById("action") as HTMLInputElement;

loadCommands();

toggleButton.addEventListener("click", async () => {
    // Toggle visibility of divs
    const addCommandsVisible = addCommandsDiv.style.display === "block";
    addCommandsDiv.style.display = addCommandsVisible ? "none" : "block";
    viewRemoveCommandsDiv.style.display = addCommandsVisible ? "block" : "none";

    // Update button text
    toggleButton.textContent = addCommandsVisible ? "Add Commands": "View Commands";
    commandInput.value = "";
    actionInput.value = "";
    await loadCommands();
});

// Load commands from backend and display them in the list
async function loadCommands() {
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
            if (commands[command].includes("that")) valueCell.textContent = `[gesture]_${commands[command]}`;
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

// Function to delete a command
async function deleteCommand(commandKey: string): Promise<void> {
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
document.getElementById("commandForm")!.addEventListener("submit", async (event) => {
    event.preventDefault();

    const command = (document.getElementById("command") as HTMLInputElement).value;
    const action = (document.getElementById("action") as HTMLInputElement).value;

    try {
        const response = await fetch(`${backendURL}/add-command`, {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ command, action })
        });

        if (response.ok) {
            document.getElementById("message")!.innerText = "Command added successfully!";
            commandInput.value = "";
            actionInput.value = "";
            await loadCommands();
        } else {
            document.getElementById("message")!.innerText = "Failed to add command.";
        }
    } catch (error) {
        document.getElementById("message")!.innerText = "Error: Could not connect to server.";
    }
});
