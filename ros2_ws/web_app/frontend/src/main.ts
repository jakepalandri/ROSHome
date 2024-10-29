document.getElementById("commandForm")!.addEventListener("submit", async (event) => {
    event.preventDefault();
    
    const command = (document.getElementById("command") as HTMLInputElement).value;
    const action = (document.getElementById("action") as HTMLInputElement).value;
    // const backendURL = "http://192.168.0.100:5000"; // on lab router network
    const backendURL = "http://192.168.0.186:5000"; // on home network

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
        } else {
            document.getElementById("message")!.innerText = "Failed to add command.";
        }
    } catch (error) {
        document.getElementById("message")!.innerText = "Error: Could not connect to server.";
    }
});
