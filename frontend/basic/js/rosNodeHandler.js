let ros;

function connectToIP() {
    ros = new ROSLIB.Ros({ url: `ws:${location.hostname}:9090` });

    ros.on("connection", () => {
        document.getElementById("status").innerText = "successful";
        subscribeToTerminalTopic();
        subscribeToFirstNodeTopic();
        subscribeToMultimediaTopic();
    });

    ros.on("error", (error) => {
        document.getElementById("status").innerText = `errored out (${error})`;
    });

    ros.on("close", () => {
        document.getElementById("status").innerText = "closed";
    });
}

function subscribeToTerminalTopic() {
    const terminalTopicListener = new ROSLIB.Topic({
        ros,
        name: "/terminal_topic",
        messageType: "std_msgs/String",
    });

    terminalTopicListener.subscribe((message) => {
        const ul = document.getElementById("terminal-messages");
        const newMessage = document.createElement("li");

        const now = new Date();
        const hour = String(now.getHours()).padStart(2, "0");
        const minute = String(now.getMinutes()).padStart(2, "0");
        const timestamp = `[${hour}:${minute}]`;

        newMessage.appendChild(
            document.createTextNode(`${timestamp} - ${message.data}`)
        );
        ul.appendChild(newMessage);
    });
}

function subscribeToFirstNodeTopic() {
    const firstNodeTopicListener = new ROSLIB.Topic({
        ros: ros,
        name: "/first_node_topic",
        messageType: "std_msgs/String",
    });

    firstNodeTopicListener.subscribe(function (message) {
        const ul = document.getElementById("first-node-messages");
        const newMessage = document.createElement("li");

        const now = new Date();
        const hour = String(now.getHours()).padStart(2, "0");
        const minute = String(now.getMinutes()).padStart(2, "0");
        const timestamp = `[${hour}:${minute}]`;

        newMessage.appendChild(
            document.createTextNode(`${timestamp} - ${message.data}`)
        );
        ul.appendChild(newMessage);
    });
}

function subscribeToMultimediaTopic() {
    const multimediaTopicListener = new ROSLIB.Topic({
        ros: ros,
        name: "/multimedia_topic",
        messageType: "sensor_msgs/Image",
    });

    const imageElement = document.getElementById("multimedia-image");
    const sizeElement = document.getElementById("multimedia-size");
    let lastTimestamp = 0;

    multimediaTopicListener.subscribe(function (message) {
        const height = message.height;
        const width = message.width;
        // Dividing by 1000 instead of 1024 more accurate
        const sizeIn_Kb = message.data.length / 1000;

        const currentTimestamp =
            message.header.stamp.secs * 1000 + message.header.stamp.nsecs / 1e6;
        const deltaTime = currentTimestamp - lastTimestamp;
        const fps = 1000 / deltaTime;

        sizeElement.innerText = `${height}x${width}
        ${sizeIn_Kb.toFixed(2)} Kb`;

        //FPS: ${fps.toFixed(2)}

        lastTimestamp = currentTimestamp;

        imageElement.src = `data:image/jpeg;base64,${message.data}`;
    });
}
