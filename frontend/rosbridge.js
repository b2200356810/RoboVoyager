//CHANGE THE IP LATER
const ros = new ROSLIB.Ros({ url: "ws:192.168.1.23:9090" });

function connectToROS() {
  ros.on("connection", () => { document.getElementById("status").innerText = "successful"; });
  ros.on("error", (error) => { document.getElementById("status").innerText = `errored out (${error})`; });
  ros.on("close", () => { document.getElementById("status").innerText = "closed"; });
}

const messageForm = document.getElementById("message-form");
messageForm.addEventListener("submit", function (event) {
  event.preventDefault();

  const topic = document.getElementById("topic").value;
  const messageData = document.getElementById("message").value;
  const message = new ROSLIB.Message({ data: messageData });

  const publisher = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: "std_msgs/String"
  });
  publisher.publish(message);
});

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
    const hour = String(now.getHours()).padStart(2, '0');
    const minute = String(now.getMinutes()).padStart(2, '0');
    const timestamp = `[${hour}:${minute}]`;

    newMessage.appendChild(document.createTextNode(`${timestamp} - ${message.data}`));
    ul.appendChild(newMessage);
  });
}

function subscribeToReplyTopic() {
  const replyTopicListener = new ROSLIB.Topic({
    ros: ros,
    name: "/first_node_topic",
    messageType: 'std_msgs/String'
  });

  replyTopicListener.subscribe(function (message) {

    console.log(message)

    const replyTopic = document.getElementById("reply-topic");
    replyTopic.innerText = "first_node_topic";

    const ul = document.getElementById("reply-messages");
    const newMessage = document.createElement("li");

    const now = new Date();
    const hour = String(now.getHours()).padStart(2, '0');
    const minute = String(now.getMinutes()).padStart(2, '0');
    const timestamp = `[${hour}:${minute}]`;

    newMessage.appendChild(document.createTextNode(`${timestamp} - ${message.data}`));
    ul.appendChild(newMessage);
  });
}

connectToROS();
subscribeToTerminalTopic();
subscribeToReplyTopic();

// Doesn't work due to no event triggering the loop
// Only runs once after the page loads
// ros.getTopics((topics) => {
//   topics.topics.forEach((topic) => {
//     const dynamicTopicListener = new ROSLIB.Topic({
//       ros: ros,
//       name: topic,
//       messageType: 'std_msgs/String'
//     });

//     const now = new Date();
//     const hour = String(now.getHours()).padStart(2, '0');
//     const minute = String(now.getMinutes()).padStart(2, '0');
//     const timestamp = `[${hour}:${minute}]`;

//     const newMessage = document.createElement("li");

//     switch (topic) {
//       case "/terminal_topic":
//         dynamicTopicListener.subscribe(function (message) {
//           const ulId = "terminal-messages";
//           const ul = document.getElementById(ulId);
//           newMessage.appendChild(document.createTextNode(`${timestamp} - ${message.data}`));
//           ul.appendChild(newMessage);
//         });
//         console.log("In terminal topics")
//         break;

//       case "/first_node_topic":
//         dynamicTopicListener.subscribe(function (message) {
//           const ulId = "reply-messages";
//           const ul = document.getElementById(ulId);
//           newMessage.appendChild(document.createTextNode(`${timestamp} - ${message.data}`));
//           ul.appendChild(newMessage);
//         });
//         console.log("In reply topics")
//         break;
//     }
//   });
// });