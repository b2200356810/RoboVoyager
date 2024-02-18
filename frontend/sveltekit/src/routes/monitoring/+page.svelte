<script>
	import { onMount, onDestroy } from 'svelte';
	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';
	import MonitorWindow from '$lib/components/MonitorWindow.svelte';

	let ros;
	let terminalTopicListener;
	let terminalTopicData;
	let helloWorldTopicListener;
	let helloWorldTopicData;
	let videoTopicListener;
	let videoTopicData;

	let isModalOpen = false;
	let selectedTopics = [
		{ topicID: 0, topicName: 'default' },
		{ topicID: 1, topicName: '/terminal_topic' },
		// { topicID: 2, topicName: '/hello_world_topic' },
		{ topicID: 3, topicName: '/video_streaming_topic' }
	];

	onMount(() => {
		const unsubscribe = rosStore.subscribe((value) => {
			if (value !== null) {
				ros = value;
				console.log('Inside monitoring');
				subscribeToTerminalTopic();
				subscribeToHelloWorldTopic();
				subscribeToVideoStreamingTopic();
			}
		});

		const storedTopics = localStorage.getItem('selectedTopics');
		if (storedTopics) selectedTopics = JSON.parse(storedTopics);
	});

	onDestroy(() => {
		if (terminalTopicListener) {
			terminalTopicListener.unsubscribe();
		}
		if (helloWorldTopicListener) {
			helloWorldTopicListener.unsubscribe();
		}
		if (videoTopicListener) {
			videoTopicListener.unsubscribe();
		}
	});

	function subscribeToTerminalTopic() {
		terminalTopicListener = new ROSLIB.Topic({
			ros,
			name: '/terminal_topic',
			messageType: 'std_msgs/String'
		});

		terminalTopicListener.subscribe(
			(message) => {
				terminalTopicData = `${getTimestamp()} - ${message.data}`;
			},
			(error) => {
				// Handle error (e.g., topic stops publishing)
				console.error(`Error in /terminal_topic subscription: ${error}`);
				console.log(error);
				// Add your logic here to inform the user or take appropriate action
			}
		);
	}

	function subscribeToHelloWorldTopic() {
		helloWorldTopicListener = new ROSLIB.Topic({
			ros,
			name: '/hello_world_topic',
			messageType: 'std_msgs/String'
		});

		helloWorldTopicListener.subscribe((message) => {
			helloWorldTopicData = `${getTimestamp()} - ${message.data}`;
		});
	}

	function subscribeToVideoStreamingTopic() {
		videoTopicListener = new ROSLIB.Topic({
			ros: ros,
			name: '/video_streaming_topic',
			messageType: 'sensor_msgs/Image',
			// messageType: 'sensor_msgs/CompressedImage',
			transportHint: 'udp'
		});

		videoTopicListener.subscribe(function (message) {
			videoTopicData = message.data;
			// console.log(message.data.length / 1000);
			// console.log(message);
		});
	}

	function getTimestamp() {
		const now = new Date();
		const hour = String(now.getHours()).padStart(2, '0');
		const minute = String(now.getMinutes()).padStart(2, '0');
		const timestamp = `[${hour}:${minute}]`;
		return timestamp;
	}
	function openModal() {
		isModalOpen = true;
	}

	function closeModal() {
		isModalOpen = false;
	}

	function confirmTopic() {
		const selectedTopicName = document.getElementById('topicSelect').value;
		const existingTopic = selectedTopics.find((topic) => topic.topicName === selectedTopicName);

		console.log(existingTopic);

		if (existingTopic) {
			alert('Topic exists already');
			return;
		} else {
			const topicID = generateTopicID();
			const newTopic = {
				topicID,
				topicName: selectedTopicName
			};
			selectedTopics = [...selectedTopics, newTopic];
			localStorage.setItem('selectedTopics', JSON.stringify(selectedTopics));
		}

		// if (!existingTopic) {
		// 	// Assign a unique topicID
		// 	const topicID = generateTopicID();

		// 	// Create a new topic object
		// 	const newTopic = {
		// 		topicID,
		// 		topicName: selectedTopicName
		// 	};

		// 	// Update selectedTopics
		// 	selectedTopics = [...selectedTopics, newTopic];

		// 	// Save to localStorage
		// 	localStorage.setItem('selectedTopics', JSON.stringify(selectedTopics));
		// }

		closeModal();
	}

	// Function to generate a unique topicID
	function generateTopicID() {
		return Math.random().toString(36).substr(2, 9); // You can use a more robust method for generating unique IDs
	}
</script>

<dialog class="dialog-topic" style={`display: ${isModalOpen ? 'flex' : 'none'}`}>
	<form>
		<label>
			Topic:
			<select id="topicSelect">
				<option value="default">Choose…</option>
				<option>/terminal_topic</option>
				<option>/hello_world_topic</option>
				<option>/video_streaming_topic</option>
			</select>
		</label>
		<div class="dialog-button-container">
			<button class="dialog-button" on:click={closeModal} value="cancel" formmethod="dialog"
				>Cancel</button
			>
			<button class="dialog-button" on:click={confirmTopic} id="confirmBtn" value="default"
				>Confirm</button
			>
		</div>
	</form>
</dialog>

<div class="topic-add-container">
	<button on:click={openModal}>Add a node</button>
</div>

<div class="monitor-container">
	<MonitorWindow monitorName="/terminal_topic" monitorContent={terminalTopicData}></MonitorWindow>
	<MonitorWindow monitorName="/hello_world_topic" monitorContent={helloWorldTopicData}
	></MonitorWindow>
	<MonitorWindow monitorName="/video_streaming_topic" monitorContent={videoTopicData}
	></MonitorWindow>
</div>

<style>
	:root {
		--add-button-background: rgb(222, 222, 222);
	}

	:global(.dark-theme) {
		--add-button-background: #2e2e2e;
	}

	.dialog-topic {
		width: 100%;
		background: rgba(0, 0, 0, 0.2);
		border: none;
		display: flex;
		align-items: center;
		justify-content: center;
		height: calc(100dvh - 80px);
	}

	.dialog-topic form {
		display: flex;
		flex-direction: column;
		gap: 10px;
		background-color: gray;
		padding: 20px;
		border-radius: 10px;
		font-size: 2.5rem;
	}

	.dialog-topic select {
		padding: 5px;
		font-size: 2rem;
	}

	.dialog-topic option {
		/* color: rebeccapurple; */
	}

	.dialog-button-container {
		display: flex;
		gap: 10px;
		justify-content: center;
	}

	.dialog-button {
		font-size: 2rem;
		padding: 5px;
		align-self: middle;
		border-radius: 5px;
		border: none;
		/* color: var(--text-color); */
		/* background-color: var(--add-button-background); */
	}
	.dialog-button:hover {
		scale: 0.98;
	}

	.topic-add-container {
		display: flex;
		justify-content: center;
		padding: 20px;
		align-items: middle;
	}

	.topic-add-container button {
		background-color: var(--add-button-background);
		border: none;
		color: var(--text-color);
		padding: 20px;
		font-size: 2.5rem;
		border-radius: 10px;
		box-shadow: 0px 0px 5px 0px rgba(0, 0, 0, 0.75);
		transition: var(--theme-transition-time);
		cursor: pointer;
	}

	.topic-add-container button:hover {
		background-color: var(--text-color);
		color: var(--add-button-background);
	}

	.monitor-container {
		display: flex;
		/* display: none; */
		justify-content: center;
		align-items: center;
		gap: 20px;
		padding: 20px;
		flex-wrap: wrap;
	}
</style>
