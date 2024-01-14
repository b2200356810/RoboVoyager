<script>
	import { onMount, onDestroy } from 'svelte';
	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';
	import MonitorWindow from '$lib/components/MonitorWindow.svelte';

	let ros;
	let terminalTopicListener;
	let terminalTopicData;
	let videoTopicListener;
	let videoTopicData;

	onMount(() => {
		const unsubscribe = rosStore.subscribe((value) => {
			if (value !== null) {
				ros = value;
				console.log('Inside monitoring');
				subscribeToTerminalTopic();
				subscribeToMultimediaTopic();
			}
		});
	});

	onDestroy(() => {
		if (terminalTopicListener) {
			terminalTopicListener.unsubscribe();
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

		terminalTopicListener.subscribe((message) => {
			const now = new Date();
			const hour = String(now.getHours()).padStart(2, '0');
			const minute = String(now.getMinutes()).padStart(2, '0');
			const timestamp = `[${hour}:${minute}]`;
			// console.log(`${timestamp} - ${message.data}`);
			terminalTopicData = `${timestamp} - ${message.data}`;
		});
	}

	function subscribeToMultimediaTopic() {
		videoTopicListener = new ROSLIB.Topic({
			ros: ros,
			name: '/multimedia_topic',
			messageType: 'sensor_msgs/Image'
		});

		videoTopicListener.subscribe(function (message) {
			videoTopicData = message.data;
		});
	}
</script>

<div class="monitor-container">
	<MonitorWindow monitorName="/terminal_topic" monitorContent={terminalTopicData}></MonitorWindow>
	<MonitorWindow monitorName="/hello_world_topic"></MonitorWindow>
	<MonitorWindow monitorName="/multimedia_topic" monitorContent={videoTopicData}></MonitorWindow>
</div>

<style>
	.monitor-container {
		display: flex;
		justify-content: center;
		align-items: center;
		gap: 20px;
		padding: 20px;
		flex-wrap: wrap;
	}
</style>
