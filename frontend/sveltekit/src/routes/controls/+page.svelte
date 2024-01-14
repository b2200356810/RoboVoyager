<script>
	import { rosStore } from '$lib/stores/rosStore.js';
	import { onMount, onDestroy } from 'svelte';
	import ROSLIB from 'roslib';

	let ros;
	let terminalTopicListener;
	onMount(() => {
		const unsubscribe = rosStore.subscribe((value) => {
			if (value !== null) {
				ros = value;
				console.log('Inside controls');
				subscribeToTerminalTopic();
			}
		});
	});

	onDestroy(() => {
		if (terminalTopicListener) {
			terminalTopicListener.unsubscribe();
		}
	});

	function subscribeToTerminalTopic() {
		terminalTopicListener = new ROSLIB.Topic({
			ros,
			name: '/terminal_topic',
			messageType: 'std_msgs/String'
		});

		terminalTopicListener.subscribe((message) => {
			console.log(message.data);
		});
	}
</script>
