<script>
	import { onMount, onDestroy } from 'svelte';
	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';

	let ros;
	let terminalTopicListener;
	onMount(() => {
		const unsubscribe = rosStore.subscribe((value) => {
			if (value !== null) {
				ros = value;
				console.log('Inside home');
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

<div class="home-container">
	<section class="feature-section">
		<h1 class="feature-title">Robo Voyager</h1>
		<blockquote class="feature-description">To infinity, and beyond! - Buzz Lightyear</blockquote>
	</section>

	<section class="feature-section">
		<h2 class="feature-title">Network Connection</h2>
		<p class="feature-description">
			Ensure your device is connected to the same local network. Check the <strong
				>connection</strong
			> status in the navbar.
		</p>
	</section>

	<section class="feature-section">
		<h2 class="feature-title">Monitoring Tab</h2>
		<p class="feature-description">
			View sensor messages and video stream. Subscribe/unsubscribe from topics.
		</p>
	</section>

	<section class="feature-section">
		<h2 class="feature-title">Controls Tab</h2>
		<p class="feature-description">
			Utilize virtual joysticks displayed over a video to manually control a robot's movement.
		</p>
	</section>

	<section class="feature-section">
		<h2 class="feature-title">AI Models</h2>
		<div class="feature-description">
			<ul class="ai-list">
				<li>Object Detection: OpenPCDet</li>
				<li>Segmentation Framework: mmsegmentation</li>
				<li>Anomaly Detection: PEBAL</li>
			</ul>
		</div>
	</section>
</div>

<style>
	.home-container {
		display: flex;
		flex-direction: column;
		gap: 30px;
		padding: 20px;
	}
	.feature-section h1 {
		font-size: 4rem;
	}
	.ai-list {
		padding-left: 20px;
	}

	@media (min-width: 800px) {
		.home-container {
			margin: 20px auto;
			width: 800px;
		}
	}
</style>
