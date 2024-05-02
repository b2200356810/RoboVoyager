<script>
	import { onMount, onDestroy } from 'svelte';
	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';
	// import { Loader } from '@googlemaps/js-api-loader';
	import * as GMaps from '@googlemaps/js-api-loader';
	const { Loader } = GMaps;

	console.log('Robot');

	let ros;
	let terminalTopicListener;
	let terminalTopicData;
	let helloWorldTopicListener;
	let helloWorldTopicData;
	let videoTopicListener;
	let videoTopicData;

	let joystick;
	let debugPositionX = '-';
	let debugPositionY = '-';
	let debugForce = '-';
	let debugPressure = '-';
	let debugDistance = '-';
	let debugAngleRadian = '-';
	let debugAngleDegree = '-';
	let debugDirectionX = '-';
	let debugDirectionY = '-';
	let debugDirectionAngle = '-';

	let mapElement;

	onMount(async () => {
		loadNippleJs();

		const loader = new Loader({
			apiKey: 'AIzaSyCU6uBfjvi2BBu596I8SdGg736LkupkvpY',
			version: 'weekly'
		});

		const { Map } = await loader.importLibrary('maps');

		let map = new Map(mapElement, {
			center: { lat: 39.9334, lng: 32.8597 },
			zoom: 12
		});

		const unsubscribe = rosStore.subscribe((value) => {
			if (value !== null) {
				ros = value;
				// subscribeToTerminalTopic();
				// subscribeToHelloWorldTopic();
				subscribeToVideoStreamingTopic();
			}
		});
	});

	onDestroy(() => {
		if (joystick) {
			joystick.destroy();
		}
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

	const loadNippleJs = async () => {
		const { default: Nipple } = await import('nipplejs');

		joystick = Nipple.create({
			zone: document.getElementById('joystick-container'),
			mode: 'static',
			position: { left: '50%', top: '50%' },
			color: 'red',
			restOpacity: 0.8
		});

		joystick.on('move', (evt, data) => {
			updateDebugFields(data);
			// publishJoystickCommands(data);
		});

		joystick.on('end', () => {
			resetDebugFields();
		});
	};

	const updateDebugFields = (data) => {
		debugPositionX = data.position.x.toFixed(2);
		debugPositionY = data.position.y.toFixed(2);
		debugForce = data.force.toFixed(2);
		debugPressure = data.pressure.toFixed(2);
		debugDistance = data.distance.toFixed(2);
		debugAngleRadian = data.angle.radian.toFixed(2);
		debugAngleDegree = data.angle.degree.toFixed(2);
		debugDirectionX = data.direction.x !== undefined ? data.direction.x : '-';
		debugDirectionY = data.direction.y !== undefined ? data.direction.y : '-';
		debugDirectionAngle = data.direction.angle !== undefined ? data.direction.angle : '-';
	};

	const resetDebugFields = () => {
		debugPositionX = '-';
		debugPositionY = '-';
		debugForce = '-';
		debugPressure = '-';
		debugDistance = '-';
		debugAngleRadian = '-';
		debugAngleDegree = '-';
		debugDirectionX = '-';
		debugDirectionY = '-';
		debugDirectionAngle = '-';
	};

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
</script>

<div class="robot-container">
	<div class="robot-container-navbar">
		<button>AI</button>
		<button>Map</button>
		<button>Sensors</button>
		<button>Controls</button>
		<button>Capture</button>
		<button>Video</button>
	</div>
	<img class="robot-container-video-stream" src="data:image/jpeg;base64,{videoTopicData}" alt="" />
	<div class="robot-container-wrapper">
		<div class="robot-container-navigation">
			<div
				bind:this={mapElement}
				style:height={'100%'}
				style:width={'100%'}
				style="border-radius: 50%;"
			/>
		</div>
		<div class="aligning-container">
			<div id="joystick-container"></div>
		</div>

		<!-- <div id="debug">
			<ul>
				<li class="position">
					position :
					<ul>
						<li class="x">x : {debugPositionX}</li>
						<li class="y">y : {debugPositionY}</li>
					</ul>
				</li>
				<li class="force">force : {debugForce}</li>
				<li class="pressure">pressure : {debugPressure}</li>
				<li class="distance">distance : {debugDistance}</li>
				<li class="angle">
					angle :
					<ul>
						<li class="radian">radian : {debugAngleRadian}</li>
						<li class="degree">degree : {debugAngleDegree}</li>
					</ul>
				</li>
				<li class="direction">
					direction :
					<ul>
						<li class="x">x : {debugDirectionX}</li>
						<li class="y">y : {debugDirectionY}</li>
						<li class="angle">angle : {debugDirectionAngle}</li>
					</ul>
				</li>
			</ul>
		</div> -->

		<div class="robot-container-sensors">
			sensors
			<div>GPS</div>
			<div>Battery</div>
			<div>Heat</div>
		</div>
	</div>
</div>

<style>
	.robot-container {
		height: calc(100dvh - 80px);
		font-size: 2rem;
		display: flex;
		flex-direction: column;
		justify-content: space-between;
	}

	.robot-container-navbar {
		display: flex;
		/* color: blue; */
		padding: 10px;
		justify-content: center;
		gap: 20px;
		border-bottom: 1px solid var(--text-color);
		z-index: 1;
	}

	.robot-container-navbar button {
		background: none;
		color: var(--text-color);
		border: none;
		padding: 5px;
		font-size: 2rem;
	}

	.robot-container-navbar button:hover {
		color: red;
		transition: var(--theme-transition-time);
	}

	.robot-container-video-stream {
		position: absolute;
		height: calc(100dvh - 80px);
		width: 100%;
	}

	.robot-container-wrapper {
		display: flex;
		justify-content: space-between;
		gap: 20px;
		padding: 20px;
	}

	.robot-container-navigation,
	.robot-container-sensors {
		border: 1px solid green;
		text-align: center;
		width: 100px;
	}

	/* .robot-container-navigation {
		border-radius: 50%;
	} */

	#debug {
		position: absolute;
		top: 0;
		left: 0;
		z-index: 10;
		width: 100%;
		height: 100%;
		display: flex;
		justify-content: center;
		align-items: center;
		font-size: 2.5rem;
	}

	.aligning-container {
		/* height: calc(100dvh - 80px); */
		display: flex;
		justify-content: center;
		align-items: center;
	}

	#joystick-container {
		position: relative;
		width: 100px;
		height: 100px;
		/* background-color: rgba(255, 0, 0, 0.5); */
		border-radius: 50%;
	}
</style>
