<script>
	import { onMount, onDestroy } from 'svelte';
	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';
	import * as GMaps from '@googlemaps/js-api-loader';
	const { Loader } = GMaps;

	let ros;

	let helloWorldTopicListener;
	let helloWorldTopicData;
	let terminalTopicListener;
	let terminalTopicData;
	let videoTopicListener;
	let videoTopicData;

	let controlsTopicListener;

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

	let showAI = false;
	let showMap = true;
	let showSensors = true;
	let showControls = true;
	let showCapture = false;
	let showVideo = false;

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
				// subscribeToVideoStreamingTopic();
				subscribeToControlsTopic();
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

	function subscribeToTerminalTopic() {
		terminalTopicListener = new ROSLIB.Topic({
			ros,
			name: '/terminal_topic',
			messageType: 'std_msgs/String'
		});

		terminalTopicListener.subscribe(
			(message) => {
				terminalTopicData = `${getTimestamp()} - ${message.data}`;
				console.log(terminalTopicData);
			},
			(error) => {
				console.error(`Error in /terminal_topic subscription: ${error}`);
				console.log(error);
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
			messageType: 'sensor_msgs/CompressedImage'
		});

		videoTopicListener.subscribe(
			(message) => {
				// console.log(message);
				videoTopicData = message.data;
				// console.log(message.data.length / 1000);
				// console.log(message);
			},
			(error) => {
				console.error(`Error in /terminal_topic subscription: ${error}`);
				console.log(error);
			}
		);
	}

	function getTimestamp() {
		const now = new Date();
		const hour = String(now.getHours()).padStart(2, '0');
		const minute = String(now.getMinutes()).padStart(2, '0');
		const timestamp = `[${hour}:${minute}]`;
		return timestamp;
	}

	const loadNippleJs = async () => {
		const { default: Nipple } = await import('nipplejs');

		joystick = Nipple.create({
			zone: document.getElementById('joystick-container'),
			mode: 'static',
			position: { left: '50%', top: '50%' },
			color: 'red',
			size: 80,
			restOpacity: 0.8
		});

		joystick.on('move', (evt, data) => {
			updateDebugFields(data);
			publishJoystickCommands(data);
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

	function subscribeToControlsTopic() {
		controlsTopicListener = new ROSLIB.Topic({
			ros,
			name: '/controls_topic',
			messageType: 'sensor_msgs/Joy'
		});
	}

	function publishJoystickCommands(data) {
		// console.log(typeof(data.position.y), data.position.y)
		if (controlsTopicListener) {
			let x = parseFloat(data.position.x);
			let y = parseFloat(data.position.y);

			const axes = [x, y];
			const buttons = [];
			const joyMessage = new ROSLIB.Message({ axes });
			controlsTopicListener.publish(joyMessage);
		}
	}
</script>

<div class="robot-container">
	<div class="robot-container-header">
		<nav class="robot-container-header-navbar">
			<button
				style="color: {showAI ? 'green' : ''}; opacity:.5;"
				on:click={() => {
					showAI = !showAI;
				}}>AI</button
			>
			<button
				style="color: {showMap ? 'green' : ''}"
				on:click={() => {
					showMap = !showMap;
				}}>Map</button
			>
			<button
				style="color: {showSensors ? 'green' : ''}"
				on:click={() => {
					showSensors = !showSensors;
				}}
				>Sensors
			</button>
			<button
				style="color: {showControls ? 'green' : ''}"
				on:click={() => {
					showControls = !showControls;
				}}>Controls</button
			>
			<button
				style="color: {showCapture ? 'green' : ''}"
				on:click={() => {
					showCapture = !showCapture;
				}}>Capture</button
			>
			<button
				style="color: {showVideo ? 'green' : 'red'}"
				on:click={(event) => {
					showVideo = !showVideo;
					if (showVideo) {
						subscribeToVideoStreamingTopic();
					} else {
						videoTopicListener.unsubscribe();
					}
				}}>Video</button
			>
		</nav>
	</div>
	<div class="robot-container-footer">
		<img
			class="robot-container-footer-video"
			src="data:image/jpeg;base64,{videoTopicData}"
			alt=""
			style="visibility: {showVideo ? 'visible' : 'hidden'}"
		/>

		<!-- <p id="error-text" style="display: none;">Image not available</p> -->
		<ul
			class="robot-container-footer-sensors"
			style="visibility: {showSensors ? 'visible' : 'hidden'}"
		>
			<li>
				<span
					><svg
						xmlns="http://www.w3.org/2000/svg"
						width="20"
						height="20"
						fill="currentColor"
						viewBox="0 0 16 16"
						style="vertical-align: middle;"
					>
						<path
							d="M5 0a.5.5 0 0 1 .5.5V2h1V.5a.5.5 0 0 1 1 0V2h1V.5a.5.5 0 0 1 1 0V2h1V.5a.5.5 0 0 1 1 0V2A2.5 2.5 0 0 1 14 4.5h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14a2.5 2.5 0 0 1-2.5 2.5v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14A2.5 2.5 0 0 1 2 11.5H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2A2.5 2.5 0 0 1 4.5 2V.5A.5.5 0 0 1 5 0m-.5 3A1.5 1.5 0 0 0 3 4.5v7A1.5 1.5 0 0 0 4.5 13h7a1.5 1.5 0 0 0 1.5-1.5v-7A1.5 1.5 0 0 0 11.5 3zM5 6.5A1.5 1.5 0 0 1 6.5 5h3A1.5 1.5 0 0 1 11 6.5v3A1.5 1.5 0 0 1 9.5 11h-3A1.5 1.5 0 0 1 5 9.5zM6.5 6a.5.5 0 0 0-.5.5v3a.5.5 0 0 0 .5.5h3a.5.5 0 0 0 .5-.5v-3a.5.5 0 0 0-.5-.5z"
						/>
					</svg> CPU</span
				>
			</li>
			<li>
				<span
					><svg
						xmlns="http://www.w3.org/2000/svg"
						width="20"
						height="20"
						fill="currentColor"
						viewBox="0 0 16 16"
						style="vertical-align: middle;"
					>
						<path
							d="M4 8a1.5 1.5 0 1 1 3 0 1.5 1.5 0 0 1-3 0m7.5-1.5a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3"
						/>
						<path
							d="M0 1.5A.5.5 0 0 1 .5 1h1a.5.5 0 0 1 .5.5V4h13.5a.5.5 0 0 1 .5.5v7a.5.5 0 0 1-.5.5H2v2.5a.5.5 0 0 1-1 0V2H.5a.5.5 0 0 1-.5-.5m5.5 4a2.5 2.5 0 1 0 0 5 2.5 2.5 0 0 0 0-5M9 8a2.5 2.5 0 1 0 5 0 2.5 2.5 0 0 0-5 0"
						/>
						<path
							d="M3 12.5h3.5v1a.5.5 0 0 1-.5.5H3.5a.5.5 0 0 1-.5-.5zm4 1v-1h4v1a.5.5 0 0 1-.5.5h-3a.5.5 0 0 1-.5-.5"
						/>
					</svg> GPU</span
				>
			</li>
			<li>
				<span>
					<svg
						xmlns="http://www.w3.org/2000/svg"
						width="20"
						height="20"
						fill="currentColor"
						viewBox="0 0 16 16"
						style="vertical-align: middle;"
					>
						<path d="M2 6h10v4H2z" />
						<path
							d="M2 4a2 2 0 0 0-2 2v4a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V6a2 2 0 0 0-2-2zm10 1a1 1 0 0 1 1 1v4a1 1 0 0 1-1 1H2a1 1 0 0 1-1-1V6a1 1 0 0 1 1-1zm4 3a1.5 1.5 0 0 1-1.5 1.5v-3A1.5 1.5 0 0 1 16 8"
						/>
					</svg> N/A
				</span>
			</li>

			<li>
				<span
					><svg
						xmlns="http://www.w3.org/2000/svg"
						width="20"
						height="20"
						fill="currentColor"
						viewBox="0 0 16 16"
						style="vertical-align: middle;"
					>
						<path
							d="M8 0a8 8 0 1 0 0 16A8 8 0 0 0 8 0M2.04 4.326c.325 1.329 2.532 2.54 3.717 3.19.48.263.793.434.743.484q-.121.12-.242.234c-.416.396-.787.749-.758 1.266.035.634.618.824 1.214 1.017.577.188 1.168.38 1.286.983.082.417-.075.988-.22 1.52-.215.782-.406 1.48.22 1.48 1.5-.5 3.798-3.186 4-5 .138-1.243-2-2-3.5-2.5-.478-.16-.755.081-.99.284-.172.15-.322.279-.51.216-.445-.148-2.5-2-1.5-2.5.78-.39.952-.171 1.227.182.078.099.163.208.273.318.609.304.662-.132.723-.633.039-.322.081-.671.277-.867.434-.434 1.265-.791 2.028-1.12.712-.306 1.365-.587 1.579-.88A7 7 0 1 1 2.04 4.327Z"
						/>
					</svg> GPS</span
				>
			</li>
			<li>
				<span
					><svg
						xmlns="http://www.w3.org/2000/svg"
						width="20"
						height="20"
						fill="currentColor"
						viewBox="0 0 16 16"
						style="vertical-align: middle;"
					>
						<path
							d="M4 11H2v3h2zm5-4H7v7h2zm5-5v12h-2V2zm-2-1a1 1 0 0 0-1 1v12a1 1 0 0 0 1 1h2a1 1 0 0 0 1-1V2a1 1 0 0 0-1-1zM6 7a1 1 0 0 1 1-1h2a1 1 0 0 1 1 1v7a1 1 0 0 1-1 1H7a1 1 0 0 1-1-1zm-5 4a1 1 0 0 1 1-1h2a1 1 0 0 1 1 1v3a1 1 0 0 1-1 1H2a1 1 0 0 1-1-1z"
						/>
					</svg> Wifi:</span
				>
			</li>
			<li>
				<span
					><svg
						xmlns="http://www.w3.org/2000/svg"
						width="20"
						height="20"
						fill="currentColor"
						viewBox="0 0 16 16"
						style="vertical-align: middle;"
					>
						<path
							d="m7.788 2.34-.799 1.278A.25.25 0 0 0 7.201 4h1.598a.25.25 0 0 0 .212-.382l-.799-1.279a.25.25 0 0 0-.424 0Zm0 11.32-.799-1.277A.25.25 0 0 1 7.201 12h1.598a.25.25 0 0 1 .212.383l-.799 1.278a.25.25 0 0 1-.424 0ZM3.617 9.01 2.34 8.213a.25.25 0 0 1 0-.424l1.278-.799A.25.25 0 0 1 4 7.201V8.8a.25.25 0 0 1-.383.212Zm10.043-.798-1.277.799A.25.25 0 0 1 12 8.799V7.2a.25.25 0 0 1 .383-.212l1.278.799a.25.25 0 0 1 0 .424Z"
						/>
						<path
							d="M6.5 0A1.5 1.5 0 0 0 5 1.5v3a.5.5 0 0 1-.5.5h-3A1.5 1.5 0 0 0 0 6.5v3A1.5 1.5 0 0 0 1.5 11h3a.5.5 0 0 1 .5.5v3A1.5 1.5 0 0 0 6.5 16h3a1.5 1.5 0 0 0 1.5-1.5v-3a.5.5 0 0 1 .5-.5h3A1.5 1.5 0 0 0 16 9.5v-3A1.5 1.5 0 0 0 14.5 5h-3a.5.5 0 0 1-.5-.5v-3A1.5 1.5 0 0 0 9.5 0zM6 1.5a.5.5 0 0 1 .5-.5h3a.5.5 0 0 1 .5.5v3A1.5 1.5 0 0 0 11.5 6h3a.5.5 0 0 1 .5.5v3a.5.5 0 0 1-.5.5h-3a1.5 1.5 0 0 0-1.5 1.5v3a.5.5 0 0 1-.5.5h-3a.5.5 0 0 1-.5-.5v-3A1.5 1.5 0 0 0 4.5 10h-3a.5.5 0 0 1-.5-.5v-3a.5.5 0 0 1 .5-.5h3A1.5 1.5 0 0 0 6 4.5z"
						/>
					</svg> Pos</span
				>
			</li>
		</ul>
		<div class="robot-container-footer-wrapper">
			<div class="empty"></div>
			<div id="joystick-container" style="visibility: {showControls ? 'visible' : 'hidden'}"></div>
			<div class="robot-container-navigation" style="visibility: {showMap ? 'visible' : 'hidden'}">
				<!-- <div>GPS</div> -->
				<div bind:this={mapElement}>gps</div>
			</div>
		</div>
	</div>
</div>

<style>
	.robot-container {
		height: calc(100dvh - 80px);
		font-size: 2rem;
		display: flex;
		flex-direction: column;
	}

	.robot-container-header-navbar {
		display: flex;
		padding: 10px;
		justify-content: center;
		gap: 20px;
		border-bottom: 1px solid var(--text-color);
		z-index: 1;
		flex-wrap: wrap;
	}

	.robot-container-header-navbar button {
		background: none;
		color: var(--text-color);
		border: none;
		padding: 5px;
		font-size: 2rem;
	}

	.robot-container-header-navbar button:hover {
		color: yellow !important;
		transition: var(--theme-transition-time);
	}

	.robot-container-footer {
		height: 100%;
		display: flex;
		flex-direction: column;
		justify-content: end;
		position: relative;
	}

	.robot-container-footer-video {
		position: absolute;
		height: 100%;
		width: 100%;
		/* display: none; */
		/* height: calc(100dvh - 134px); */
		/* width: 100%; */
	}

	.robot-container-footer-sensors {
		position: absolute;
		top: 0;
		padding: 10px;
		background: var(--background-color);
		/* background: green; */
		transition: var(--theme-transition-time);
		list-style: none;
		line-height: 3rem;
	}

	.robot-container-footer-wrapper {
		display: flex;
		height: 120px;
		justify-content: space-between;
		padding: 10px;
		gap: 20px;
	}

	.robot-container-footer-wrapper div {
		/* border: 1px solid green; */
		text-align: center;
		width: 100px;
		height: 100px;
	}

	#joystick-container {
		position: relative;
		border-radius: 50%;
	}

	.robot-container-navigation div {
		border: 1px solid black;
	}
</style>
