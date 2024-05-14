<script>
	import { onMount, onDestroy } from 'svelte';
	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';
	import * as GMaps from '@googlemaps/js-api-loader';
	const { Loader } = GMaps;

	let ros;

	let terminalTopicListener;
	let terminalTopicData;
	let videoTopicListener;
	let videoTopicData;
	let sensorsTopicListener;
	let sensorsTopicData = {};

	let commandVelocity;

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
				subscribeToTerminalTopic();
				subscribeToControlsTopic();
				subscribeToSensorsTopic();
			}
		});
	});

	onDestroy(() => {
		if (joystick) joystick.destroy();
		if (terminalTopicListener) terminalTopicListener.unsubscribe();
		if (videoTopicListener) videoTopicListener.unsubscribe();
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

	function subscribeToSensorsTopic() {
		sensorsTopicListener = new ROSLIB.Topic({
			ros,
			name: 'sensor_topic',
			messageType: 'std_msgs/String'
		});

		sensorsTopicListener.subscribe(
			(message) => {
				// console.log('Received message:', message.data); // Log the message data

				const pairs = message.data.slice(1, -1).split(',');
				const sensorData = {};

				pairs.forEach((pair) => {
					const [key, value] = pair.split(':');
					const cleanKey = key.trim().replace(/['"]+/g, '');
					let cleanValue = value.trim().replace(/['"]+/g, '');
					sensorData[cleanKey] = cleanValue;
				});

				const cpu = sensorData.cpu;
				const gpu = sensorData.gpu;
				const ram = sensorData.ram;
				const disk = sensorData.disk;
				const networkSent = sensorData.network_sent;
				const networkReceived = sensorData.network_received;
				const wifi = sensorData.wifi;

				sensorsTopicData = sensorData;

				// Log or use the extracted sensor readings as needed
				// console.log('CPU:', cpu);
				// console.log('GPU:', gpu);
				// console.log('RAM:', ram);
				// console.log('Disk:', disk);
				// console.log('Network Sent:', networkSent);
				// console.log('Network Received:', networkReceived);
				// console.log('WIFI:', wifi);
			},
			(error) => {
				console.error(`Error in / subscription: ${error}`);
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
		commandVelocity = new ROSLIB.Topic({
			ros: ros,
			name: '/cmd_vel',
			messageType: 'geometry_msgs/Twist'
		});
	}

	function publishJoystickCommands(data) {
		let xAxis = parseFloat(0);
		let zAxis = parseFloat(0);
		let force = parseFloat(data.force);
		let degree = parseFloat(data.angle.degree);

		if (force > 1.0) force = 1.0;

		if (degree > 45 && degree < 135) xAxis = force;
		if (degree > 135 && degree < 225) zAxis = force;
		if (degree > 225 && degree < 315) xAxis = -force;
		if ((degree > 315 && degree < 360) || (degree > 0 && degree < 45)) zAxis = -force;

		const twist = new ROSLIB.Message({
			linear: {
				x: xAxis,
				y: 0,
				z: 0
			},
			angular: {
				x: 0,
				y: 0,
				z: zAxis
			}
		});

		if (commandVelocity) commandVelocity.publish(twist);
		else console.error('Command velocity topic not initialized');
	}

	function subscribeToVideoStreamingTopic() {
		videoTopicListener = new ROSLIB.Topic({
			ros: ros,
			name: '/video_streaming_topic',
			messageType: 'sensor_msgs/CompressedImage'
		});

		videoTopicListener.subscribe(
			(message) => {
				videoTopicData = message.data;
				// console.log(message.data.length / 1000);
				// console.log(`Size of received image: ${videoTopicData.length / 1024} KB`);
			},
			(error) => {
				console.error(`Error in /terminal_topic subscription: ${error}`);
				console.log(error);
			}
		);
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
		<div>
			<ul
				class="robot-container-footer-sensors"
				style="visibility: {showSensors ? 'visible' : 'hidden'}"
			>
				<!-- <li>
					<div>
						<svg
							xmlns="http://www.w3.org/2000/svg"
							width="16"
							height="16"
							fill="currentColor"
							viewBox="0 0 16 16"
							style="vertical-align: middle;"
						>
							<path
								d="M5 0a.5.5 0 0 1 .5.5V2h1V.5a.5.5 0 0 1 1 0V2h1V.5a.5.5 0 0 1 1 0V2h1V.5a.5.5 0 0 1 1 0V2A2.5 2.5 0 0 1 14 4.5h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14a2.5 2.5 0 0 1-2.5 2.5v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14A2.5 2.5 0 0 1 2 11.5H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2A2.5 2.5 0 0 1 4.5 2V.5A.5.5 0 0 1 5 0m-.5 3A1.5 1.5 0 0 0 3 4.5v7A1.5 1.5 0 0 0 4.5 13h7a1.5 1.5 0 0 0 1.5-1.5v-7A1.5 1.5 0 0 0 11.5 3zM5 6.5A1.5 1.5 0 0 1 6.5 5h3A1.5 1.5 0 0 1 11 6.5v3A1.5 1.5 0 0 1 9.5 11h-3A1.5 1.5 0 0 1 5 9.5zM6.5 6a.5.5 0 0 0-.5.5v3a.5.5 0 0 0 .5.5h3a.5.5 0 0 0 .5-.5v-3a.5.5 0 0 0-.5-.5z"
							/>
						</svg>
					</div>
					<div>CPU:</div>
					<div>{sensorsTopicData.cpu}</div>
				</li> -->
				<li>
					<span
						><svg
							xmlns="http://www.w3.org/2000/svg"
							width="16"
							height="16"
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
						</svg>
						GPU: {sensorsTopicData.gpu}</span
					>
				</li>
				<li>
					<span
						><svg
							xmlns="http://www.w3.org/2000/svg"
							width="16"
							height="16"
							fill="currentColor"
							class="bi bi-sliders"
							viewBox="0 0 16 16"
							style="vertical-align: middle;"
						>
							<path
								fill-rule="evenodd"
								d="M11.5 2a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3M9.05 3a2.5 2.5 0 0 1 4.9 0H16v1h-2.05a2.5 2.5 0 0 1-4.9 0H0V3zM4.5 7a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3M2.05 8a2.5 2.5 0 0 1 4.9 0H16v1H6.95a2.5 2.5 0 0 1-4.9 0H0V8zm9.45 4a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3m-2.45 1a2.5 2.5 0 0 1 4.9 0H16v1h-2.05a2.5 2.5 0 0 1-4.9 0H0v-1z"
							/>
						</svg>
						RAM: {sensorsTopicData.ram}</span
					>
				</li>
				<li>
					<span
						><svg
							xmlns="http://www.w3.org/2000/svg"
							width="16"
							height="16"
							fill="currentColor"
							class="bi bi-database-gear"
							viewBox="0 0 16 16"
							style="vertical-align: middle;"
						>
							<path
								d="M12.096 6.223A5 5 0 0 0 13 5.698V7c0 .289-.213.654-.753 1.007a4.5 4.5 0 0 1 1.753.25V4c0-1.007-.875-1.755-1.904-2.223C11.022 1.289 9.573 1 8 1s-3.022.289-4.096.777C2.875 2.245 2 2.993 2 4v9c0 1.007.875 1.755 1.904 2.223C4.978 15.71 6.427 16 8 16c.536 0 1.058-.034 1.555-.097a4.5 4.5 0 0 1-.813-.927Q8.378 15 8 15c-1.464 0-2.766-.27-3.682-.687C3.356 13.875 3 13.373 3 13v-1.302c.271.202.58.378.904.525C4.978 12.71 6.427 13 8 13h.027a4.6 4.6 0 0 1 0-1H8c-1.464 0-2.766-.27-3.682-.687C3.356 10.875 3 10.373 3 10V8.698c.271.202.58.378.904.525C4.978 9.71 6.427 10 8 10q.393 0 .774-.024a4.5 4.5 0 0 1 1.102-1.132C9.298 8.944 8.666 9 8 9c-1.464 0-2.766-.27-3.682-.687C3.356 7.875 3 7.373 3 7V5.698c.271.202.58.378.904.525C4.978 6.711 6.427 7 8 7s3.022-.289 4.096-.777M3 4c0-.374.356-.875 1.318-1.313C5.234 2.271 6.536 2 8 2s2.766.27 3.682.687C12.644 3.125 13 3.627 13 4c0 .374-.356.875-1.318 1.313C10.766 5.729 9.464 6 8 6s-2.766-.27-3.682-.687C3.356 4.875 3 4.373 3 4"
							/>
							<path
								d="M11.886 9.46c.18-.613 1.048-.613 1.229 0l.043.148a.64.64 0 0 0 .921.382l.136-.074c.561-.306 1.175.308.87.869l-.075.136a.64.64 0 0 0 .382.92l.149.045c.612.18.612 1.048 0 1.229l-.15.043a.64.64 0 0 0-.38.921l.074.136c.305.561-.309 1.175-.87.87l-.136-.075a.64.64 0 0 0-.92.382l-.045.149c-.18.612-1.048.612-1.229 0l-.043-.15a.64.64 0 0 0-.921-.38l-.136.074c-.561.305-1.175-.309-.87-.87l.075-.136a.64.64 0 0 0-.382-.92l-.148-.045c-.613-.18-.613-1.048 0-1.229l.148-.043a.64.64 0 0 0 .382-.921l-.074-.136c-.306-.561.308-1.175.869-.87l.136.075a.64.64 0 0 0 .92-.382zM14 12.5a1.5 1.5 0 1 1-3 0 1.5 1.5 0 0 1 3 0"
							/>
						</svg>
						DISK: {sensorsTopicData.disk}</span
					>
				</li>
				<li>
					<span>
						<svg
							xmlns="http://www.w3.org/2000/svg"
							width="16"
							height="16"
							fill="currentColor"
							viewBox="0 0 16 16"
							style="vertical-align: middle;"
						>
							<path d="M2 6h10v4H2z" />
							<path
								d="M2 4a2 2 0 0 0-2 2v4a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V6a2 2 0 0 0-2-2zm10 1a1 1 0 0 1 1 1v4a1 1 0 0 1-1 1H2a1 1 0 0 1-1-1V6a1 1 0 0 1 1-1zm4 3a1.5 1.5 0 0 1-1.5 1.5v-3A1.5 1.5 0 0 1 16 8"
							/>
						</svg> BAT: N/A
					</span>
				</li>

				<li>
					<span
						><svg
							xmlns="http://www.w3.org/2000/svg"
							width="16"
							height="16"
							fill="currentColor"
							viewBox="0 0 16 16"
							style="vertical-align: middle;"
						>
							<path
								d="M8 0a8 8 0 1 0 0 16A8 8 0 0 0 8 0M2.04 4.326c.325 1.329 2.532 2.54 3.717 3.19.48.263.793.434.743.484q-.121.12-.242.234c-.416.396-.787.749-.758 1.266.035.634.618.824 1.214 1.017.577.188 1.168.38 1.286.983.082.417-.075.988-.22 1.52-.215.782-.406 1.48.22 1.48 1.5-.5 3.798-3.186 4-5 .138-1.243-2-2-3.5-2.5-.478-.16-.755.081-.99.284-.172.15-.322.279-.51.216-.445-.148-2.5-2-1.5-2.5.78-.39.952-.171 1.227.182.078.099.163.208.273.318.609.304.662-.132.723-.633.039-.322.081-.671.277-.867.434-.434 1.265-.791 2.028-1.12.712-.306 1.365-.587 1.579-.88A7 7 0 1 1 2.04 4.327Z"
							/>
						</svg> GPS: N/A</span
					>
				</li>
				<li>
					<span
						><svg
							xmlns="http://www.w3.org/2000/svg"
							width="16"
							height="16"
							fill="currentColor"
							class="bi bi-arrow-up-circle"
							viewBox="0 0 16 16"
							style="vertical-align: middle;"
						>
							<path
								fill-rule="evenodd"
								d="M1 8a7 7 0 1 0 14 0A7 7 0 0 0 1 8m15 0A8 8 0 1 1 0 8a8 8 0 0 1 16 0m-7.5 3.5a.5.5 0 0 1-1 0V5.707L5.354 7.854a.5.5 0 1 1-.708-.708l3-3a.5.5 0 0 1 .708 0l3 3a.5.5 0 0 1-.708.708L8.5 5.707z"
							/>
						</svg>
						SENT: {sensorsTopicData.network_sent}
					</span>
				</li>
				<li>
					<span
						><svg
							xmlns="http://www.w3.org/2000/svg"
							width="16"
							height="16"
							fill="currentColor"
							class="bi bi-arrow-down-circle"
							viewBox="0 0 16 16"
						>
							<path
								fill-rule="evenodd"
								d="M1 8a7 7 0 1 0 14 0A7 7 0 0 0 1 8m15 0A8 8 0 1 1 0 8a8 8 0 0 1 16 0M8.5 4.5a.5.5 0 0 0-1 0v5.793L5.354 8.146a.5.5 0 1 0-.708.708l3 3a.5.5 0 0 0 .708 0l3-3a.5.5 0 0 0-.708-.708L8.5 10.293z"
							/>
						</svg>
						REC: {sensorsTopicData.network_received}</span
					>
				</li>
			</ul>
		</div>
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
		gap: 10px 20px;
		border-bottom: 1px solid var(--text-color);
		z-index: 1;
		flex-wrap: wrap;
	}

	.robot-container-header-navbar button {
		background: none;
		color: var(--text-color);
		border: 1px solid var(--text-color);
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
		object-fit: contain;
	}

	.robot-container-footer-sensors {
		position: absolute;
		top: 0;
		padding: 10px;
		background: var(--background-color);
		/* background: green; */
		transition: var(--theme-transition-time);
		list-style: none;
		/* line-height: 1rem; */
		border-right: 1px solid var(--text-color);
		border-bottom: 1px solid var(--text-color);
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
