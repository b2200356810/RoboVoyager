<script>
	import { onMount, onDestroy } from 'svelte';
	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';

	let ros;

	let terminalTopicListener;
	let terminalTopicData;
	let videoTopicListener;
	let sensorsTopicListener;
	let sensorsTopicData = {
		cpu: 'N/A',
		gpu: 'N/A',
		ram: 'N/A',
		disk: 'N/A',
		battery: 'N/A',
		gps: 'N/A',
		network_sent: 'N/A',
		network_received: 'N/A'
	};

	let joystick;
	let commandVelocity;
	let joystickX = 0;
	let joystickZ = 0;
	let speedGain = 0.0;

	let showAI = false;
	let showMap = true;
	let showSensors = true;
	let showControls = true;
	let showCapture = false;
	let showVideo = false;

	let showCaptureButtons = false;
	let captureStarted = false;
	let capturePaused = false;
	let captureStopped = false;

	let makeService = false;

	onMount(async () => {
		loadNippleJs();

		const unsubscribe = rosStore.subscribe((value) => {
			if (value !== null) {
				ros = value;
				subscribeToTerminalTopic();
				subscribeToSensorsTopic();
				publishToControlsTopic();
			}
		});
	});

	onDestroy(() => {
		if (joystick) joystick.destroy();
		if (terminalTopicListener) terminalTopicListener.unsubscribe();
		if (sensorsTopicListener) sensorsTopicListener.unsubscribe();
		if (videoTopicListener) videoTopicListener.unsubscribe();
	});

	function getTimestamp() {
		const now = new Date();
		const hour = String(now.getHours()).padStart(2, '0');
		const minute = String(now.getMinutes()).padStart(2, '0');
		const timestamp = `[${hour}:${minute}]`;
		return timestamp;
	}

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
			name: '/sensors_topic',
			messageType: 'std_msgs/String'
		});

		sensorsTopicListener.subscribe(
			(message) => {
				const sensorData = JSON.parse(message.data);

				Object.keys(sensorData).forEach((key) => {
					if (key in sensorsTopicData) {
						sensorsTopicData[key] = sensorData[key];
					}
				});

				sensorsTopicData.cpu = sensorData.cpu_info.total_cpu_usage + ' %';
				sensorsTopicData.ram = sensorData.memory_info.memory_percentage + ' %';
				sensorsTopicData.disk = sensorData.disk_info.usage_percentage + ' %';
				sensorsTopicData.network_sent =
					sensorData.internet_speed.megabytes_sent_per_second + ' MB/s';
				sensorsTopicData.network_received =
					sensorData.internet_speed.megabytes_received_per_second + ' MB/s';
			},
			(error) => {
				console.error(`Error in /sensors_topic subscription: ${error}`);
				console.log(error);
			}
		);
	}

	function subscribeToVideoStreamingTopic() {
		let canvas = document.getElementById('video-stream');
		let ctx = canvas.getContext('2d');

		// let imageElement = document.getElementById('video-stream');

		videoTopicListener = new ROSLIB.Topic({
			ros: ros,
			name: '/video_streaming_topic',
			messageType: 'sensor_msgs/CompressedImage'
		});

		videoTopicListener.subscribe(
			(message) => {
				const img = new Image();
				img.onload = function () {
					// ctx.clearRect(0, 0, canvas.width, canvas.height);
					ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
				};
				img.src = 'data:image/jpeg;base64,' + message.data;
				// imageElement.src = 'data:image/jpeg;base64,' + message.data;
			},
			(error) => {
				console.error(`Error in /terminal_topic subscription: ${error}`);
				console.log(error);
			}
		);
	}

	function changeVideoStreamSettings(width, height, fps) {
		const service = new ROSLIB.Service({
			ros: ros,
			name: '/video_streaming_change_settings',
			serviceType: 'video_streaming/SetVideoStreamSettings'
		});

		const request = new ROSLIB.ServiceRequest({
			width: width,
			height: height,
			fps: fps
		});

		service.callService(request, (result) => {
			console.log(`Service call result: ${result.success}, message: ${result.message}`);
		});
	}

	function publishToControlsTopic() {
		commandVelocity = new ROSLIB.Topic({
			ros: ros,
			name: '/cmd_vel',
			messageType: 'geometry_msgs/Twist'
		});
	}

	async function loadNippleJs() {
		const { default: Nipple } = await import('nipplejs');

		joystick = Nipple.create({
			zone: document.getElementById('joystick-container'),
			mode: 'static',
			position: { left: '50%', top: '50%' },
			color: 'red',
			size: 100,
			restOpacity: 0.8
		});

		joystick.on('move', (evt, data) => {
			publishJoystickCommands(data);
		});

		joystick.on('end', () => {
			publishJoystickCommandsSTOP();
		});
	}

	function publishJoystickCommands(data) {
		// Up Down
		let xAxis = 0.0;
		//  Left Right (turning)
		let zAxis = 0.0;
		// Speed
		let force = parseFloat(parseFloat(data.force).toFixed(2));
		// Actual direction (right of circle is 0)
		let degree = parseFloat(data.angle.degree);

		// Limit the speed
		force = force / 2;
		force = force + speedGain;
		if (force > 1.0) force = 1.0;

		let turnRate = 2;

		if (degree > 60 && degree < 120) xAxis = force; // UP
		if (degree > 240 && degree < 300) xAxis = -force; // DOWN
		if (degree > 150 && degree < 210) zAxis = force; // LEFT
		if ((degree > 0 && degree < 30) || (degree > 330 && degree < 360)) zAxis = -force; // RIGHT

		// LEFT UP
		if (degree > 120 && degree < 150) {
			xAxis = force;
			zAxis = force / turnRate;
		}
		// RIGHT UP
		if (degree > 30 && degree < 60) {
			xAxis = force;
			zAxis = -force / turnRate;
		}
		// LEFT DOWN
		if (degree > 210 && degree < 240) {
			xAxis = -force;
			zAxis = force / turnRate;
		}
		// RIGHT DOWN
		if (degree > 300 && degree < 330) {
			xAxis = -force;
			zAxis = -force / turnRate;
		}

		xAxis = parseFloat(xAxis);
		zAxis = parseFloat(zAxis);

		joystickX = xAxis.toFixed(2);
		joystickZ = zAxis.toFixed(2);

		const twist = new ROSLIB.Message({
			linear: {
				x: parseFloat(xAxis),
				y: 0.0,
				z: 0.0
			},
			angular: {
				x: 0.0,
				y: 0.0,
				z: parseFloat(zAxis)
			}
		});

		if (commandVelocity) commandVelocity.publish(twist);
		else console.error('Command velocity topic not initialized');
	}

	function publishJoystickCommandsSTOP() {
		const twist = new ROSLIB.Message({
			linear: {
				x: 0.0,
				y: 0.0,
				z: 0.0
			},
			angular: {
				x: 0.0,
				y: 0.0,
				z: 0.0
			}
		});

		joystickX = 0;
		joystickZ = 0;

		if (commandVelocity) commandVelocity.publish(twist);
		else console.error('Command velocity topic not initialized');
	}

	function publishDataSaverStatus(command) {
		var controlTopic = new ROSLIB.Topic({
			ros: ros,
			name: '/data_saver_control',
			messageType: 'std_msgs/String'
		});

		var message = new ROSLIB.Message({
			data: command
		});

		controlTopic.publish(message);
	}
</script>

<div class="robot-container">
	<div class="robot-container-header">
		<nav class="robot-container-header-navbar">
			<button
				style="color: {showAI ? 'green' : 'red'};"
				on:click={() => {
					showAI = !showAI;
				}}>AI</button
			>

			<button
				style="color: {showSensors ? 'green' : 'red'}"
				on:click={() => {
					showSensors = !showSensors;
				}}
				>Sensors
			</button>

			<button
				style="color: {showControls ? 'green' : 'red'}"
				on:click={() => {
					showControls = !showControls;
				}}>Controls</button
			>

			<button
				style="color: {showCapture ? 'green' : 'red'}"
				on:click={() => {
					showCapture = !showCapture;
					showCaptureButtons = !showCaptureButtons;
				}}>Capture</button
			>

			<button
				style="color: {showVideo ? 'green' : 'red'}"
				on:click={() => {
					showVideo = !showVideo;
					if (showVideo) subscribeToVideoStreamingTopic();
					else videoTopicListener.unsubscribe();
				}}>Video</button
			>
		</nav>
	</div>

	<div class="robot-container-body">
		<div class="data-saver" style="display: {showCaptureButtons ? 'initial' : 'none'}">
			<div>
				<button style="visibility: hidden;"></button>
				<button style="visibility: hidden;"></button>

				<button
					style="color: {captureStarted ? 'green' : 'red'}"
					on:click={() => {
						if (capturePaused === true || captureStopped === true) {
							capturePaused = false;
							captureStopped = false;
						}
						captureStarted = true;
						publishDataSaverStatus('start');
					}}>Start</button
				>
				<button
					style="color: {capturePaused ? 'green' : 'red'}"
					on:click={() => {
						if (captureStarted === true || captureStopped === true) {
							captureStarted = false;
							captureStopped = false;
						}
						capturePaused = true;
						publishDataSaverStatus('pause');
					}}>Pause</button
				>
				<button
					style="color: {captureStopped ? 'green' : 'red'}"
					on:click={() => {
						if (captureStarted === true || capturePaused === true) {
							captureStarted = false;
							capturePaused = false;
						}
						captureStopped = true;
						publishDataSaverStatus('stop');
					}}>Stop</button
				>
			</div>
		</div>

		<canvas id="video-stream" style="display: {showVideo ? 'initial' : 'none'}"></canvas>
		<!-- <img id="video-stream" alt="" style="display: {showVideo ? 'initial' : 'none'}" /> -->

		<div class="sensors-grid" style="visibility: {showSensors ? 'visible' : 'hidden'}">
			<div class="sensor-icon">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					fill="currentColor"
					viewBox="0 0 16 16"
					style="vertical-align: middle;"
				>
					<path
						d="M5 0a.5.5 0 0 1 .5.5V2h1V.5a.5.5 0 0 1 1 0V2h1V.5a.5.5 0 0 1 1 0V2h1V.5a.5.5 0 0 1 1 0V2A2.5 2.5 0 0 1 14 4.5h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14v1h1.5a.5.5 0 0 1 0 1H14a2.5 2.5 0 0 1-2.5 2.5v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14h-1v1.5a.5.5 0 0 1-1 0V14A2.5 2.5 0 0 1 2 11.5H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2v-1H.5a.5.5 0 0 1 0-1H2A2.5 2.5 0 0 1 4.5 2V.5A.5.5 0 0 1 5 0m-.5 3A1.5 1.5 0 0 0 3 4.5v7A1.5 1.5 0 0 0 4.5 13h7a1.5 1.5 0 0 0 1.5-1.5v-7A1.5 1.5 0 0 0 11.5 3zM5 6.5A1.5 1.5 0 0 1 6.5 5h3A1.5 1.5 0 0 1 11 6.5v3A1.5 1.5 0 0 1 9.5 11h-3A1.5 1.5 0 0 1 5 9.5zM6.5 6a.5.5 0 0 0-.5.5v3a.5.5 0 0 0 .5.5h3a.5.5 0 0 0 .5-.5v-3a.5.5 0 0 0-.5-.5z"
					/>
				</svg>
			</div>
			<div class="sensor-label">CPU:</div>
			<div class="sensor-value">{sensorsTopicData.cpu}</div>

			<div class="sensor-icon">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					viewBox="0 0 16 16"
					fill="currentColor"
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
			</div>
			<div class="sensor-label">GPU:</div>
			<div class="sensor-value">{sensorsTopicData.gpu}</div>

			<div class="sensor-icon">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					viewBox="0 0 16 16"
					fill="currentColor"
					style="vertical-align: middle;"
				>
					<path
						fill-rule="evenodd"
						d="M11.5 2a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3M9.05 3a2.5 2.5 0 0 1 4.9 0H16v1h-2.05a2.5 2.5 0 0 1-4.9 0H0V3zM4.5 7a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3M2.05 8a2.5 2.5 0 0 1 4.9 0H16v1H6.95a2.5 2.5 0 0 1-4.9 0H0V8zm9.45 4a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3m-2.45 1a2.5 2.5 0 0 1 4.9 0H16v1h-2.05a2.5 2.5 0 0 1-4.9 0H0v-1z"
					/>
				</svg>
			</div>
			<div class="sensor-label">RAM:</div>
			<div class="sensor-value">{sensorsTopicData.ram}</div>

			<div class="sensor-icon">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					viewBox="0 0 16 16"
					fill="currentColor"
					style="vertical-align: middle;"
				>
					<path
						d="M12.096 6.223A5 5 0 0 0 13 5.698V7c0 .289-.213.654-.753 1.007a4.5 4.5 0 0 1 1.753.25V4c0-1.007-.875-1.755-1.904-2.223C11.022 1.289 9.573 1 8 1s-3.022.289-4.096.777C2.875 2.245 2 2.993 2 4v9c0 1.007.875 1.755 1.904 2.223C4.978 15.71 6.427 16 8 16c.536 0 1.058-.034 1.555-.097a4.5 4.5 0 0 1-.813-.927Q8.378 15 8 15c-1.464 0-2.766-.27-3.682-.687C3.356 13.875 3 13.373 3 13v-1.302c.271.202.58.378.904.525C4.978 12.71 6.427 13 8 13h.027a4.6 4.6 0 0 1 0-1H8c-1.464 0-2.766-.27-3.682-.687C3.356 10.875 3 10.373 3 10V8.698c.271.202.58.378.904.525C4.978 9.71 6.427 10 8 10q.393 0 .774-.024a4.5 4.5 0 0 1 1.102-1.132C9.298 8.944 8.666 9 8 9c-1.464 0-2.766-.27-3.682-.687C3.356 7.875 3 7.373 3 7V5.698c.271.202.58.378.904.525C4.978 6.711 6.427 7 8 7s3.022-.289 4.096-.777M3 4c0-.374.356-.875 1.318-1.313C5.234 2.271 6.536 2 8 2s2.766.27 3.682.687C12.644 3.125 13 3.627 13 4c0 .374-.356.875-1.318 1.313C10.766 5.729 9.464 6 8 6s-2.766-.27-3.682-.687C3.356 4.875 3 4.373 3 4"
					/>
					<path
						d="M11.886 9.46c.18-.613 1.048-.613 1.229 0l.043.148a.64.64 0 0 0 .921.382l.136-.074c.561-.306 1.175.308.87.869l-.075.136a.64.64 0 0 0 .382.92l.149.045c.612.18.612 1.048 0 1.229l-.15.043a.64.64 0 0 0-.38.921l.074.136c.305.561-.309 1.175-.87.87l-.136-.075a.64.64 0 0 0-.92.382l-.045.149c-.18.612-1.048.612-1.229 0l-.043-.15a.64.64 0 0 0-.921-.38l-.136.074c-.561.305-1.175-.309-.87-.87l.075-.136a.64.64 0 0 0-.382-.92l-.148-.045c-.613-.18-.613-1.048 0-1.229l.148-.043a.64.64 0 0 0 .382-.921l-.074-.136c-.306-.561.308-1.175.869-.87l.136.075a.64.64 0 0 0 .92-.382zM14 12.5a1.5 1.5 0 1 1-3 0 1.5 1.5 0 0 1 3 0"
					/>
				</svg>
			</div>
			<div class="sensor-label">DISK:</div>
			<div class="sensor-value">{sensorsTopicData.disk}</div>

			<div class="sensor-icon">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					viewBox="0 0 16 16"
					fill="currentColor"
					style="vertical-align: middle;"
				>
					<path d="M2 6h10v4H2z" />
					<path
						d="M2 4a2 2 0 0 0-2 2v4a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V6a2 2 0 0 0-2-2zm10 1a1 1 0 0 1 1 1v4a1 1 0 0 1-1 1H2a1 1 0 0 1-1-1V6a1 1 0 0 1 1-1zm4 3a1.5 1.5 0 0 1-1.5 1.5v-3A1.5 1.5 0 0 1 16 8"
					/>
				</svg>
			</div>
			<div class="sensor-label">BAT:</div>
			<div class="sensor-value">{sensorsTopicData.battery}</div>

			<div class="sensor-icon">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					viewBox="0 0 16 16"
					fill="currentColor"
					style="vertical-align: middle;"
				>
					<path
						d="M8 0a8 8 0 1 0 0 16A8 8 0 0 0 8 0M2.04 4.326c.325 1.329 2.532 2.54 3.717 3.19.48.263.793.434.743.484q-.121.12-.242.234c-.416.396-.787.749-.758 1.266.035.634.618.824 1.214 1.017.577.188 1.168.38 1.286.983.082.417-.075.988-.22 1.52-.215.782-.406 1.48.22 1.48 1.5-.5 3.798-3.186 4-5 .138-1.243-2-2-3.5-2.5-.478-.16-.755.081-.99.284-.172.15-.322.279-.51.216-.445-.148-2.5-2-1.5-2.5.78-.39.952-.171 1.227.182.078.099.163.208.273.318.609.304.662-.132.723-.633.039-.322.081-.671.277-.867.434-.434 1.265-.791 2.028-1.12.712-.306 1.365-.587 1.579-.88A7 7 0 1 1 2.04 4.327Z"
					/>
				</svg>
			</div>
			<div class="sensor-label">GPS:</div>
			<div class="sensor-value">{sensorsTopicData.gps}</div>

			<div class="sensor-icon">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					viewBox="0 0 16 16"
					fill="currentColor"
					style="vertical-align: middle;"
				>
					<path
						d="M1 8a7 7 0 1 0 14 0A7 7 0 0 0 1 8m15 0A8 8 0 1 1 0 8a8 8 0 0 1 16 0m-7.5 3.5a.5.5 0 0 1-1 0V5.707L5.354 7.854a.5.5 0 1 1-.708-.708l3-3a.5.5 0 0 1 .708 0l3 3a.5.5 0 0 1-.708.708L8.5 5.707z"
					/>
				</svg>
			</div>
			<div class="sensor-label">SENT:</div>
			<div class="sensor-value">{sensorsTopicData.network_sent}</div>

			<div class="sensor-icon">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					viewBox="0 0 16 16"
					fill="currentColor"
					style="vertical-align: middle;"
				>
					<path
						d="M1 8a7 7 0 1 0 14 0A7 7 0 0 0 1 8m15 0A8 8 0 1 1 0 8a8 8 0 0 1 16 0M8.5 4.5a.5.5 0 0 0-1 0v5.793L5.354 8.146a.5.5 0 1 0-.708.708l3 3a.5.5 0 0 0 .708 0l3-3a.5.5 0 0 0-.708-.708L8.5 10.293z"
					/>
				</svg>
			</div>
			<div class="sensor-label">REC:</div>
			<div class="sensor-value">{sensorsTopicData.network_received}</div>
		</div>

		<div class="robot-container-body-wrapper">
			<div class="joystick-speed" style="visibility: {showControls ? 'visible' : 'hidden'}">
				<span class="joystick-label">X:</span>
				<span class="joystick-value">{joystickX}</span>
				<span class="joystick-label">Z:</span>
				<span class="joystick-value">{joystickZ}</span>
			</div>

			<div id="joystick-container" style="visibility: {showControls ? 'visible' : 'hidden'}">
				<div class="diagonal-markings"></div>
				<div class="joystick-axis x-axis-pos">+x</div>
				<div class="joystick-axis x-axis-neg">-x</div>
				<div class="joystick-axis z-axis-pos">+z</div>
				<div class="joystick-axis z-axis-neg">-z</div>
			</div>

			<div class="joystick-limit" style="visibility: {showControls ? 'visible' : 'hidden'}">
				<div>Speed gain {speedGain}</div>
				<button
					on:click={() => {
						speedGain = parseFloat((speedGain + 0.2).toFixed(1));
					}}>+0.2</button
				>
				<button
					on:click={() => {
						if (speedGain === 0) return;
						speedGain = parseFloat((speedGain - 0.2).toFixed(1));
					}}>-0.2</button
				>
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
		user-select: none;
	}

	.robot-container-header-navbar {
		display: grid;
		grid-template-columns: repeat(5, 1fr);
		padding: 10px 5px 5px 5px;
		justify-content: center;
		gap: 10px;
		z-index: 1;
	}

	.robot-container-header-navbar button {
		background: var(--navbar-background);
		border: none;
		padding: 5px;
		font-size: 1.5rem;
		cursor: pointer;
		border-radius: 10px;
		box-shadow: 0px 2px 5px 0px rgba(0, 0, 0, 0.3);
	}

	.robot-container-header-navbar button:hover {
		color: yellow !important;
		transition: var(--theme-transition-time);
	}

	.robot-container-body {
		height: 100%;
		display: flex;
		flex-direction: column;
		justify-content: space-between;
		position: relative;
	}

	.data-saver {
		position: absolute;
		width: 100%;
		padding: 5px;
		/* background-color: rgba(0, 0, 0, 0.4); */
	}

	.data-saver div {
		display: grid;
		grid-template-columns: repeat(5, 1fr);
		justify-content: center;
		gap: 10px;
	}

	.data-saver button {
		background: var(--navbar-background);
		border: none;
		padding: 5px;
		font-size: 1.5rem;
		cursor: pointer;
		border-radius: 10px;
		box-shadow: 0px 2px 5px 0px rgba(0, 0, 0, 0.3);
		color: var(--text-color);
	}

	#video-stream {
		position: absolute;
		height: 100%;
		width: 100%;
		/* object-fit: cover; */
		user-select: none;
		-webkit-user-drag: none;
		-moz-user-select: none;
		-ms-user-select: none;
		pointer-events: none;
	}

	.sensors-grid {
		position: relative;
		width: fit-content;
		top: 0;
		z-index: 10;
		list-style: none;
		padding: 10px;
		background: var(--background-color);
		transition: var(--theme-transition-time);
		/* border: 1px solid var(--text-color); */
		/* border-top: none; */
		display: grid;
		grid-template-columns: auto auto auto;
		gap: 1px 8px;
		font-size: 1.6rem;
		margin: 10px;
		border-radius: 15px;
	}

	.sensor-icon {
		width: 15px;
		height: 15px;
	}

	.robot-container-body-wrapper {
		display: flex;
		justify-content: space-between;
		padding: 30px 10px;
		gap: 20px;
	}

	.robot-container-body-wrapper div {
		width: 100px;
		height: 100px;
	}

	.joystick-speed {
		display: grid;
		grid-template-columns: 1fr 1fr;
		align-content: center;
		justify-items: center;
		gap: 5px;
		z-index: 10;
	}

	.joystick-label {
		justify-self: end;
	}

	.joystick-value {
		justify-self: start;
	}

	#joystick-container {
		position: relative;
		border-radius: 50%;
	}

	.diagonal-markings {
		position: absolute;
		border-radius: 50%;
		z-index: 49;
		background: conic-gradient(
			from 0deg,
			transparent 0deg,
			transparent 30deg,
			var(--text-color) 30deg,
			var(--text-color) 60deg,
			transparent 60deg,
			transparent 120deg,
			var(--text-color) 120deg,
			var(--text-color) 150deg,
			transparent 150deg,
			transparent 210deg,
			var(--text-color) 210deg,
			var(--text-color) 240deg,
			transparent 240deg,
			transparent 300deg,
			var(--text-color) 300deg,
			var(--text-color) 330deg,
			transparent 330deg,
			transparent 360deg
		);

		mask: radial-gradient(circle, transparent 40px, var(--text-color) 70%);
	}

	#joystick-container .joystick-axis {
		position: absolute;
		height: fit-content;
		width: fit-content;
	}

	.x-axis-pos {
		top: -2px;
		left: 50%;
		transform: translateX(-50%);
	}

	.x-axis-neg {
		bottom: -2px;
		left: 50%;
		transform: translateX(-50%);
	}

	.z-axis-pos {
		left: 3px;
		top: 50%;
		transform: translateY(-50%);
	}

	.z-axis-neg {
		right: 3px;
		top: 50%;
		transform: translateY(-50%);
	}

	.joystick-limit {
		display: grid;
		grid-template-columns: 1fr 1fr;
		grid-auto-flow: row;
		grid-template-areas:
			'Limit Limit'
			'Increase Decrease';
		gap: 5px;
		align-content: center;
		justify-items: center;
		border-radius: 10px;
		text-align: center;
	}

	.joystick-limit div {
		grid-area: Limit;
		height: fit-content;
	}

	.joystick-limit button {
		background: var(--navbar-background);
		border: none;
		padding: 5px;
		font-size: 1.5rem;
		cursor: pointer;
		border-radius: 10px;
		box-shadow: 0px 2px 5px 0px rgba(0, 0, 0, 0.3);
		color: var(--text-color);
	}

	.joystick-limit button:nth-child(1) {
		grid-area: Increase;
	}

	.joystick-limit button:nth-child(2) {
		grid-area: Decrease;
	}
	@media (min-width: 1000px) {
		.robot-container-header-navbar,
		.data-saver div {
			grid-template-columns: repeat(5, 150px);
			padding: 10px 20px;
			gap: 20px;
		}

		.data-saver div {
			padding: 0 10px 0 10px;
		}

		.robot-container-body-wrapper {
			justify-content: space-around;
		}
	}
</style>
