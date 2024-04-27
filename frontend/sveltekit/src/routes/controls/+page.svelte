<script>
	import { onMount, onDestroy } from 'svelte';
	// import { setContext } from 'svelte';

	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';

	let joystick;

	let ros;
	let terminalTopicListener;
	let controlsTopicListener;


	const loadNippleJs = async () => {
		const { default: Nipple } = await import('nipplejs');

		joystick = Nipple.create({
			zone: document.getElementById('joystick-container'),
			mode: 'static',
			position: { left: '50%', top: '50%' },
			color: 'red',
			size: 100
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

	onMount(() => {
		loadNippleJs();

		const unsubscribe = rosStore.subscribe((value) => {
			if (value !== null) {
				ros = value;
				console.log('Inside controls');
				subscribeToTerminalTopic();
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

<div class="aligning-container">
	<div id="joystick-container"></div>
</div>

<div id="debug">
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
</div>

<style>
	#debug {
		position: absolute;
		top: 0;
		left: 0;
		z-index: -1; /* Place the debug div behind the joystick */
		width: 100%;
		height: 100%;
		display: flex;
		justify-content: center;
		align-items: center;
		font-size: 2.5rem;
	}

	.aligning-container {
		height: calc(100dvh - 80px);
		display: flex;
		justify-content: center;
		align-items: center;
	}

	#joystick-container {
		position: relative;
		width: 100px;
		height: 100px;
		background-color: rgba(255, 0, 0, 0.5);
		border-radius: 50%;
	}
</style>
