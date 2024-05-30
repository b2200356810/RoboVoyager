<script>
	import { onMount, onDestroy } from 'svelte';
	import Navbar from '../lib/components/Navbar.svelte';
	import ROSLIB from 'roslib';
	import { rosStore } from '$lib/stores/rosStore.js';

	let ros;
	onMount(() => {
		connectToRosbridge();
	});

	onDestroy(() => {
		disconnectFromRosbridge();
	});

	function connectToRosbridge() {
		let connectionLightHTML = document.getElementById('connection-status-light');
		let statusHTML = document.getElementById('status');

		ros = new ROSLIB.Ros({ url: `ws://${location.hostname}:9090` });

		ros.on('connection', () => {
			statusHTML.innerText = 'successful';
			connectionLightHTML.style.backgroundColor = 'green';
		});
		ros.on('close', () => {
			statusHTML.innerText = 'closed';
			connectionLightHTML.style.backgroundColor = 'black';
		});
		ros.on('error', () => {
			statusHTML.innerText = `errored out`;
			connectionLightHTML.style.backgroundColor = 'red';
		});

		rosStore.set(ros);
	}

	function disconnectFromRosbridge() {
		if (ros) {
			ros.close();
			ros = null;
		}
	}
</script>

<Navbar></Navbar>
<div class="container">
	<slot />
</div>

<style>
	:global(*) {
		margin: 0;
		padding: 0;
		box-sizing: border-box;
	}

	:root {
		font-size: 62.5%;
		--text-color: #555555;
		--background-color: #f6f6f6;
		--theme-transition-time: 0.4s ease-in-out;
		--page-display: initial;

		--section-background: #fef7f2;
	}

	:global(.dark-theme) {
		--text-color: #e6e6e6;
		--background-color: #4e4e4e;
		--section-background: #454545;
	}

	:global(body) {
		font-size: 1.6rem;
		font-family: 'Gill Sans', 'Gill Sans MT', Calibri, 'Trebuchet MS', sans-serif;
		color: var(--text-color);
		background-color: var(--background-color);
		transition: var(--theme-transition-time);
	}

	:global(.menu-active) {
		/* Hides dvh when URL visible on phones */
		/* But also casues page to go back to top */
		--page-display: none;
	}

	:global(a) {
		color: inherit;
	}

	.container {
		width: 100%;
		display: var(--page-display);
	}
	@media (min-width: 700px) {
		.container {
			display: initial;
		}
	}
</style>
