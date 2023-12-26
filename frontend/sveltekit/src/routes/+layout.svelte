<script>
	import Navbar from '../components/Navbar.svelte';
	// import { connectToIP } from '$lib/rosNodeHandler.js';
</script>

<Navbar></Navbar>
<slot />

<style>
	:global(*) {
		margin: 0;
		padding: 0;
		box-sizing: border-box;
	}

	:root {
		font-size: 62.5%;
		--text-color: #5c5c5c;
		--background-color: #ffffff;
		--theme-transition-time: 0.4s ease-in-out;
	}

	:global(.dark-theme) {
		--text-color: #e6e6e6;
		--background-color: #1a1a1a;
	}

	:global(body) {
		font-size: 1.6rem;
		font-family: 'Gill Sans', 'Gill Sans MT', Calibri, 'Trebuchet MS', sans-serif;
		color: var(--text-color);
		background-color: var(--background-color);
		transition: var(--theme-transition-time);
	}
</style>
