<script>
	import { onMount } from 'svelte';
	import { page } from '$app/stores';

	let activePage = '';
	let isMenuActive = false;
	let isDarkTheme = false;
	let isDesktop = false;

	onMount(() => {
		activePage = $page.url.pathname;
		isDarkTheme = localStorage.getItem('theme') === 'dark';
		updateTheme();
		isDesktop = window.innerWidth >= 700;
		window.addEventListener('resize', handleResize);
	});

	function handleResize() {
		isDesktop = window.innerWidth >= 700;
		if (window.innerWidth > 700) {
			document.body.classList.remove('menu-active');
			isMenuActive = false;
		}
	}

	function toggleMenu() {
		isMenuActive = !isMenuActive;
		if (isMenuActive) {
			document.body.classList.add('menu-active');
		} else {
			document.body.classList.remove('menu-active');
		}
	}

	function toggleTheme() {
		isDarkTheme = !isDarkTheme;
		localStorage.setItem('theme', isDarkTheme ? 'dark' : 'light');
		updateTheme();
	}

	function updateTheme() {
		const favicon = document.querySelector('[rel=icon]');
		if (isDarkTheme) {
			favicon.setAttribute('href', '/images/roboDark.svg');
			document.body.classList.add('dark-theme');
		} else {
			favicon.setAttribute('href', '/images/roboLight.svg');
			document.body.classList.remove('dark-theme');
		}
	}

	function handleNavLinkClick(event) {
		const links = document.querySelectorAll('nav ul li a');
		links.forEach((link) => link.classList.remove('active'));
		event.target.classList.add('active');
		if (window.innerWidth < 700) toggleMenu();
	}
</script>

<nav>
	<button id="hamburger-button" on:click={toggleMenu}>&#9776;</button>
	<div id="connection-status">
		<div id="connection-status-light"></div>
		<div>Connection: <code id="status">N/A</code></div>
	</div>
	<ul class:active={isMenuActive}>
		<li>
			<a
				href="/"
				class={activePage === '/' ? 'active' : ''}
				on:click={handleNavLinkClick}
				tabindex={isDesktop || isMenuActive ? '0' : '-1'}>Home</a
			>
		</li>
		<li>
			<a
				href="/monitoring"
				class={activePage === '/monitoring' ? 'active' : ''}
				on:click={handleNavLinkClick}
				tabindex={isDesktop || isMenuActive ? '0' : '-1'}>Monitoring</a
			>
		</li>
		<li>
			<a
				href="/controls"
				class={activePage === '/controls' ? 'active' : ''}
				on:click={handleNavLinkClick}
				tabindex={isDesktop || isMenuActive ? '0' : '-1'}>Controls</a
			>
		</li>
		<li>
			<a
				href="/specifications"
				class={activePage === '/specifications' ? 'active' : ''}
				on:click={handleNavLinkClick}
				tabindex={isDesktop || isMenuActive ? '0' : '-1'}>Specifications</a
			>
		</li>
		<li>
			<a
				href="/contact"
				class={activePage === '/contact' ? 'active' : ''}
				on:click={handleNavLinkClick}
				tabindex={isDesktop || isMenuActive ? '0' : '-1'}>Contact</a
			>
		</li>
	</ul>
	<button id="theme-button" on:click={toggleTheme}>
		<div></div>
	</button>
</nav>

<style>
	:root {
		--navbar-background: #ffffff;
		--hamburger-hover: black;
		--theme-button-border: 1px solid #b0b0b0;
		--theme-button-border-hover: 1px solid #212121;
		--theme-button-position: translate(2px, 0%);
		--theme-button-div-background: #e6f1ff;
		--theme-button-icon: url('/images/moon.svg');
	}

	:global(.dark-theme) {
		--navbar-background: #2e2e2e;
		--hamburger-hover: white;
		--theme-button-border: 1px solid #383838;
		--theme-button-border-hover: 1px solid #e6e6e6;
		--theme-button-position: translate(30px, 0%);
		--theme-button-div-background: #383838;
		--theme-button-icon: url('/images/sun.svg');
	}

	:global(body) {
		margin-top: 80px;
	}

	nav {
		position: fixed;
		z-index: 100;
		height: 80px;
		width: 100%;
		margin-top: -80px;
		padding: 0 20px;
		display: flex;
		justify-content: space-between;
		background-color: var(--navbar-background);
		transition: var(--theme-transition-time);
		box-shadow: 0px 0px 5px 1px rgba(0, 0, 0, 0.75);
	}

	#hamburger-button {
		height: fit-content;
		align-self: center;
		font-size: 26px;
		padding: 5px;
		border: none;
		cursor: pointer;
		color: var(--text-color);
		transition: var(--theme-transition-time);
		background-color: var(--navbar-background);
	}

	#hamburger-button:hover {
		color: var(--hamburger-hover);
	}

	#connection-status {
		align-self: center;
		color: var(--text-color);
		transition: 2s;
		display: flex;
		justify-content: center;
		align-items: center;
		gap: 6px;
	}

	#connection-status-light {
		height: 10px;
		width: 10px;
		border-radius: 50%;
		background-color: gray;
	}

	#theme-button {
		height: 30px;
		width: 60px;
		align-self: center;
		cursor: pointer;
		border-radius: 15px;
		border: var(--theme-button-border);
		transition:
			border 0.3s,
			background-color var(--theme-transition-time);
		background-color: var(--navbar-background);
	}

	#theme-button:hover {
		border: var(--theme-button-border-hover);
	}

	#theme-button div {
		height: 26px;
		width: 26px;
		padding: 4px;
		border-radius: 50%;
		transition: 0.5s ease-out;
		content: var(--theme-button-icon);
		transform: var(--theme-button-position);
		background-color: var(--theme-button-div-background);
	}

	nav ul {
		position: fixed;
		inset: 0;
		height: calc(100vh - 85px);
		transform: translate(0%, -100%);
		z-index: 100;
		display: flex;
		flex-direction: column;
		gap: 20px;
		justify-content: center;
		text-align: center;
		list-style: none;
		background-color: var(--background-color);
		transition:
			background-color var(--theme-transition-time),
			transform 0.8s ease-in-out;
	}

	nav ul.active {
		transform: translate(0%, 81px);
	}

	nav ul li a {
		font-size: 3rem;
		padding: 5px;
		color: var(--text-color);
		text-decoration: none;
		background: linear-gradient(var(--text-color), var(--text-color));
		background-size: 0% 1px;
		background-position: 50% 100%;
		background-repeat: no-repeat;
		transition: background-size 400ms;
	}

	nav ul li a:hover {
		background-size: 100% 0.1em;
	}

	nav ul li a.active {
		color: red;
		background: linear-gradient(red, red);
		background-size: 0% 1px;
		background-position: 50% 100%;
		background-repeat: no-repeat;
		transition: background-size 400ms;
	}

	nav ul li a.active:hover {
		background-size: 100% 0.1em;
	}

	@media (min-width: 700px) {
		:global(body) {
			margin-top: 0;
		}
		nav {
			position: initial;
			margin-top: 0;
		}
		nav ul,
		nav ul.active {
			position: initial;
			flex-direction: row;
			height: 100%;
			gap: 10px;
			align-items: center;
			transform: translate(0%);
			background-color: var(--navbar-background);
		}
		nav ul li a {
			font-size: 2rem;
		}

		#hamburger-button {
			display: none;
		}
	}
</style>
