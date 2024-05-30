<div class="home-container">
	<section class="hero">
		<div class="lines">
			<div class="line"></div>
			<div class="line"></div>
			<div class="line"></div>
			<div class="line"></div>
		</div>
		<h1>RoboVoyager</h1>
		<p>Your interface to the robot</p>
	</section>

	<section class="about">
		<h2>About</h2>
		<p>
			In the world of teleoperations, being able to remotely control robots, as well as monitor
			their sensors and other various data, is a must. This interface makes it easy for the users to
			do so
		</p>
	</section>

	<section class="how-it-works">
		<h2>How it Works</h2>
		<p>
			Make sure the connection is established by connecting to the robot's network and IP from your
			phone or computer. In the Robot page, you can toggle features on or off by clicking on their
			buttons
		</p>
	</section>

	<section class="features">
		<h2>Features</h2>

		<div class="feature">
			<h3>Controls</h3>
			<p>
				A Virtual Joystick is provided to remotely control the robot. It sends a vector of movement
				to the robot by publishing it to the <code>/cmd_vel</code> (command velocity) topic. There
				are 8 possible directions, including moving diagonally. RoboVoyager is a
				<a href="https://medium.com/manual-robotics/drives-76c2b2dac97c">differential</a> robot, so one
				wheel's speed is "differentiated" from the other and that's how it can rotate around and move
				in a curvy line
			</p>
			<img src="images/differential_drive.png" alt="" />
			<div>
				<svg xmlns="http://www.w3.org/2000/svg" fill="currentColor" viewBox="0 0 24 24"
					><g id="SVGRepo_bgCarrier" stroke-width="0"></g><g
						id="SVGRepo_tracerCarrier"
						stroke-linecap="round"
						stroke-linejoin="round"
					></g><g id="SVGRepo_iconCarrier"
						><path
							d="M20 7h-3V4c0-1.103-.897-2-2-2H9c-1.103 0-2 .897-2 2v3H4c-1.103 0-2 .897-2 2v6c0 1.103.897 2 2 2h3v3c0 1.103.897 2 2 2h6c1.103 0 2-.897 2-2v-3h3c1.103 0 2-.897 2-2V9c0-1.103-.897-2-2-2zm0 8h-5v4h.001v1H9v-5H4V9h5V4h6v5h5v6z"
						></path><path d="M8 14v-4l-3 2zm8 0 3-2-3-2zm-6-6h4l-2-3zm2 11 2-3h-4z"></path><circle
							cx="12"
							cy="12"
							r="2"
						></circle></g
					></svg
				>
			</div>
		</div>

		<div class="feature">
			<h3>Video Streaming</h3>
			<p>
				When a user is remotely controling a robot over a distance, a video stream is essential.
				Video streaming is offered at a low resolution for faster viewing speed. Raw camera footage
				is first resized and then converted into JPGs. Robot's computer only sends data after a user
				subscribes to the <code>/video_streaming</code> topic
			</p>
			<div>
				<svg xmlns="http://www.w3.org/2000/svg" fill="currentColor" viewBox="0 0 16 16">
					<path
						fill-rule="evenodd"
						d="M0 5a2 2 0 0 1 2-2h7.5a2 2 0 0 1 1.983 1.738l3.11-1.382A1 1 0 0 1 16 4.269v7.462a1 1 0 0 1-1.406.913l-3.111-1.382A2 2 0 0 1 9.5 13H2a2 2 0 0 1-2-2zm11.5 5.175 3.5 1.556V4.269l-3.5 1.556zM2 4a1 1 0 0 0-1 1v6a1 1 0 0 0 1 1h7.5a1 1 0 0 0 1-1V5a1 1 0 0 0-1-1z"
					/>
				</svg>
			</div>
		</div>

		<div class="feature">
			<h3>Sensors</h3>
			<p>
				Monitor sensor data from the robot. The most useful ones are battery levels and upload rate
				during video streaming
			</p>
			<div>
				<svg xmlns="http://www.w3.org/2000/svg" fill="currentColor" viewBox="0 0 16 16">
					<path
						d="M14.5 3a.5.5 0 0 1 .5.5v9a.5.5 0 0 1-.5.5h-13a.5.5 0 0 1-.5-.5v-9a.5.5 0 0 1 .5-.5zm-13-1A1.5 1.5 0 0 0 0 3.5v9A1.5 1.5 0 0 0 1.5 14h13a1.5 1.5 0 0 0 1.5-1.5v-9A1.5 1.5 0 0 0 14.5 2z"
					/>
					<path
						d="M5 8a.5.5 0 0 1 .5-.5h7a.5.5 0 0 1 0 1h-7A.5.5 0 0 1 5 8m0-2.5a.5.5 0 0 1 .5-.5h7a.5.5 0 0 1 0 1h-7a.5.5 0 0 1-.5-.5m0 5a.5.5 0 0 1 .5-.5h7a.5.5 0 0 1 0 1h-7a.5.5 0 0 1-.5-.5m-1-5a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0M4 8a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0m0 2.5a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0"
					/>
				</svg>
			</div>
		</div>

		<div class="feature">
			<h3>Data Saving</h3>
			<p>
				Save and analyze collected data for later. Data saver node collects information from Lidar,
				GPS, and Zed2 camera (but the camera has many more sensors built inside it). Be cautious of
				prolonged use, as 10 seconds fills up around 100 MBs
			</p>
			<div>
				<svg
					xmlns="http://www.w3.org/2000/svg"
					fill="currentColor"
					class="bi bi-pencil-square"
					viewBox="0 0 16 16"
				>
					<path
						d="M15.502 1.94a.5.5 0 0 1 0 .706L14.459 3.69l-2-2L13.502.646a.5.5 0 0 1 .707 0l1.293 1.293zm-1.75 2.456-2-2L4.939 9.21a.5.5 0 0 0-.121.196l-.805 2.414a.25.25 0 0 0 .316.316l2.414-.805a.5.5 0 0 0 .196-.12l6.813-6.814z"
					/>
					<path
						fill-rule="evenodd"
						d="M1 13.5A1.5 1.5 0 0 0 2.5 15h11a1.5 1.5 0 0 0 1.5-1.5v-6a.5.5 0 0 0-1 0v6a.5.5 0 0 1-.5.5h-11a.5.5 0 0 1-.5-.5v-11a.5.5 0 0 1 .5-.5H9a.5.5 0 0 0 0-1H2.5A1.5 1.5 0 0 0 1 2.5z"
					/>
				</svg>
			</div>
		</div>
	</section>

	<section class="other-projects">
		<h2>Other projects</h2>
		<div class="project">
			<h3>
				<a href="https://tuanacetinkaya.github.io/Moborobot/" class="cta-button">Moborobot</a>
			</h3>
			<p>
				This project was about making the robot go from point A to point B, by using Simultaneous
				Localization and Mapping (SLAM) and object avoidance algorithm
			</p>
		</div>

		<div class="project">
			<h3>
				<a href="https://github.com/MAli7319/Moborobo_GMapping" class="cta-button"
					>Moborobo GMapping</a
				>
			</h3>
			<p>
				This project is an upgraded version of Moborobot. Navigating from point A to point B now
				works without mapping prior area, but tiles of an offline map are still necessary as a
				general overview of the area, and designating destinations
			</p>
		</div>
	</section>

	<footer></footer>
</div>

<style>
	:root {
		--line-color: rgba(0, 0, 0, 0.3);
		--line-travel: rgba(0, 0, 0, 0);
	}

	:global(.dark-theme) {
		--line-color: rgba(255, 255, 255, 0.2);
		--line-travel: rgba(255, 255, 255, 0);
	}

	.hero {
		height: calc(100dvh - 80px);
		display: flex;
		flex-direction: column;
		justify-content: center;
		align-items: center;
		position: relative;
		color: var(--text-color);
		font-size: 2.2rem;
		background-color: var(--section-background);
		box-shadow: 0px 5px 15px 2px rgba(0, 0, 0, 0.3);
		margin-bottom: 40px;
	}

	.lines {
		position: absolute;
		top: 0;
		left: 0;
		right: 0;
		height: 100%;
		margin: auto;
		/* width: 90vw; */
	}
	.line {
		position: absolute;
		height: 100%;
		width: 2px;
		top: 0;
		background: var(--line-color);
		overflow: hidden;
	}
	.line::after {
		content: '';
		display: block;
		position: absolute;
		height: 5vh;
		width: 100%;
		top: -50%;
		left: 0;
		background: linear-gradient(
			to bottom,
			var(--line-travel) 0%,
			var(--line-color) 75%,
			var(--line-color) 100%
		);

		animation: drop 8s 0s infinite;
		animation-fill-mode: forwards;
		animation-timing-function: cubic-bezier(0.4, 0.26, 0, 0.97);
	}
	.line:nth-child(1) {
		left: 10%;
	}

	.line:nth-child(2) {
		left: 20%;
	}

	.line:nth-child(3) {
		left: 80%;
	}

	.line:nth-child(4) {
		left: 90%;
	}
	@keyframes drop {
		0% {
			top: -50%;
		}
		100% {
			top: 110%;
		}
	}

	section:not(:first-child) {
		display: grid;
		gap: 10px;
		padding: 20px;
		margin: 20px 10px;
		background-color: var(--section-background);
		border: 1px solid var(--text-color);
		border-radius: 15px;
		box-shadow: 0px 10px 15px -3px rgba(0, 0, 0, 0.3);
	}

	.features {
		gap: 20px !important;
	}

	.feature {
		display: grid;
		grid-template-columns: 25px 1fr;
		grid-template-rows: fit-content 1fr;
		gap: 10px 10px;
		grid-auto-flow: row;
		grid-template-areas:
			'SVG Title'
			'Text Text'
			'IMG IMG';
	}

	.feature h3 {
		grid-area: Title;
		border-bottom: 1px solid var(--text-color);
		align-self: center;
	}

	.feature p {
		grid-area: Text;
	}

	.feature div {
		grid-area: SVG;
		display: flex;
		justify-content: center;
		align-items: center;
	}

	.feature img {
		grid-area: IMG;
		width: 100%;
		border: 1px solid var(--text-color);
	}

	.feature code {
		padding: 1px;
		background-color: var(--text-color);
		color: var(--section-background);
		opacity: 0.5;
		border-radius: 4px;
		font-size: 1.4rem;
	}

	.project {
		display: grid;
		gap: 5px;
	}

	@media (min-width: 700px) {
		.home-container {
			font-size: 2rem;
		}
		.hero {
			font-size: 4rem;
		}

		section:not(:first-child) {
			padding: 20px;
			margin: 20px;
		}

		.feature {
			grid-template-columns: 60px 1fr;
			gap: 10px 20px;
		}

		.feature code {
			font-size: 1.7rem;
		}

		.feature img {
			max-width: 900px;
			margin: 20px auto;
		}
	}
</style>
