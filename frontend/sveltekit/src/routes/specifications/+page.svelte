<script>
	let imageIsOpen = false;
	let currentImage = '';

	function openLightbox(image) {
		currentImage = image;
		imageIsOpen = true;
	}

	let robotImages1 = [
		{ src: 'images/robot/robot1.png', alt: '' },
		{ src: 'images/robot/robot2.png', alt: '' }
	];

	let robotImages2 = [{ src: 'images/robot/robot3.png', alt: '' }];

	let computerImages1 = [
		{ src: 'images/jetson/jetson_front.jpg', alt: '' },
		{ src: 'images/jetson/jetson_side.jpg', alt: '' }
	];

	let computerImages2 = [{ src: 'images/jetson/jetson_motherboard.png', alt: '' }];

	let zedImages1 = [
		{ src: 'images/zed/zed_color_front.png', alt: '' },
		{ src: 'images/zed/zed_color_bottom.png', alt: '' },
		{ src: 'images/zed/zed_front_diagram.png', alt: '' },
		{ src: 'images/zed/zed_back_diagram.png', alt: '' },
		{ src: 'images/zed/zed_mount_diagram.png', alt: '' },
		{ src: 'images/zed/zed_sensors_diagram.png', alt: '' },
		{ src: 'images/zed/zed_sensors.png', alt: '' },
		{ src: 'images/zed/zed_sensors_physical.png', alt: '' }
	];

	let zedImages2 = [
		{ src: 'images/zed/zed_camera.png', alt: '' },
		{ src: 'images/zed/zed_temperature.png', alt: '' },
		{ src: 'images/zed/zed_barometer.png', alt: '' },
		{ src: 'images/zed/zed_imu.png', alt: '' },
		{ src: 'images/zed/zed_magnometer.png', alt: '' }
	];

	let lidarImages1 = [
		{ src: 'images/lidar/lidar1.png', alt: '' },
		{ src: 'images/lidar/lidar2.png', alt: '' },
		{ src: 'images/lidar/lidar4.png', alt: '' },
		{ src: 'images/lidar/lidar3.png', alt: '' }
	];

	let lidarImages2 = [
		{ src: 'images/lidar/lidar5.png', alt: '' },
		{ src: 'images/lidar/lidar6.png', alt: '' },
		{ src: 'images/lidar/lidar7.png', alt: '' },
		{ src: 'images/lidar/lidar8.png', alt: '' },
		{ src: 'images/lidar/lidar9.png', alt: '' }
	];

	let gpsImages1 = [
		{ src: 'images/gps/gps1.png', alt: '' },
		{ src: 'images/gps/gps2.png', alt: '' },
		{ src: 'images/gps/gps3.png', alt: '' }
	];

	let gpsImages2 = [
		{ src: 'images/gps/gps4.png', alt: '' },
		{ src: 'images/gps/gps5.png', alt: '' }
	];
</script>

<div></div>

{#if imageIsOpen}
	<div
		class="lightbox"
		on:click={() => {
			imageIsOpen = false;
		}}
	>
		<img class="lightbox-close" src={currentImage} alt="Lightbox Image" />
	</div>
{/if}

<div class="specifications-container">
	<section class="robot">
		<div class="title">
			<h2>Robot</h2>
		</div>

		<div class="description">
			<p>
				RoboVoyager is a differential robot. It has 2 differential wheels and 4 smaller supporting
				wheels that aren't operable, a base, and a stand part. The base contains a battery and
				motors inside it, while the stand has a camera and lidar mounted on top. It also has a power
				button, emergency and LED buttons, of which 2 are disabled, and the red one powers the
				motors when pressed in. Emergency button is portruded out and if the robot collides from the
				front, it will disable the motor functions
			</p>
		</div>

		<div class="robot-specifications">
			<div class="specification-label">Dimensions</div>
			<div class="specification-value">774 cm (length) x 470 cm (width) x 795 cm (height)</div>

			<div class="specification-label">Speed</div>
			<div class="specification-value">1.5 m/s</div>

			<div class="specification-label">Weight</div>
			<div class="specification-value">55 kg</div>

			<div class="specification-label">Battery</div>
			<div class="specification-value">40 AH (24 V)</div>

			<div class="specification-label">DC - DC Converter</div>
			<div class="specification-value">24 V to 5 V, 60 W</div>

			<div class="specification-label">DC - DC Converter</div>
			<div class="specification-value">24 V to 12 V, 60 W</div>

			<div class="specification-label">Locomotion</div>
			<div class="specification-value">2 x BLDC Motor, Femsan, 0,25 kw 2600 rpm 24 V</div>

			<div class="specification-label">Motor Driver</div>
			<div class="specification-value">SBL23XX, Roboteq</div>
		</div>

		<div class="images">
			<div class="column row">
				{#each robotImages1 as image}
					<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
				{/each}
			</div>

			<div class="column row">
				{#each robotImages2 as image}
					<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
				{/each}
			</div>
		</div>
	</section>

	<section class="computer">
		<div class="title">
			<h2>Computer</h2>
		</div>

		<div class="description">
			<p>
				The robot's computer is
				<a
					href="https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf21/jetson-orin/nvidia-jetson-agx-orin-technical-brief.pdf"
					>NVIDIA Jetson AGX Orin</a
				>. It specializes in delivering image processing, artificial intelligence related tasks,
				real-time sensor fusion, deep learning acceleration, making it ideal development kit for
				complex robotics applications. Compared to traditional computers, the Jetson Orin offers
				superior efficiency and performance-per-watt, enabling longer battery life and more
				responsive robotic operations
			</p>
		</div>

		<!-- https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html -->
		<div class="images">
			<div class="column row">
				{#each computerImages1 as image}
					<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
				{/each}
			</div>

			<div class="column row">
				{#each computerImages2 as image}
					<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
				{/each}
			</div>
		</div>

		<div class="datasheet">
			<h3>
				<a
					href="https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf21/jetson-orin/nvidia-jetson-agx-orin-technical-brief.pdf"
					>Datasheet</a
				>
			</h3>
		</div>
		<div class="computer-specifications">
			<div class="specification">
				<h3>Dimensions</h3>
				<ul>
					<li>Dimensions: 110 mm (length) x 110 mm (width) 72 mm (height)</li>
				</ul>
			</div>

			<div class="specification">
				<h3>CPU</h3>
				<ul>
					<li>
						Model: <a href="https://developer.arm.com/Processors/Cortex-A78AE"
							>Arm® Cortex®-A78AE v8.2 64-bit</a
						>
					</li>
					<li>Core count: 12</li>
					<li>Max frequency: 2.2 GHz</li>
					<li>Memory: L2 (3 MB) + L3 (6 MB)</li>
				</ul>
			</div>

			<div class="specification">
				<h3>GPU</h3>
				<ul>
					<li>
						Model: <a
							href="https://www.techpowerup.com/gpu-specs/jetson-agx-orin-64-gb.c4085#:~:text=NVIDIA%20has%20paired%2064%20GB,is%20running%20at%201600%20MHz."
							>GA10B</a
						>
					</li>
					<li>Cores: 2048 (CUDA), 64 (Tensor)</li>
					<li>Memory: 64 GB LPDDR5 (1300 MHz) 256 bit bus</li>
					<li>Graphics features: DirectX 12, OpenGL 4.6, OpenCL 3, Vulkan 1.3, CUDA 8.6</li>
					<li>TDP: 60 W</li>
					<li>Dimensions: 100 mm (length) x 87 mm (width)</li>
				</ul>
			</div>

			<div class="specification">
				<h3>AI Performance</h3>
				<ul>
					<li>275 TOPS (INT8)</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Display</h3>
				<ul>
					<li>1 x 8K60 multi-mode DP | 1.4a (+MST)/eDP | 1.4a/HDMI 2.1</li>
				</ul>
			</div>

			<div class="specification">
				<h3>CSI Camera</h3>
				<ul>
					<li>Up to 6 cameras (16 via virtual channels)</li>
					<li>16 lanes MIPI CSI-2</li>
					<li>D-PHY 2.1 (up to 40Gbps), C-PHY 2.0 (up to 164Gbps)</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Video Encode</h3>
				<ul>
					<li>2 x 4K60 | 4 x 4K30 | 8 x 1080p60 | 16 x 1080p30 (H.265) H.264, AV1</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Video Decode</h3>
				<ul>
					<li>
						1 x 8K30 | 3 x 4K60 | 7 x 4K30 | 11 x 1080p60| 22 x 1080p30 (H.265) H.264, VP9, AV1
					</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Storage</h3>
				<ul>
					<li>64GB eMMC 5.1</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Networking</h3>
				<ul>
					<li>1 x GbE</li>
					<li>1 x 10GbE</li>
				</ul>
			</div>

			<div class="specification">
				<h3>UPHY</h3>
				<ul>
					<li>2 x 8 | 1 x 4 | 2 x 1 (PCIe Gen4, Root Port & Endpoint), 3 x USB 3.2</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Other I/O</h3>
				<ul>
					<li>
						4 x USB 2.0 | 4 x UART | 3 x SPI | 4 x I2S | 8 x I2C | 2 x CAN, DMIC & DSPK, GPIOs
					</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Power</h3>
				<ul>
					<li>Profiles: 15 W, 30 W, 50 W, MAXN (60 W)</li>
				</ul>
			</div>
		</div>
	</section>

	<section class="peripherals">
		<div class="title">
			<h2>Peripherals</h2>
		</div>

		<div class="zed peripheral">
			<div class="title">
				<h2>Camera</h2>
			</div>
			<div class="description">
				<p>
					The <a href="https://www.stereolabs.com/products/zed-2">Zed 2</a> is a stereo camera, built
					for depth perception, motion tracking, and spatial AI, making it a versatile tool for various
					applications. It boasts an Accelerometer, Barometer, Magnetometer, Inertial Measurement Unit
					(IMU), a 120° Wide-Angle view, and various resolutions, up to 4K. The Zed 2 is a perfect camera
					in combination with Jetson Orin for ROS projects
				</p>
			</div>
			<div class="datasheet">
				<h3>
					<a
						href="https://store.stereolabs.com/cdn/shop/files/ZED_2_Datasheet.pdf?v=14666199487818461309"
						>Datasheet</a
					>
				</h3>
			</div>
			<div class="images">
				<div class="column row">
					{#each zedImages1 as image}
						<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
					{/each}
				</div>

				<div class="column row">
					{#each zedImages2 as image}
						<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
					{/each}
				</div>
			</div>
		</div>

		<div class="lidar peripheral">
			<div class="title">
				<h2>Lidar</h2>
			</div>
			<div class="description">
				<p>
					The <a href="https://www.robosense.ai/en/rslidar/RS-Helios">Robosense RS Lidar 16</a> is used
					for detecting terrain and surroundings of the robot. It rapidly emits pulse light waves (288,000
					pts/s), that bounce back to the lidar head. It then calculates the time travelled for each
					wave and forms a precise 3D map (point cloud). The Helios 16 lidar is suitable for variety
					of applications, including: autonomous driving, vehicle-to-roadside (V2R, wireless vehicle
					communication), robototics and industrial uses. Its main features are: 150m range, 2cm range
					accuracy and temperature resistance (-30°C to +60°C).
				</p>
			</div>
			<div class="datasheet">
				<h3>
					<a href="https://cdn.robosense.cn/20200723161715_42428.pdf">Datasheet</a>
				</h3>
			</div>
			<div class="images">
				<div class="column row">
					{#each lidarImages1 as image}
						<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
					{/each}
				</div>
				<div class="column row">
					{#each lidarImages2 as image}
						<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
					{/each}
				</div>
			</div>
		</div>

		<div class="gps peripheral">
			<div class="title">
				<h2>GPS</h2>
			</div>
			<div class="description">
				<p>
					The <a href="https://www.pololu.com/product/2138">LOCOSYS LS20031 GPS</a> receiver integrates
					a MediaTek 66-channel chip with a ceramic antenna to create a complete GPS module that can
					track up to 66 GPS satellites at a time. The GPS module supports up to a 10Hz update rate,
					a built-in rechargeable battery for rapid satellite acquisition (external power is still required
					for normal operation), and more than 6 different NMEA ASCII sentences that are output to a
					3.3 V TTL-level serial port.
				</p>
			</div>

			<div class="datasheet">
				<h3><a href="https://datasheetspdf.com/datasheet-pdf/900733/LS20031.html">Datasheet</a></h3>
			</div>

			<div class="images">
				<div class="column row">
					{#each gpsImages1 as image}
						<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
					{/each}
				</div>
				<div class="column row">
					{#each gpsImages2 as image}
						<img src={image.src} alt={image.alt} on:click={() => openLightbox(image.src)} />
					{/each}
				</div>
			</div>
		</div>
	</section>
</div>

<style>
	.lightbox {
		position: fixed;
		top: 0;
		left: 0;
		width: 100%;
		height: 100%;
		background: rgba(0, 0, 0, 0.8);
		display: flex;
		justify-content: center;
		align-items: center;
		z-index: 1000;
	}
	.lightbox img {
		max-width: 90%;
		max-height: 90%;
	}

	.lightbox-close {
		cursor: zoom-out;
	}

	.datasheet {
		text-align: center;
	}

	.images {
		display: grid;
		grid-template-columns: 1fr 1fr;
		gap: 10px;
		justify-items: center;
		margin: auto;
	}
	.images .column {
		display: flex;
		flex-direction: column;
		gap: 10px;
	}

	.images img {
		width: 100%;
		border: 1px solid var(--text-color);
		cursor: zoom-in;
	}

	.specifications-container {
		padding: 20px 10px;
	}

	section {
		display: grid;
		gap: 20px;
		padding: 20px;
		margin-bottom: 20px;
		border: 1px solid var(--text-color);
		border-radius: 15px;
		box-shadow: 0px 10px 15px -3px rgba(0, 0, 0, 0.3);
		background-color: var(--section-background);
	}

	.robot-specifications {
		display: grid;
		grid-template-columns: 1fr 1fr;
		gap: 10px 5px;
	}

	.specification-label,
	.specification-value {
		border-bottom: 1px solid var(--text-color);
	}

	.specification-label {
		font-weight: 800;
		/* align-self: end; */
	}

	.computer-specifications {
		display: grid;
		gap: 20px;
		grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
	}

	.specification {
		padding: 20px;
		border: 1px solid var(--text-color);
		border-radius: 15px;
		display: grid;
		gap: 10px;
		box-shadow: 0px 10px 10px -8px rgba(0, 0, 0, 0.3);
	}

	.specification ul {
		list-style-type: none;
	}

	.peripheral {
		border-bottom: 1px solid var(--text-color);
		display: grid;
		gap: 10px;
		padding-bottom: 20px;
	}

	.lidar {
		margin: 20px 0;
	}

	@media (min-width: 700px) {
		.specifications-container {
			padding: 40px;
		}
		section {
			padding: 40px;
			font-size: 2rem;
		}

		.robot-specifications {
			gap: 20px;
			margin-bottom: 20px;
		}

		.images {
			padding: 20px;
		}

		.images .column {
			flex-direction: row;
			flex-wrap: wrap;
			justify-content: center;
			align-items: center;
			gap: 20px;
		}

		.images img {
			max-width: 400px;
		}
	}
</style>
