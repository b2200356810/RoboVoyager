<div class="specifications-container">
	<section class="robot">
		<div class="robot-title">
			<h2>Robot</h2>
		</div>

		<div class="robot-description">
			<p>
				RoboVoyager is a differential, two wheeled robot. It has 4 smaller supporting wheels that
				aren't operable, a base, and a stand part. The base contains a battery and motors while the
				stand has a camera and lidar mounted on it. It also has power, emergency and LED buttons
				(disabled). Emergency button is portruded and if the robot collides from the front, it will
				disable the motor functions
			</p>
		</div>

		<div class="robot-specifications">
			<div class="specification-label">Base</div>
			<div class="specification-value">50cm (height) x 50cm (width) x 50cm (length)</div>

			<div class="specification-label">Stand</div>
			<div class="specification-value">70 cm x 20 cm x 10 cm</div>

			<div class="specification-label">Base + Stand</div>
			<div class="specification-value">120 cm x 70 cm x 60 cm</div>

			<div class="specification-label">Speed</div>
			<div class="specification-value">1.5 m/s</div>

			<div class="specification-label">Locomotion</div>
			<div class="specification-value">2 x BLDC Motor, Femsan, 0,25 kw 2600 rpm 24 V</div>

			<div class="specification-label">Weight</div>
			<div class="specification-value">55 kg</div>

			<div class="specification-label">Battery</div>
			<div class="specification-value">40 AH (24 V)</div>

			<div class="specification-label">DC - DC Converter</div>
			<div class="specification-value">24 V to 5 V, 60 W</div>

			<div class="specification-label">DC - DC Converter</div>
			<div class="specification-value">24 V to 12 V, 60 W</div>
		</div>

		<div class="robot-images">
			<input type="checkbox" id="zoomCheckFooter" />
			<label for="zoomCheckFooter">
				<img class="footer-img" src="images/robot.png" alt="" />
			</label>
		</div>
	</section>

	<section class="computer gap">
		<div class="computer-title">
			<h2>Computer</h2>
		</div>

		<div class="computer-description">
			<p>
				The robot's computer is
				<a
					href="https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf21/jetson-orin/nvidia-jetson-agx-orin-technical-brief.pdf"
					>NVIDIA Jetson AGX Orin</a
				>. It specializes in delivering image processing, artificial intelligence related tasks,
				real-time sensor fusion, deep learning acceleration, making it ideal for complex robotics
				applications. Compared to traditional computers, the Jetson Orin offers superior efficiency
				and performance-per-watt, enabling longer battery life and more responsive robotic
				operations.
			</p>
		</div>

		<div class="computer-images">
			<div>
				<input type="checkbox" id="zoomCheck1" />
				<label for="zoomCheck1"><img class="" src="images/jetson/jetson_front.jpg" alt="" /></label>
			</div>

			<div>
				<input type="checkbox" id="zoomCheck2" />
				<label for="zoomCheck2"><img class="" src="images/jetson/jetson_side.jpg" alt="" /></label>
			</div>

			<div>
				<input type="checkbox" id="zoomCheck3" />
				<label for="zoomCheck3"
					><img class="" src="images/jetson/jetson_motherboard.png" alt="" /></label
				>
				<!-- https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html -->
			</div>
		</div>

		<div class="computer-specifications">
			<div class="specification">
				<h3>CPU</h3>
				<ul>
					<li>Model: Arm® Cortex®-A78AE v8.2 64-bit</li>
					<li>Core count: 12</li>
					<li>Max frequency: 2.2 GHz</li>
					<li>Memory: L2 (3MB) + L3 (6MB)</li>
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
					<li>Graphics Features: DirectX 12, OpenGL 4.6, OpenCL 3, Vulkan 1.3, CUDA 8.6</li>
					<li>TDP: 60W</li>
					<li>Dimensions: 100mm (lenght) x 87mm (width)</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Power</h3>
				<ul>
					<li>Profiles: 15W, 30W, 50W, MAXN (60W)</li>
				</ul>
			</div>

			<div class="specification">
				<h3>I/O</h3>
				<ul>
					<li>4 x UART</li>
					<li>3 x SPI</li>
					<li>4 x I2S</li>
					<li>8 x I2C</li>
					<li>2 x CAN</li>
					<li>PWM</li>
					<li>GPIOs</li>
					<li>DMIC & DSPK</li>
				</ul>
			</div>

			<div class="specification">
				<h3>Display (1 slot)</h3>
				<ul>
					<li>HDMI 2.1 OR</li>
					<li>(+MST)/eDP 1.4a OR</li>
					<li>8K60 multi-mode DP 1.4a OR</li>
				</ul>
			</div>

			<div class="specification">
				<h3>PCIe (Gen4)</h3>
				<ul>
					<li>2 x 8 lanes</li>
					<li>1 x 4 lanes</li>
					<li>2 x 1 lane</li>
				</ul>
			</div>

			<div class="specification">
				<h3>CSI Camera</h3>
				<ul>
					<li>Up to 6 cameras (16 via virtual channels)</li>
					<li>16 lanes MIPI CSI-2</li>
					<li>D-PHY 2.1 (up to 40Gbps) C-PHY 2.0 (up to 164Gbps)</li>
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
					<li>1x GbE</li>
					<li>1x 10GbE</li>
				</ul>
			</div>
		</div>
	</section>

	<section class="peripherals">
		<div class="peripherals-title">
			<h2>Peripherals</h2>
		</div>
		<div class="peripherals-body">
			<div class="zed peripheral gap">
				<div class="zed-header gap">
					<h3>Zed 2 Camera</h3>
					<p>
						The <a
							href="https://store.stereolabs.com/cdn/shop/files/ZED_2_Datasheet.pdf?v=14666199487818461309"
							>Zed 2</a
						> is a stereo camera, built for depth perception, motion tracking, and spatial AI, making
						it a versatile tool for various applications. It boasts an Accelerometer, Barometer, Magnetometer,
						IMU, a 120° Wide-Angle view, resolutions up to 4K, and more. Zed 2 is a perfect camera in
						conjuction with Jetson Orin products and ROS projects.
					</p>
				</div>
				<div class="zed-body zed-images">
					<div>
						<input type="checkbox" id="zedCheck1" />
						<label for="zedCheck1">
							<img class="" src="images/zed/zed_color_front.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck2" />
						<label for="zedCheck2">
							<img class="" src="images/zed/zed_color_bottom.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck3" />
						<label for="zedCheck3">
							<img class="" src="images/zed/zed_front_diagram.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck4" />
						<label for="zedCheck4">
							<img class="" src="images/zed/zed_back_diagram.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck5" />
						<label for="zedCheck5">
							<img class="" src="images/zed/zed_mount_diagram.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck6" />
						<label for="zedCheck6">
							<img class="" src="images/zed/zed_sensors_diagram.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck7" />
						<label for="zedCheck7">
							<img class="" src="images/zed/zed_sensors.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck8" />
						<label for="zedCheck8">
							<img class="" src="images/zed/zed_sensors_physical.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck9" />
						<label for="zedCheck9">
							<img class="" src="images/zed/zed_temperature.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck10" />
						<label for="zedCheck10">
							<img class="" src="images/zed/zed_barometer.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck11" />
						<label for="zedCheck11">
							<img class="" src="images/zed/zed_imu.png" alt="" />
						</label>
					</div>

					<div>
						<input type="checkbox" id="zedCheck12" />
						<label for="zedCheck12">
							<img class="" src="images/zed/zed_magnometer.png" alt="" />
						</label>
					</div>

					<!-- <img class="" src="images/zed/zed_color_front.png" alt="" />
					<img class="" src="images/zed/zed_color_bottom.png" alt="" />
					<img class="" src="images/zed/zed_front_diagram.png" alt="" />
					<img class="" src="images/zed/zed_back_diagram.png" alt="" />
					<img class="" src="images/zed/zed_mount_diagram.png" alt="" />
					<img class="" src="images/zed/zed_sensors_diagram.png" alt="" />
					<img class="" src="images/zed/zed_sensors.png" alt="" />
					<img class="" src="images/zed/zed_sensors_physical.png" alt="" />
					<img class="" src="images/zed/zed_temperature.png" alt="" />
					<img class="" src="images/zed/zed_barometer.png" alt="" />
					<img class="" src="images/zed/zed_imu.png" alt="" />
					<img class="" src="images/zed/zed_magnometer.png" alt="" /> -->

					<!-- <div class="row">
						<img class="" src="images/zed/zed_color_front.png" alt="" />
						<img class="" src="images/zed/zed_color_bottom.png" alt="" />
					</div>

					<div class="row">
						<img class="" src="images/zed/zed_front_diagram.png" alt="" />
						<img class="" src="images/zed/zed_back_diagram.png" alt="" />
						<img class="" src="images/zed/zed_mount_diagram.png" alt="" />
						<img class="" src="images/zed/zed_sensors_diagram.png" alt="" />
					</div>

					<div class="row">
						<img class="" src="images/zed/zed_sensors.png" alt="" />
						<img class="" src="images/zed/zed_sensors_physical.png" alt="" />
						<img class="" src="images/zed/zed_camera.png" alt="" />
						<img class="" src="images/zed/zed_temperature.png" alt="" />
						<img class="" src="images/zed/zed_barometer.png" alt="" />
						<img class="" src="images/zed/zed_imu.png" alt="" />
						<img class="" src="images/zed/zed_magnometer.png" alt="" />
					</div> -->
				</div>
			</div>

			<div class="lidar peripheral gap">
				<div class="lidar-header gap">
					<h3>Lidar</h3>
					<p>
						The <a href="https://www.mapix.com/lidar-scanner-sensors/velodyne/velodyne-vlp16-lite/"
							>Lidar</a
						> is a useful tool in field of robotics. With the ability to scan up to 600,000 points per
						second, it can be used for mapping an area of 100m around itself. Scanning at an angle also
						provides detection of objects and roads.
					</p>
				</div>
				<div class="lidar-body lidar-images">
					<img class="" src="images/lidar.png" alt="" />
				</div>
			</div>

			<div class="gps peripheral">
				<div class="gps-header gap">
					<h3>GPS</h3>
					<p>The GPS is one of the most bla bla</p>
				</div>
				<div class="gps-body"></div>
			</div>
		</div>
	</section>
</div>

<style>
	:global(nav) {
		position: relative !important;
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
		border-bottom: 1px solid white;
	}

	.specification-label {
		font-weight: 800;
		/* align-self: end; */
	}

	.robot-images {
		/* display: flex;
		gap: 20px;
		justify-content: center;
		align-items: center; */
		position: relative;
	}

	.robot-images img {
		height: 100%;
		width: 100%;
		border: 1px solid var(--text-color);
		cursor: zoom-in;
	}

	input[type='checkbox'] {
		display: none;
	}

	input[type='checkbox']:checked ~ label > img {
		position: absolute;
		height: fit-content;
		inset: 0;
		margin: 5px auto;
		cursor: zoom-out;
		z-index: 100;
	}

	.computer-images {
		display: grid;
		grid-template-columns: repeat(auto-fit, minmax(100px, 1fr));
		gap: 20px;
		justify-items: center;
		position: relative;
	}

	.computer-images img {
		width: 100%;
		height: 100%;
		border: 1px solid var(--text-color);
		transition: transform 0.25s ease;
		cursor: zoom-in;
		background-color: white;
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
	}

	.specification ul {
		list-style-type: none;
	}

	.peripheral {
		border-bottom: 1px solid var(--text-color);
		padding-bottom: 20px;
	}

	.zed-images {
		display: grid;
		grid-template-columns: 1fr 1fr;
		gap: 10px;
		position: relative;
	}

	.zed-images img {
		cursor: zoom-in;
		width: 100%;
		border: 1px solid var(--text-color);
	}

	.lidar {
		margin: 20px 0;
	}

	.lidar-images img {
		width: 100%;
		border: 1px solid var(--text-color);
	}

	@media (min-width: 600px) {
		section {
			padding: 40px;
			font-size: 2rem;
		}
	}

	@media (min-width: 1000px) {
		section {
			padding: 100px;
			font-size: 3rem;
		}

		.peripheral {
			border-bottom: 2px solid var(--text-color);
		}

		input[type='checkbox']:checked ~ label > img {
			position: initial;
			height: 10%;
			cursor: auto;
		}

		.computer-images img {
			cursor: default;
			max-width: 500px;
		}

		.zed-images {
			align-items: center;
			gap: 50px;
			cursor: default;
		}

		.zed-images img {
			max-width: 600px;
		}

		.zed-images div {
			text-align: center;
		}
	}
</style>
