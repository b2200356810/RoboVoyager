<div class="specifications-container">
	<h2>Specifications</h2>

	<div class="specification-section">
		<h3>Differential Robot</h3>
		<ul>
			<li><strong>Height:</strong> 1.5m</li>
			<li><strong>Width:</strong> .7m</li>
			<li><strong>Length:</strong> 1m</li>
			<li><strong>Acceleration:</strong> 2m/s^2</li>
			<li><strong>Top Speed:</strong> 5m/s</li>
		</ul>
	</div>

	<div class="specification-section">
		<h3>
			Computer: <a
				href="https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/"
				>Nvidia Jetson Orin</a
			>
		</h3>
		<ul>
			<!-- https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/ -->
			<li><strong>CPU:</strong> 12-core Arm® Cortex®-A78AE v8.2 64-bit CPU 3MB L2 + 6MB L3</li>
			<li><strong>GPU:</strong> 2048-core NVIDIA Ampere architecture GPU with 64 Tensor Cores</li>
			<li><strong>Power:</strong> 15W - 60W</li>
			<li>
				<strong>I/O:</strong> 4x UART, 3x SPI, 4x I2S, 8x I2C, 2x CAN, PWM, DMIC & DSPK, GPIOs
			</li>
			<li><strong>Display:</strong> 1x 8K60 multi-mode DP 1.4a (+MST)/eDP 1.4a/HDMI 2.1</li>
			<li><strong>PCIe:</strong> Up to 2 x8 + 1 x4 + 2 x1 (PCIe Gen4, Root Port, & Endpoint)</li>
			<li>
				<strong>CSI Camera:</strong> Up to 6 cameras (16 via virtual channels) 16 lanes MIPI CSI-2 D-PHY
				2.1 (up to 40Gbps) | C-PHY 2.0 (up to 164Gbps)
			</li>
			<li><strong>Storage:</strong> 64GB eMMC 5.1</li>
			<li><strong>Networking:</strong> 1x GbE 1x 10GbE</li>
		</ul>
	</div>

	<div class="specification-section">
		<h3>Camera: <a href="https://www.stereolabs.com/products/zed-2">Zed 2</a></h3>
		<ul>
			<li><strong>Resolution:</strong> [Specify resolution]</li>
			<li><strong>FPS:</strong> [Specify frames per second]</li>
		</ul>
	</div>

	<div class="specification-section">
		<h3>Audio</h3>
		<p>None</p>
	</div>

	<div class="specification-section">
		<h3>GPS</h3>
		<p>To be filled out later</p>
	</div>

	<div class="specification-section">
		<h3>Battery</h3>
		<p>20Ah</p>
	</div>
</div>

<style>
	.specifications-container {
		display: flex;
		flex-direction: column;
		gap: 20px;
		padding: 20px;
	}
	.specification-section ul {
		padding-left: 15px;
	}
	@media (min-width: 800px) {
		.specifications-container {
			margin: 20px auto;
			width: 800px;
		}
	}
</style>
