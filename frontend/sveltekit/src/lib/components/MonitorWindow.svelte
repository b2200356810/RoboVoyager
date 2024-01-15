<script>
	export let monitorName;
	export let monitorContent;
</script>

<div class="monitor-window">
	<div class="monitor-header">
		<div class="monitor-name">{monitorName}</div>
		<!-- <button class="pause">| |</button> -->
		<button class="pause">
			<svg xmlns="http://www.w3.org/2000/svg" height="16" width="10" viewBox="0 0 320 512"
				><path
					d="M48 64C21.5 64 0 85.5 0 112V400c0 26.5 21.5 48 48 48H80c26.5 0 48-21.5 48-48V112c0-26.5-21.5-48-48-48H48zm192 0c-26.5 0-48 21.5-48 48V400c0 26.5 21.5 48 48 48h32c26.5 0 48-21.5 48-48V112c0-26.5-21.5-48-48-48H240z"
				/></svg
			>
		</button>

		<!-- <button class="close">X</button> -->
		<button class="close"
			><svg
				xmlns="http://www.w3.org/2000/svg"
				width="16"
				height="16"
				fill="currentColor"
				class="bi bi-x-lg"
				viewBox="0 0 16 16"
			>
				<path
					d="M2.146 2.854a.5.5 0 1 1 .708-.708L8 7.293l5.146-5.147a.5.5 0 0 1 .708.708L8.707 8l5.147 5.146a.5.5 0 0 1-.708.708L8 8.707l-5.146 5.147a.5.5 0 0 1-.708-.708L7.293 8z"
				/>
			</svg></button
		>
	</div>
	<!-- <div class="monitor-body">{monitorContent}</div> -->
	{#if monitorName === '/terminal_topic'}
		<div class="monitor-body">
			{monitorContent}
		</div>
	{/if}

	{#if monitorName === '/video_streaming_topic'}
		<div class="monitor-body">
			<img src="data:image/jpeg;base64,{monitorContent}" alt="" />
		</div>
	{/if}
	<div class="monitor-footer">foot</div>
</div>

<style>
	:root {
		--monitor-background: gray;
		--monitor-text-color: white;
	}

	:global(.dark-theme) {
		--monitor-background: #2e2e2e;
		/* --monitor-text-color: white; */
	}

	.monitor-window {
		display: flex;
		flex-direction: column;
		gap: 10px;
		height: 400px;
		width: 200px;
		background-color: var(--monitor-background);
		color: var(--monitor-text-color);
		border-radius: 10px;
	}

	.monitor-body {
		/* height: 100%; */
		/* width: 100%; */
	}

	.monitor-body img {
		height: 100%;
		width: 100%;
	}

	.monitor-header {
		display: flex;
		gap: 3px;
		border: 1px solid black;
		align-items: center;
		padding: 10px;
		/* height: 30px; */
	}

	.monitor-name {
		margin-right: auto;
	}

	.monitor-header button {
		width: 20px;
		height: 20px;
		display: flex;
		align-items: center;
		justify-content: center;
		background-color: transparent;
		border: none;
	}

	.monitor-body {
		border: 1px solid black;
		flex: 1;
	}

	.monitor-footer {
		padding: 10px;
		border: 1px solid black;
	}
</style>
