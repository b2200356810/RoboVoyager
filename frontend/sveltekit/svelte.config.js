import adapter from '@sveltejs/adapter-static';

export default {
	kit: {
		adapter: adapter({
			pages: 'build',
			assets: 'build/assets',
			fallback: undefined,
			precompress: false,
			strict: true
		})
	}
};