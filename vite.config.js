import { defineConfig } from 'vite';

export default defineConfig({
  server: {
    port: 5173,
    open: false
  },
  build: {
    target: 'esnext',
    outDir: 'dist'
  },
  test: {
    environment: 'node',
    globals: true,
    include: ['tests/**/*.test.ts']
  }
});
