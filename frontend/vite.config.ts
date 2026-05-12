import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  build: {
    // Output directly into backend/static so FastAPI serves the built SPA
    outDir: '../backend/static',
    emptyOutDir: true,
  },
  server: {
    port: 5173,
    proxy: {
      // Forward /api requests to the FastAPI backend during development
      '/api': {
        target: 'http://localhost:8080',
        changeOrigin: true,
      },
    },
  },
})
