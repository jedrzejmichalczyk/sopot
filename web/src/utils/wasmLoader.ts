import type { SopotModule } from '../types/sopot';

/**
 * Load the SOPOT WebAssembly module with proper path resolution
 *
 * This function handles the dynamic loading of the WASM module via blob URL
 * to bypass Vite's transform pipeline, and provides the necessary locateFile
 * callback to ensure the .wasm file is loaded from the correct location.
 *
 * @returns Promise resolving to the SOPOT module instance
 * @throws Error if the module fails to load
 */
export async function loadSopotWasmModule(): Promise<SopotModule> {
  // Use base URL to handle GitHub Pages deployment path
  const basePath = import.meta.env.BASE_URL || '/';
  const moduleUrl = `${basePath}sopot.js`;

  console.log('[WASM] Loading module from:', moduleUrl);

  // Fetch the module and create a blob URL to bypass Vite's transform
  const response = await fetch(moduleUrl);
  if (!response.ok) {
    throw new Error(`Failed to fetch ${moduleUrl}: ${response.status} ${response.statusText}`);
  }

  const moduleText = await response.text();
  const blob = new Blob([moduleText], { type: 'application/javascript' });
  const blobUrl = URL.createObjectURL(blob);

  try {
    // @ts-ignore - Dynamic import of WebAssembly module
    const createSopotModule = await import(/* @vite-ignore */ blobUrl);

    // Instantiate the WASM module with locateFile to resolve .wasm path
    const module = await createSopotModule.default({
      locateFile: (path: string) => {
        // WASM file should be loaded from the public directory
        if (path.endsWith('.wasm')) {
          return `${basePath}${path}`;
        }
        return path;
      }
    });

    console.log('[WASM] Module loaded successfully');
    return module;
  } finally {
    // Clean up the blob URL
    URL.revokeObjectURL(blobUrl);
  }
}

/**
 * Check if a specific simulator is available in the WASM module
 *
 * @param module - The SOPOT WASM module
 * @param simulatorName - Name of the simulator to check
 * @returns true if the simulator is available
 */
export function hasSimulator(
  module: SopotModule,
  simulatorName: 'RocketSimulator' | 'Grid2DSimulator' | 'InvertedPendulumSimulator'
): boolean {
  return typeof module[simulatorName] === 'function';
}
