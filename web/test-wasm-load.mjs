#!/usr/bin/env node

/**
 * Simple test to verify WASM module loads correctly
 */

import { createRequire } from 'module';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

async function testWasmLoading() {
    console.log('Testing WASM module loading...\n');

    try {
        // Import the WASM module
        const modulePath = join(__dirname, 'public', 'sopot.js');
        console.log(`Loading module from: ${modulePath}`);

        const { default: createSopotModule } = await import(modulePath);
        console.log('✓ Module file loaded');

        // Initialize the WASM module with locateFile to help it find the .wasm file
        console.log('Initializing WASM...');
        const Module = await createSopotModule({
            locateFile: (path) => {
                if (path.endsWith('.wasm')) {
                    return join(__dirname, 'public', path);
                }
                return path;
            }
        });
        console.log('✓ WASM initialized');

        // Check for available simulators
        console.log('\nChecking available simulators:');

        const hasRocket = typeof Module.RocketSimulator === 'function';
        console.log(`  RocketSimulator: ${hasRocket ? '✓ Available' : '✗ Missing'}`);

        const hasGrid2D = typeof Module.Grid2DSimulator === 'function';
        console.log(`  Grid2DSimulator: ${hasGrid2D ? '✓ Available' : '✗ Missing'}`);

        const hasPendulum = typeof Module.InvertedPendulumSimulator === 'function';
        console.log(`  InvertedPendulumSimulator: ${hasPendulum ? '✓ Available' : '✗ Missing'}`);

        if (!hasPendulum) {
            throw new Error('InvertedPendulumSimulator not found in module!');
        }

        // Try to instantiate the pendulum simulator
        console.log('\nTesting InvertedPendulumSimulator instantiation:');
        const pendulum = new Module.InvertedPendulumSimulator();
        console.log('✓ Simulator created');

        // Call setupDefault to initialize
        pendulum.setupDefault();
        console.log('✓ setupDefault() called');

        // Get initial state
        const t = pendulum.getTime();
        const x = pendulum.getCartPosition();
        const theta1 = pendulum.getTheta1();
        const theta2 = pendulum.getTheta2();

        console.log(`\nInitial state:`);
        console.log(`  Time: ${t.toFixed(3)}s`);
        console.log(`  Cart position: ${x.toFixed(3)}m`);
        console.log(`  Theta1: ${theta1.toFixed(3)}rad`);
        console.log(`  Theta2: ${theta2.toFixed(3)}rad`);

        // Try a simulation step
        pendulum.step();
        const t2 = pendulum.getTime();
        console.log(`\n✓ Simulation step successful (t = ${t2.toFixed(3)}s)`);

        // Cleanup
        pendulum.delete();
        console.log('✓ Cleanup successful');

        console.log('\n========================================');
        console.log('SUCCESS: All tests passed!');
        console.log('========================================\n');

        return true;

    } catch (error) {
        console.error('\n========================================');
        console.error('ERROR: WASM loading failed!');
        console.error('========================================');
        console.error(error);
        console.error('');
        return false;
    }
}

// Run the test
testWasmLoading().then(success => {
    process.exit(success ? 0 : 1);
});
