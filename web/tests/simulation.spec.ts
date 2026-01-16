import { test, expect } from '@playwright/test';

test.describe('SOPOT Web UI', () => {
  test('page loads successfully', async ({ page }) => {
    await page.goto('/');
    await expect(page).toHaveTitle(/SOPOT/i);
  });

  test('simulation selector is visible', async ({ page }) => {
    await page.goto('/');

    // Wait for the app to load
    await expect(page.getByText('Simulation Type')).toBeVisible();

    // Both simulation options should be visible
    await expect(page.getByText('Rocket Flight')).toBeVisible();
    await expect(page.getByText('2D Mass-Spring Grid')).toBeVisible();
  });

  test('can switch to Grid2D simulation', async ({ page }) => {
    await page.goto('/');

    // Click on the Grid2D simulation button
    await page.getByText('2D Mass-Spring Grid').click();

    // Grid2D specific controls should appear
    await expect(page.getByText('GRID2D')).toBeVisible();
  });

  test('can switch back to Rocket simulation', async ({ page }) => {
    await page.goto('/');

    // First switch to Grid2D
    await page.getByText('2D Mass-Spring Grid').click();

    // Then switch back to Rocket
    await page.getByText('Rocket Flight').click();

    // Rocket specific elements should be visible
    await expect(page.getByText('RKTFLT')).toBeVisible();
  });

  test('WASM module loads', async ({ page }) => {
    await page.goto('/');

    // Wait for WASM to load - the initialize button should be visible
    await expect(page.getByRole('button', { name: 'Initialize Simulation' })).toBeVisible({
      timeout: 10000,
    });
  });
});
