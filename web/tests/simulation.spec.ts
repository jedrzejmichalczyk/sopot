import { test, expect } from '@playwright/test';

test.describe('SOPOT Web UI', () => {
  test('page loads successfully', async ({ page }) => {
    await page.goto('/');
    await expect(page).toHaveTitle(/SOPOT/i);
  });

  test('problem selector tabs are visible', async ({ page }) => {
    await page.goto('/');

    // The tab bar lists every available problem
    const tablist = page.getByRole('tablist', { name: 'Simulation problem' });
    await expect(tablist).toBeVisible();

    await expect(page.getByRole('tab', { name: /Rocket Flight/ })).toBeVisible();
    await expect(page.getByRole('tab', { name: /Mass-Spring Grid/ })).toBeVisible();
    await expect(page.getByRole('tab', { name: /Inverted Pendulum/ })).toBeVisible();
  });

  test('problem selector tabs are visible on mobile', async ({ page }) => {
    await page.setViewportSize({ width: 390, height: 844 });
    await page.goto('/');

    await expect(page.getByRole('tablist', { name: 'Simulation problem' })).toBeVisible();
    await expect(page.getByRole('tab', { name: /Rocket Flight/ })).toBeVisible();
  });

  test('can switch to Grid2D simulation', async ({ page }) => {
    await page.goto('/');

    // Click on the Grid2D tab
    await page.getByRole('tab', { name: /Mass-Spring Grid/ }).click();

    // Tab becomes selected and Grid2D specific info appears
    await expect(page.getByRole('tab', { name: /Mass-Spring Grid/ })).toHaveAttribute(
      'aria-selected',
      'true'
    );
    await expect(page.getByText('GRID2D').first()).toBeVisible();
  });

  test('can switch back to Rocket simulation', async ({ page }) => {
    await page.goto('/');

    // First switch to Grid2D
    await page.getByRole('tab', { name: /Mass-Spring Grid/ }).click();

    // Then switch back to Rocket
    await page.getByRole('tab', { name: /Rocket Flight/ }).click();

    await expect(page.getByRole('tab', { name: /Rocket Flight/ })).toHaveAttribute(
      'aria-selected',
      'true'
    );
    await expect(page.getByText('RKTFLT').first()).toBeVisible();
  });

  test('WASM module loads', async ({ page }) => {
    await page.goto('/');

    // Wait for WASM to load - the initialize button should be visible
    await expect(page.getByRole('button', { name: 'Initialize Simulation' })).toBeVisible({
      timeout: 10000,
    });
  });
});
