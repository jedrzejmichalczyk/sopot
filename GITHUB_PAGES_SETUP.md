# 🚀 Enable GitHub Pages - Quick Setup Guide

Follow these steps to deploy the SOPOT interactive demo to GitHub Pages.

## Step 1: Merge to Main Branch

The GitHub Actions workflow triggers on the `main` or `master` branch.

```bash
# Option A: Merge your branch via GitHub PR (Recommended)
# Go to GitHub and create/merge your PR

# Option B: Merge locally (if you have permissions)
git checkout main
git merge claude/webassembly-simulation-ui-C9A7o
git push origin main
```

## Step 2: Enable GitHub Pages

1. **Go to your repository on GitHub:**
   ```
   https://github.com/jedrzejmichalczyk/sopot
   ```

2. **Click on "Settings"** (top navigation bar)

3. **Scroll to "Pages"** in the left sidebar

4. **Under "Build and deployment":**
   - **Source:** Select **"GitHub Actions"** from the dropdown
   - (NOT "Deploy from a branch" - we want GitHub Actions)

5. **Click "Save"** (if there's a save button)

That's it! GitHub Pages is now enabled.

## Step 3: Trigger First Deployment

### Automatic Trigger (After Merge)

If you merged to `main`, the workflow will automatically trigger.

### Manual Trigger (If Needed)

1. Go to the **"Actions"** tab in your repository

2. Click on **"Deploy SOPOT Web to GitHub Pages"** in the left sidebar

3. Click the **"Run workflow"** button (top right)

4. Select branch: **`main`** or **`master`**

5. Click **"Run workflow"** (green button)

## Step 4: Monitor Deployment

1. **Go to "Actions" tab**

2. **Click on the running workflow** (you'll see a yellow indicator)

3. **Watch the build steps:**
   - ⏳ Build job (5-8 minutes)
     - Checkout code
     - Set up Node.js
     - Set up Emscripten
     - Build WebAssembly
     - Build React app
   - ⏳ Deploy job (1-2 minutes)
     - Deploy to GitHub Pages

4. **Wait for green checkmark** ✅

## Step 5: Access Your Live Demo! 🎉

Once deployment is complete (green checkmark):

**Live URL:** **https://jedrzejmichalczyk.github.io/sopot/**

**First load may take 2-3 minutes** for DNS propagation.

## 🎯 What You'll See

When you visit the URL, you'll see:

- **Left Panel:** Configuration controls (elevation, azimuth, diameter, timestep)
- **Center Panel:** Beautiful 3D rocket visualization with real-time physics
- **Right Panel:** Live telemetry (altitude, speed, mass, vectors)

**Try this:**
1. Click "Initialize Simulation"
2. Click "▶ Start"
3. Watch the rocket launch in 3D!
4. Rotate the camera (left-click + drag)
5. Zoom in/out (scroll wheel)

## 📊 Verification Checklist

✅ GitHub Pages enabled in Settings → Pages → Source: "GitHub Actions"
✅ Workflow ran successfully (green checkmark in Actions tab)
✅ URL accessible: https://jedrzejmichalczyk.github.io/sopot/
✅ WebAssembly module loads (check browser console for errors)
✅ 3D scene renders correctly
✅ Simulation runs when you click Start

## 🐛 Troubleshooting

### "404 Not Found"

**Wait 2-3 minutes** after deployment for GitHub's CDN to update.

If still not working:
- Verify GitHub Pages is enabled (Settings → Pages)
- Check that Source is "GitHub Actions" (not "Deploy from a branch")
- Try accessing with `/index.html` explicitly

### Build Fails in Actions

Check the Actions log for errors:
- Common: Emscripten version incompatibility → workflow will retry
- Common: npm package installation → check package.json
- Common: TypeScript errors → check web/src/ for type issues

### Page Loads but Shows Error

Open browser console (F12) and check for:
- "Failed to load WebAssembly module" → Workflow didn't copy WASM files
- "404 on /sopot.js" → Base path misconfiguration
- Three.js errors → Check if WebGL is enabled in browser

## 📝 After First Deployment

### Share Your Demo!

Update your project README, papers, presentations with:
```
🚀 Live Demo: https://jedrzejmichalczyk.github.io/sopot/
```

### Automatic Updates

Every time you push changes to `main` branch that affect `web/` or `wasm/`, the site will automatically rebuild and redeploy!

### Monitor Usage

Go to: **Repository → Insights → Traffic** to see page views.

## 🎊 Success!

If you can see the rocket simulation running in 3D in your browser, **congratulations!**

You've successfully deployed:
- C++20 compile-time physics simulation
- Compiled to WebAssembly
- Integrated with React + Three.js
- Running at 60 FPS in the browser
- With beautiful 3D visualization

**Share it with the world!** 🌍🚀

---

**Questions or Issues?**

Check the detailed [DEPLOYMENT.md](DEPLOYMENT.md) guide or open an issue on GitHub.
