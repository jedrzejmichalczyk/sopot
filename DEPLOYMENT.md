# SOPOT WebAssembly - GitHub Pages Deployment Guide

This guide explains how to deploy the SOPOT interactive rocket simulation to GitHub Pages.

## 🌐 Live Demo

Once deployed, the application will be available at:
**https://jedrzejmichalczyk.github.io/sopot/**

## 🚀 Automatic Deployment (Recommended)

The repository is configured with GitHub Actions for automatic deployment.

### Setup (One-Time)

1. **Enable GitHub Pages in repository settings:**
   - Go to your repository on GitHub
   - Navigate to **Settings** → **Pages**
   - Under "Build and deployment":
     - **Source**: Select "GitHub Actions"
   - Click **Save**

2. **Trigger deployment:**
   - Push changes to `main` or `master` branch
   - Or manually trigger via **Actions** tab → **Deploy SOPOT Web to GitHub Pages** → **Run workflow**

### What Happens Automatically

The GitHub Actions workflow (`.github/workflows/deploy-github-pages.yml`) will:

1. ✅ Check out the repository
2. ✅ Set up Node.js 20
3. ✅ Set up Emscripten (latest version)
4. ✅ Build the WebAssembly module (`wasm/sopot.js` + `sopot.wasm`)
5. ✅ Install web application dependencies
6. ✅ Copy WebAssembly files to `web/public/`
7. ✅ Build the React application for production
8. ✅ Deploy to GitHub Pages

**Build time:** ~5-10 minutes

### Monitoring Deployment

1. Go to **Actions** tab in your repository
2. Click on the latest workflow run
3. Monitor the build and deploy steps
4. Once complete, visit the live URL

## 🛠️ Manual Deployment (Local Build)

If you prefer to build locally and deploy manually:

### Prerequisites

- Node.js 18+
- Emscripten SDK installed and activated
- Git

### Steps

```bash
# 1. Navigate to repository root
cd /path/to/sopot

# 2. Build WebAssembly module
cd wasm
./build.sh Release
cd ..

# 3. Navigate to web directory
cd web

# 4. Install dependencies (if not already done)
npm install

# 5. Copy WebAssembly files
cp ../wasm/sopot.js ../wasm/sopot.wasm public/

# 6. Build for GitHub Pages
npm run build:github

# 7. The dist/ directory now contains the production build
# You can deploy this manually using gh-pages npm package or other tools
```

### Alternative: Using gh-pages Package

```bash
# Install gh-pages
npm install -D gh-pages

# Add to package.json scripts:
# "deploy:manual": "gh-pages -d dist"

# Deploy
npm run deploy:manual
```

## 🔧 Configuration

### Base Path

The application is configured to work with the `/sopot/` base path for GitHub Pages.

This is set via the `VITE_BASE_PATH` environment variable:

```bash
# In vite.config.ts:
base: process.env.VITE_BASE_PATH || '/',

# For GitHub Pages deployment:
VITE_BASE_PATH=/sopot/ npm run build
```

### Custom Domain

If you want to use a custom domain:

1. Add a `CNAME` file to `web/public/` with your domain:
   ```
   sopot.yourdomain.com
   ```

2. Update `vite.config.ts`:
   ```typescript
   base: '/',  // Root for custom domain
   ```

3. Configure DNS:
   - Add a CNAME record pointing to `jedrzejmichalczyk.github.io`

4. Update GitHub Pages settings to use custom domain

## 📊 Build Output

After building, the `web/dist/` directory will contain:

```
dist/
├── index.html           # Main HTML file
├── assets/
│   ├── index-[hash].js  # React application bundle
│   ├── index-[hash].css # Styles
│   └── ...
├── sopot.js            # WebAssembly loader
└── sopot.wasm          # Compiled simulation (600-800 KB)
```

**Total size:** ~1.5-2 MB (including all assets)

## 🐛 Troubleshooting

### "Failed to load WebAssembly module"

**Issue:** WebAssembly files not found at runtime

**Solutions:**
1. Ensure `sopot.js` and `sopot.wasm` are in `web/public/` before building
2. Check that base path is set correctly in deployment
3. Verify files exist in the deployed `dist/` directory

### Build fails in GitHub Actions

**Issue:** Emscripten or Node.js build errors

**Solutions:**
1. Check the Actions log for specific error messages
2. Verify Emscripten version is compatible (3.1.0+)
3. Ensure all dependencies in `package.json` are correct
4. Try building locally first to isolate the issue

### "404 Not Found" on GitHub Pages

**Issue:** Page doesn't load after deployment

**Solutions:**
1. Verify GitHub Pages is enabled in repository settings
2. Check that the source is set to "GitHub Actions"
3. Wait 2-3 minutes after deployment for DNS propagation
4. Try accessing with `/index.html` explicitly

### Base path issues (assets not loading)

**Issue:** Page loads but assets 404

**Solutions:**
1. Verify `VITE_BASE_PATH=/sopot/` is set during build
2. Check `vite.config.ts` has correct base configuration
3. Inspect browser console for asset URL patterns
4. Ensure assets are in the `dist/assets/` directory

## 📝 Workflow Triggers

The GitHub Actions workflow triggers on:

1. **Push to main/master branch** with changes in:
   - `web/**` (web application)
   - `wasm/**` (WebAssembly module)
   - `.github/workflows/deploy-github-pages.yml` (workflow itself)

2. **Manual trigger** via GitHub Actions UI

To deploy without pushing, use the manual trigger:
- GitHub repository → **Actions** tab
- Select "Deploy SOPOT Web to GitHub Pages"
- Click **Run workflow** → Select branch → **Run workflow**

## 🔒 Permissions

The workflow requires these permissions (already configured):
- `contents: read` - Read repository files
- `pages: write` - Deploy to GitHub Pages
- `id-token: write` - OIDC token for secure deployment

## 📈 Monitoring & Analytics

To track usage of your deployed application:

1. **GitHub Insights:**
   - Repository → **Insights** → **Traffic**
   - See page views and unique visitors

2. **Add Google Analytics** (optional):
   - Add tracking code to `web/index.html`
   - Monitor user interactions

3. **Error Tracking** (optional):
   - Integrate Sentry or similar service
   - Track runtime errors in production

## 🎯 Next Steps After Deployment

1. **Test the live application:**
   - Visit https://jedrzejmichalczyk.github.io/sopot/
   - Test all features
   - Try on mobile devices
   - Check browser console for errors

2. **Share the link:**
   - Add to README.md
   - Share on social media
   - Include in research papers/presentations

3. **Monitor performance:**
   - Check PageSpeed Insights
   - Test on different browsers
   - Gather user feedback

4. **Iterate:**
   - Fix any issues found
   - Add new features
   - Push updates (auto-deploys!)

## 📚 Resources

- [GitHub Pages Documentation](https://docs.github.com/en/pages)
- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Vite Build Guide](https://vitejs.dev/guide/build.html)
- [Emscripten Setup](https://emscripten.org/docs/getting_started/downloads.html)

---

**Deployment Status:** ⏳ Waiting for first deployment

Once deployed, update this file with the live URL and any production-specific notes!
