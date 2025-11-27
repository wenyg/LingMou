
```bash
export PUPPETEER_SKIP_CHROMIUM_DOWNLOAD=true
export PUPPETEER_SKIP_DOWNLOAD=true

## 首次构建（只需执行一次）

# 1. build xviz
cd /workspace/frontend/xviz
yarn bootstrap

# 2. build streetscape.gl
cd /workspace/frontend/streetscape.gl
yarn bootstrap

# 链接本地 xviz
cd /workspace/frontend
./setup-link.sh

# 编译 website-demo
/workspace/frontend/streetscape.gl/examples/website-demo
yarn
yarn start-streaming-local
```
