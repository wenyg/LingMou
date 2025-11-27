#!/bin/bash

echo "=== 设置 XVIZ 本地链接 ==="

# 1. 注册 xviz 模块到 yarn link
echo "注册 XVIZ 模块..."
for D in /workspace/frontend/xviz/modules/*; do
  if [ -d "$D" ]; then
    echo "  - $(basename $D)"
    (cd "$D" && yarn link > /dev/null 2>&1)
  fi
done

# 2. 在 streetscape.gl 中链接本地 xviz
echo "链接 XVIZ 到 streetscape.gl..."
cd /workspace/frontend/streetscape.gl
./scripts/dev-link-dependencies.sh link > /dev/null 2>&1

echo "✓ 完成！验证链接："
ls -l /workspace/frontend/streetscape.gl/modules/core/node_modules/@xviz/parser 2>/dev/null | grep -q "^l" && \
  echo "  ✓ @xviz/parser 已链接到本地" || \
  echo "  ✗ @xviz/parser 链接失败"

