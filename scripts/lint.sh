#!/bin/bash
echo "==> Running clang-tidy..."
# 依赖 build 目录下的 compile_commands.json
find include src examples tests -name "*.cpp" -o -name "*.hpp" | xargs clang-tidy -p build/
echo "==> Linting complete!"