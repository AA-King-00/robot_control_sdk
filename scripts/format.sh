#!/bin/bash
echo "==> Running clang-format..."

# 查找 src, include, examples, tests 目录下的所有 c++ 文件并格式化
find include src examples tests -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i -style-file
echo "==> Formatting complete!"