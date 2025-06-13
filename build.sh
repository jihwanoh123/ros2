#!/bin/bash

# Build the matrix_publisher package with all necessary CMake arguments
colcon build --packages-select matrix_publisher --cmake-args \
  -DPYTHON_EXECUTABLE=$PYTHON_EXECUTABLE \
  -DPython3_EXECUTABLE=$Python3_EXECUTABLE \
  -DPython3_INCLUDE_DIR=$Python3_INCLUDE_DIR \
  -DPython3_LIBRARY=$Python3_LIBRARY \
  -DPython3_NumPy_INCLUDE_DIRS=$Python3_NumPy_INCLUDE_DIRS \
  -DCMAKE_C_COMPILER=/usr/bin/clang \
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++

# Source the setup file if build was successful
if [ $? -eq 0 ]; then
    source install/setup.zsh
    echo "Build successful! Environment sourced."
else
    echo "Build failed!"
fi 