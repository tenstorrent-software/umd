# UMD
## About
Usermode Driver for Tenstorrent AI Accelerators

## Dependencies
Required Ubuntu dependencies:
```
sudo apt install -y libhwloc-dev cmake ninja-build
```

Suggested third-party dependency is Clang 17:
```
wget https://apt.llvm.org/llvm.sh
chmod u+x llvm.sh
sudo ./llvm.sh 17
```

## Build flow

To build `libdevice.so`: 
```
cmake -B build -G Ninja
ninja -C build
# or
ninja umd_device -C build
```

Tests are build separatelly for each architecture.
Specify the `ARCH_NAME` environment variable as `grayskull`,  `wormhole_b0` or `blackhole` before building.
You also need to configure cmake to enable tests, hence the need to run cmake configuration step again.
To build tests:
```
cmake -B build -G Ninja -DTT_UMD_BUILD_TESTS=ON
ninja umd_tests -C build
```

To build with GCC, set these environment variables before invoking `cmake`:
```
export CMAKE_C_COMPILER=/usr/bin/gcc
export CMAKE_CXX_COMPILER=/usr/bin/g++
```

## As a submodule/external project
If your project has CMake support, simply add this repo as a subdirectory:
```
add_subdirectory(<path to umd>)
```
You can then use `libdevice.so` by linking against the `umd_device` target wheverever is needed.
```
target_link_libraries(tt_metal PUBLIC umd_device)
```

# Formatting C++ code with clang-format

## Installing clang-format

If you're using an IRD docker, clang-format 19 should be already available.
If you don't have clang-format in your working environment, follow the instructions
on [llvm website](https://apt.llvm.org/) for installing it.

## Formatting files

If working with VSCode, you can copy the provided default settings:
```bash
cp .vscode/default.settings.json .vscode/settings.json
```

From now on, c++ files will be formatted on save (given that clang-format is available).

If you want to format the whole repo, you can use this script:
```bash
.github/clang-format-repo.sh
```

## Git pre-commit hook

TBD
