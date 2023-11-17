# UMD

## Dependencies

Required Ubuntu dependencies:

```
sudo apt install -y \
libyaml-cpp-dev \
libhwloc-dev \
libgtest-dev \
libboost-dev
```

Ensure submodules are set:
```
git submodule update
```
## Build

This target builds `libdevice.so`. Specify the `ARCH_NAME` environment variable when building ( [See Device Selection](#device-selection) ):

```
make build ARCH_NAME=<arch goes here>
```

## Test

Run this target to build library, and gtest suite.

```
make test
```

Running test suite:

```
make run
```

## Clean

```
make clean
```

## Device Selection

Change the `ARCH_NAME` flag in the top-level Makefile or run:

Valid options:
* wormhole_b0
* grayskull

<br/>

```
make build ARCH_NAME=...
make test ARCH_NAME=...
```
