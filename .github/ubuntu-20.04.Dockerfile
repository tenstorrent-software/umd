FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Install build and runtime deps
RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    cmake \
    ninja-build \
    git \
    libhwloc-dev \
    libgtest-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    wget

# Install clang 17
RUN wget https://apt.llvm.org/llvm.sh && \
    chmod u+x llvm.sh && \
    ./llvm.sh 17 && \
    apt install -y libc++-17-dev libc++abi-17-dev && \
    ln -s /usr/bin/clang-17 /usr/bin/clang && \
    ln -s /usr/bin/clang++-17 /usr/bin/clang++

# Install clang-format
RUN apt install -y clang-format-17 && \
    ln -s /usr/bin/clang-format-17 /usr/bin/clang-format
