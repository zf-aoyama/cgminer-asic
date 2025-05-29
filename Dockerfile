FROM ubuntu:22.04

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential autoconf automake libtool pkg-config \
    libcurl4-openssl-dev libudev-dev libusb-1.0-0-dev \
    libncurses5-dev zlib1g-dev git && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

# bash + pipefail を常時有効にして CFLAGS に -fcommon を追加
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV CFLAGS="-O2 -fcommon"

RUN ./autogen.sh && \
    ./configure --enable-bitaxe CFLAGS="$CFLAGS" && \
    make -j"$(nproc)" 2>&1 | tee /tmp/build.log

CMD ["bash"]

