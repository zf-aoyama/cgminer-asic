FROM ubuntu:22.04

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential autoconf automake libtool pkg-config \
    libcurl4-openssl-dev libudev-dev libusb-1.0-0-dev \
    libncurses5-dev zlib1g-dev git && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

RUN ./autogen.sh && ./configure --enable-bitaxe && make -j"$(nproc)"

CMD ["bash"]
