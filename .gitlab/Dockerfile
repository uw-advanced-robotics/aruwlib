# Download GoogleTest
FROM ubuntu:24.04 as gtest

ARG GTEST_COMMIT=7d76a231b0e29caf86e68d1df858308cd53b2a66

RUN apt-get update -qq \
    && DEBIAN_FRONTEND=noninteractive apt-get install -qq --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN URL="https://github.com/google/googletest/archive/${GTEST_COMMIT}.tar.gz" \
    && mkdir -p gtest/build \
    && wget -qO- "$URL" | tar xz --strip-components=1 -C gtest \
    && cd gtest/build \
    && cmake ../ \
    && make -j

# Download ARM GCC
FROM ubuntu:24.04 as gcc-arm

ARG ARM_GCC_VERSION=13.3.rel1

RUN apt-get update -qq \
    && DEBIAN_FRONTEND=noninteractive apt-get install -qq --no-install-recommends \
    ca-certificates \
    wget \
    xz-utils \
    && rm -rf /var/lib/apt/lists/*

RUN ARCH=$(uname -m) \
    URL="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_GCC_VERSION}/binrel/arm-gnu-toolchain-${ARM_GCC_VERSION}-${ARCH}-arm-none-eabi.tar.xz" \
    && mkdir gcc-arm \
    && wget -qO- "$URL" | tar xJ --strip-components=1 -C gcc-arm

# Download run-clang-format.py
FROM ubuntu:24.04 AS run-clang-format

RUN apt-get update -qq \
    && DEBIAN_FRONTEND=noninteractive apt-get install -qq --no-install-recommends \
    ca-certificates \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN wget -O /run-clang-format.py https://raw.githubusercontent.com/Sarcasm/run-clang-format/50a5250a2666218595ee21d68c71f50ac39866d2/run-clang-format.py

# Main stage
FROM ubuntu:24.04

COPY --from=gtest /gtest/googletest/include/ /usr/local/include/
COPY --from=gtest /gtest/googlemock/include/ /usr/local/include/
COPY --from=gtest /gtest/build/lib/ /usr/local/lib/

COPY --from=gcc-arm /gcc-arm /gcc-arm
ENV PATH="/gcc-arm/bin:$PATH"

COPY --from=run-clang-format /run-clang-format.py /

RUN apt-get update -qq \
    && DEBIAN_FRONTEND=noninteractive apt-get install -qq --no-install-recommends \
    clang-format \
    g++ \
    gcovr \
    git \
    lcov \
    libcpanel-json-xs-perl \
    locales \
    locales-all \
    python3-pip \
    valgrind \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages --root-user-action=ignore \
    lbuild \
    pyelftools \
    scons