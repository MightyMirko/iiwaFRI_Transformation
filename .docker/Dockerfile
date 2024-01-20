# Use the official image as a base image
# Stage 1: Build Stage
FROM ubuntu:22.04 AS prepare
LABEL image_name="mmatosin/base_image"
ENV HOME /root
SHELL ["/bin/bash", "-c"]

# Install required dependencies
WORKDIR /app
COPY . .

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y \
    curl \
    software-properties-common \
    git \
    wget \
    gcc \
    gdb\
    g++\
    libc6-dev \
    build-essential \
    cmake \
    ninja-build \
    libprotobuf-dev \
    protobuf-compiler \
    doxygen \
    libboost-all-dev \
    libbullet-dev \
    libeigen3-dev \
    libnlopt-dev \
    libode-dev \
    libsimage-dev \
    libsoqt-dev \
    libxml2-dev \
    libxslt1-dev \
    libdc1394-dev \
    ssh \
    clang \
    tar\
    rsync

FROM mmatosin/mybase AS base
LABEL image_name="mmatosin/rl-app"
ENV DEBIAN_FRONTEND noninteractive

WORKDIR /app
# Build and install the Robotics Library dependencies
RUN add-apt-repository ppa:roblib/ppa &&\
    apt-get update && \
    apt-get install -y libsolid3d-dev

#ADD --link https://github.com/roboticslibrary/rl.git rl

WORKDIR /opt
RUN git clone https://github.com/roboticslibrary/rl.git rl
WORKDIR /opt/rl
RUN rm -rf /opt/rl/.git

ARG RL_USE_QT6=ON

RUN mkdir build
RUN cmake -S . -B build -D CMAKE_BUILD_TYPE=Debug -D RL_USE_QT6=${RL_USE_QT6}
RUN cmake --build build --parallel 12
RUN cmake --install build

RUN ldconfig

FROM mmatosin/rl-app AS roblib

LABEL org.zimmer.image.authors="mirko"
LABEL image_name="mmatosin/masters-app"
LABEL org.image.authors="@mightymirko"
LABEL "com.example.vendor"="AndAction"
LABEL version="1.0"
LABEL description="This text illustrates \
    that label-values can span multiple lines."
ENV DEBIAN_FRONTEND noninteractive

# Install Python3 and Doxygen
RUN apt-get update && \
    apt-get install -y python3-dev python3 python3-pip doxygen graphviz \
    libssl-dev libffi-dev

RUN useradd -m user && yes password | passwd user
# Set ownership for directories
RUN chown user:user -R /app /home/user

# Switch to the user context
ENV HOME /home/user/
USER user

# Install matplotlib
RUN python3 -m pip install matplotlib
ENV PATH="/home/user/.local/bin:${PATH}"
# Set the working directory for the final build
WORKDIR /app

# Copy your project files into the final image
COPY . .

# Build your project
RUN mkdir build && \
    cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MAKE_PROGRAM=ninja -DBUILD_FRI_APPS:BOOL=OFF -G \
    Ninja -S . -B build && \
    cmake --build build --parallel 12

# Set the working directory for the final application
WORKDIR /app/masters
HEALTHCHECK --interval=5m --timeout=3s \
    CMD curl -f http://localhost/ || exit 1

# Uncomment and adjust the following line as needed
CMD ../build/masters/mastersApp "172.31.0.147"
