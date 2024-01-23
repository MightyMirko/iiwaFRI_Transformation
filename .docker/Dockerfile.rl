# syntax=docker/dockerfile:1.5-labs
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


