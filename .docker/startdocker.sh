#!/bin/bash

docker build -f=DockerfileBaseImage -t mmatosin/mybase ..
docker build -f=Dockerfile_rl -t mmatosin/roblib_app ..
docker build -f=Dockerfile_mastersApp -t mmatosin/masters_app ..
# Run the Docker container in privileged mode with host network
cd ..
docker run -it --rm --privileged --network=host mmatosin/masters_app /bin/bash
