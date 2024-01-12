#!/bin/bash

docker buildx build -f="Dockerfile.BaseImage"  -t mmatosin/mybase ..
docker buildx build -f="Dockerfile.rl" -t mmatosin/rl-app ..
docker buildx build -f="Dockerfile.mastersApp" -t mmatosin/masters-app ..