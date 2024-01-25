#!/bin/bash

docker buildx build -f="Dockerfile.BaseImage"  -t mmatosin/mybase ..
docker buildx build -f="Dockerfile.roblib" -t mmatosin/rl-app ..
docker buildx build -f="Dockerfile.mastersAppRun" -t mmatosin/mastersapp-run ..
docker buildx build -f="Dockerfile.mastersAppDev" -t mmatosin/mastersapp-dev ..