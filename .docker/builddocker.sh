#!/bin/bash

docker buildx build -f="Dockerfile.BaseImage"  -t localhost/mybase ..
docker buildx build -f="Dockerfile.roblib" -t localhost/rl-app ..
docker buildx build -f="Dockerfile.mastersAppRun" -t localhost/mastersapp-run ..
docker buildx build -f="Dockerfile.mastersAppDev" -t localhost/mastersapp-dev ..
