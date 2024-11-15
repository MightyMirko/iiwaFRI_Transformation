#!/bin/bash


# Run the Docker container in privileged mode with host network
cd ..
docker run -it --rm --privileged --network=host localhost/masters_app /bin/bash
