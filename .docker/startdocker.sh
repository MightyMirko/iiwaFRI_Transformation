#!/bin/bash

# Run the Docker container in privileged mode with host network
docker run -it --rm --privileged --network=host mmatosin/masters_app /bin/bash