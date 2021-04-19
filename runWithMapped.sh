#!/bin/bash

docker run -it \
    --network srcp2net \
    -v /SC2:$(pwd) \
    swarmathon:development \
    /bin/bash
