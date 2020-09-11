#!/bin/bash

(cd ..; docker build -t swarmathon:submission -f docker/Dockerfile .) && \
../srcp2-competitors/docker/scripts/qual_submission/build-submission-image.bash -w /SC2 -i swarmathon:submission -t swarmathon \
  -p scoot \
  -1 qual_round_1.launch \
  -2 qual_round_2.launch \
  -3 qual_round_3.launch
