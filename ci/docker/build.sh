#!/usr/bin/env bash

#REPO=registry.gitlab.vision.in.tum.de/rootba/rootba
REPO=ghcr.io/nikolausdemmel/rootba

docker build --pull -f Dockerfile_18.04 -t $REPO/ubuntu-ci-rootba:18.04 .
docker build --pull -f Dockerfile_20.04 -t $REPO/ubuntu-ci-rootba:20.04 .

# docker push $REPO/ubuntu-ci-rootba:18.04
# docker push $REPO/ubuntu-ci-rootba:20.04
