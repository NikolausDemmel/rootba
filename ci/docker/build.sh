#!/usr/bin/env bash

REPO=registry.gitlab.vision.in.tum.de/rootba/rootba
#REPO=ghcr.io/nikolausdemmel/rootba

if [[ $(uname -m) == 'x86_64' ]]; then
    BUILD_CMD="docker build"
else
    # make sure to build for x86 on ARM and other platforms
    BUILD_CMD="docker buildx build --platform=linux/amd64"
fi

$BUILD_CMD --pull -f Dockerfile_20.04 -t $REPO/ubuntu-ci-rootba:20.04 .
$BUILD_CMD --pull -f Dockerfile_22.04 -t $REPO/ubuntu-ci-rootba:22.04 .

#docker push $REPO/ubuntu-ci-rootba:20.04
#docker push $REPO/ubuntu-ci-rootba:22.04

#docker tag $REPO/ubuntu-ci-rootba:22.04 $REPO/ubuntu-ci-rootba:latest
#docker push $REPO/ubuntu-ci-rootba:latest
