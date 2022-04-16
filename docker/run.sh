#!/bin/sh

SRC_DIR=$(readlink -f $(dirname $0)/..)
BUILD_DIR=$SRC_DIR/build-docker
mkdir -p $BUILD_DIR

docker run --rm --interactive --tty \
    --volume $SRC_DIR:/src \
    --workdir /src/$(basename $BUILD_DIR) \
    --volume /etc/group:/etc/group:ro \
    --volume /etc/passwd:/etc/passwd:ro \
    --volume /etc/shadow:/etc/shadow:ro \
    --user $(id -u $USER):$(id -g $USER) \
    --env DISPLAY \
    --volume $HOME/.Xauthority:$HOME/.Xauthority:ro \
    --network host \
    micromouse-maze-library \
    "$@" # pass arguments as they are
