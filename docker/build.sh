#!/bin/sh

## config
set -x # show command

## build
docker build --tag micromouse-maze-library $(dirname $0)
