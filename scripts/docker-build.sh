#!/bin/sh
docker rm --force noeticslam
docker build -t noeticslam:latest $(dirname "$0")/..