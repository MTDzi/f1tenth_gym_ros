#!/bin/bash
if [ ! -d f1tenth_gym ] ; then
    git clone -b v0.0.1 https://github.com/MTDzi/f1tenth_gym
else
    echo f1tenth_gym exists, not cloning.
fi
docker build -t mtdzi/f1tenth_gym:v0.0.8 .
