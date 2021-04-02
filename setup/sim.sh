#!/bin/bash

mkdir simulator/build
cd simulator
git submodule update --init --recursive
cd build
cmake ..
make
cd ../..
