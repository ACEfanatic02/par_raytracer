#!/usr/bin/bash

mkdir -p build
g++ -g -O2 --std=c++11 -fno-strict-aliasing -m64 -Wall -o ./build/raytracer ./main.cpp
