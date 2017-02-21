#!/usr/bin/bash

mkdir -p build
g++ -g -O2 -fno-strict-aliasing -m64 -Wall -o ./build/raytracer ./raytracer.cpp
