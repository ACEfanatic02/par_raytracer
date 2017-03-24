#!/usr/bin/bash

module load mpi
mkdir -p build
#g++ -g -O2 --std=c++11 -fno-strict-aliasing -m64 -Wall -Wno-unused-variable -Wno-unused-function -Wno-write-strings -o ./build/raytracer ./main.cpp
mpic++ -g -O2 --std=c++11 -fno-strict-aliasing -m64 -D_MPI=1 -Wall -Wno-unused-variable -Wno-unused-function -Wno-write-strings -o ./build/raytracer ./main.cpp
