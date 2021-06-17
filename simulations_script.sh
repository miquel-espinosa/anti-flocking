#!/bin/bash

python3 anti_flocking.py -m "unique" -n 3 -d SIMULATIONS/obstacles -f multiple-uniq-4 --noplot
python3 anti_flocking.py -m "unique" -n 3 -d SIMULATIONS/obstacles -f multiple-uniq-5 --noplot
python3 anti_flocking.py -m "unique" -n 3 -d SIMULATIONS/obstacles -f multiple-uniq-6 --noplot

python3 anti_flocking.py -m "continuous" -n 3 -d SIMULATIONS/obstacles -f multiple-cont-4 --noplot
python3 anti_flocking.py -m "continuous" -n 3 -d SIMULATIONS/obstacles -f multiple-cont-5 --noplot
python3 anti_flocking.py -m "continuous" -n 3 -d SIMULATIONS/obstacles -f multiple-cont-6 --noplot