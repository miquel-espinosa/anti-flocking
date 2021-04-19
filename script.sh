#!/bin/bash

# ------------------------
# Shell script - Simulations
# ------------------------

STARTTIMESCRIPT=$(date +%s) #time in seconds
echo "Starting script ..."

mypath=$(pwd) # creo una variable mypath que contiene my root path desde el cual estoy ejecutando el script
# echo "Estoy en el directorio $mypath" #así se ponen commandos con variables

folder="simulation1"
# mkdir $folder # este comando crea una carpeta principal root


python3 anti_flocking.py -f "$folder/2uav-unique1" -n 2 -m "unique"
python3 anti_flocking.py -f "$folder/2uav-unique2" -n 2 -m "unique"
python3 anti_flocking.py -f "$folder/2uav-unique3" -n 2 -m "unique"

python3 anti_flocking.py -f "$folder/3uav-unique1" -n 3 -m "unique"
python3 anti_flocking.py -f "$folder/3uav-unique2" -n 3 -m "unique"
python3 anti_flocking.py -f "$folder/3uav-unique3" -n 3 -m "unique"

python3 anti_flocking.py -f "$folder/4uav-unique1" -n 4 -m "unique"
python3 anti_flocking.py -f "$folder/4uav-unique2" -n 4 -m "unique"
python3 anti_flocking.py -f "$folder/4uav-unique3" -n 4 -m "unique"

python3 anti_flocking.py -f "$folder/5uav-unique1" -n 5 -m "unique"
python3 anti_flocking.py -f "$folder/5uav-unique2" -n 5 -m "unique"
python3 anti_flocking.py -f "$folder/5uav-unique3" -n 5 -m "unique"

python3 anti_flocking.py -f "$folder/6uav-unique1" -n 6 -m "unique"
python3 anti_flocking.py -f "$folder/6uav-unique2" -n 6 -m "unique"
python3 anti_flocking.py -f "$folder/6uav-unique3" -n 6 -m "unique"


python3 anti_flocking.py -f "$folder/2uav-continuous1" -n 2 -m "continuous"
python3 anti_flocking.py -f "$folder/2uav-continuous2" -n 2 -m "continuous"
python3 anti_flocking.py -f "$folder/2uav-continuous3" -n 2 -m "continuous"

python3 anti_flocking.py -f "$folder/3uav-continuous1" -n 3 -m "continuous"
python3 anti_flocking.py -f "$folder/3uav-continuous2" -n 3 -m "continuous"
python3 anti_flocking.py -f "$folder/3uav-continuous3" -n 3 -m "continuous"

python3 anti_flocking.py -f "$folder/4uav-continuous1" -n 4 -m "continuous"
python3 anti_flocking.py -f "$folder/4uav-continuous2" -n 4 -m "continuous"
python3 anti_flocking.py -f "$folder/4uav-continuous3" -n 4 -m "continuous"

python3 anti_flocking.py -f "$folder/5uav-continuous1" -n 5 -m "continuous"
python3 anti_flocking.py -f "$folder/5uav-continuous2" -n 5 -m "continuous"
python3 anti_flocking.py -f "$folder/5uav-continuous3" -n 5 -m "continuous"

python3 anti_flocking.py -f "$folder/6uav-continuous1" -n 6 -m "continuous"
python3 anti_flocking.py -f "$folder/6uav-continuous2" -n 6 -m "continuous"
python3 anti_flocking.py -f "$folder/6uav-continuous3" -n 6 -m "continuous"



ENDTIMESCRIPT=$(date +%s) #time in seconds

echo "End of for loops"
echo "---------------------"
echo "This script takes $(($ENDTIMESCRIPT - $STARTTIMESCRIPT)) seconds to complete..."
