#!/bin/bash

# ------------------------
# Shell script - Simulations
# ------------------------

STARTTIMESCRIPT=$(date +%s) #time in seconds
echo "Starting script ..."

mypath=$(pwd) # creo una variable mypath que contiene my root path desde el cual estoy ejecutando el script
# echo "Estoy en el directorio $mypath" #así se ponen commandos con variables

folder_num_uavs="simulation_num_uavs_no_obstacles"
radius_comm="radius_comm"
always_comm="always_comm"

max_uav=8
repetitions=3


# ----------------------------- SUMMARY FILE -------------------------------------
mkdir -p $folder_num_uavs
summary_file="$folder_num_uavs/summary_simulation.txt"
touch $summary_file
echo "exec_time,NUM_UAVS,MODE,ALWAYS_COMMUNICATION,total_iter,total_cov_area,average_inst_cov_area,mission_time" > $summary_file

# ----------------------------------------------------------------------------------
# ---------------------------- DIFFERENT NUMBER OF UAVS ----------------------------
# ----------------------------------------------------------------------------------

for i in $(seq 2 $max_uav)
do 
    for j in $(seq 1 $repetitions)
    do 
        python3 anti_flocking.py -d "$folder_num_uavs/$radius_comm" -f $i"uav-unique"$j -n $i -m "unique" --noplot  >> $summary_file
    done
done

for i in $(seq 2 $max_uav)
do 
    for j in $(seq 1 $repetitions)
    do 
        python3 anti_flocking.py -d "$folder_num_uavs/$radius_comm" -f $i"uav-continuous"$j -n $i -m "continuous" --noplot >> $summary_file
    done
done

for i in $(seq 2 $max_uav)
do 
    for j in $(seq 1 $repetitions)
    do 
        python3 anti_flocking.py -d "$folder_num_uavs/$always_comm" -f $i"uav-unique"$j -n $i -m "unique" --noplot --alwayscomm >> $summary_file
    done
done

for i in $(seq 2 $max_uav)
do 
    for j in $(seq 1 $repetitions)
    do 
        python3 anti_flocking.py -d "$folder_num_uavs/$always_comm" -f $i"uav-continuous"$j -n $i -m "continuous" --noplot --alwayscomm >> $summary_file
    done
done




ENDTIMESCRIPT=$(date +%s) #time in seconds

echo "End of for loops"
echo "---------------------"
echo "This script took $(($ENDTIMESCRIPT - $STARTTIMESCRIPT)) seconds to complete..."
