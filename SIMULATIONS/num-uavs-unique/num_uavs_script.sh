#!/bin/bash

# ------------------------
# Shell script - Simulations
# ------------------------

STARTTIMESCRIPT=$(date +%s) #time in seconds
echo "Starting script ..."

mypath=$(pwd) # variable mypath: root path desde el cual estoy ejecutando el script

folder_num_uavs="simulation2_num_uavs"

max_uav=8
min_uav=2
repetitions=5


# ----------------------------- SUMMARY FILE -------------------------------------
mkdir -p $folder_num_uavs
summary_file="$folder_num_uavs/summary_num_uavs_simulation.txt"
touch $summary_file
echo "exec_time,NUM_UAVS,MODE,ALWAYS_COMMUNICATION,total_iter,total_cov_area,average_inst_cov_area,mission_time" > $summary_file

# ----------------------------------------------------------------------------------
# ---------------------------- DIFFERENT NUMBER OF UAVS ----------------------------
# ----------------------------------------------------------------------------------

step=$(( 100 / (($max_uav-$min_uav)*$repetitions) ))
progress=0
for i in $(seq $min_uav $max_uav)
do 
    for j in $(seq 1 $repetitions)
    do 

        echo "Progress: $progress%"

        python3 ../../anti_flocking.py -d "$folder_num_uavs/results" -f $i"num-uav"$j -n $i -m "unique" --noplot  >> $summary_file
        
        progress=$(( $progress + $step))

    done
done


ENDTIMESCRIPT=$(date +%s) #time in seconds

echo "End of for loops"
echo "---------------------"
echo "This script took $(($ENDTIMESCRIPT - $STARTTIMESCRIPT)) seconds to complete..."
