#!/bin/bash

# ------------------------
# Shell script - Simulations COMMUNICATION RANGES: special case for 2, 4 and 5 UAVs 
# ------------------------

STARTTIMESCRIPT=$(date +%s) #time in seconds
echo "Starting script ..."

mypath=$(pwd) # variable mypath: root path desde el cual estoy ejecutando el script

folder_num_uavs="communication_range"

uavs=( 2 3 4 5 6 9 )
ranges=( 5 10 )
repetitions=10


# ----------------------------- SUMMARY FILE -------------------------------------
mkdir -p $folder_num_uavs
summary_file="$folder_num_uavs/summary_num_uavs_simulation.txt"
touch $summary_file
echo "exec_time,NUM_UAVS,MODE,ALWAYS_COMMUNICATION,total_iter,total_cov_area,average_inst_cov_area,mission_time,coverage_ratio" > $summary_file

for u in "${uavs[@]}"
do 
    for c in "${ranges[@]}"
    do 
        for i in $(seq 1 $repetitions)
        do 

            echo "Experiment: $u uavs | $c range | sample: $i"

            python3 ../../anti_flocking.py -d "$folder_num_uavs/results" -f $u"-uav-"$c"-range-"$i -n $u -m "unique" -c $c --noplot  >> $summary_file

        done
    done
done


ENDTIMESCRIPT=$(date +%s) #time in seconds

echo "End of for loops"
echo "---------------------"
echo "This script took $(($ENDTIMESCRIPT - $STARTTIMESCRIPT)) seconds to complete..."
