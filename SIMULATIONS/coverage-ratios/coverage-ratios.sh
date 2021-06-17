#!/bin/bash

# ------------------------
# Shell script - Simulations COVERAGE RATIOS: 
# ------------------------

STARTTIMESCRIPT=$(date +%s) #time in seconds
echo "Starting script ..."

mypath=$(pwd) # variable mypath: root path desde el cual estoy ejecutando el script

folder_num_uavs="coverage_ratios_results"

min_uav=2
max_uav=6
repetitions=10
ratios=( 99 97 95 90 )

# ----------------------------- SUMMARY FILE -------------------------------------
mkdir -p $folder_num_uavs
summary_file="$folder_num_uavs/summary_num_uavs_simulation.txt"
touch $summary_file
echo "exec_time,NUM_UAVS,MODE,ALWAYS_COMMUNICATION,total_iter,total_cov_area,average_inst_cov_area,mission_time,max_coverage_ratio" > $summary_file

# ----------------------------------------------------------------------------------
# ---------------------------- DIFFERENT NUMBER OF UAVS ----------------------------
# ----------------------------------------------------------------------------------

# step=$(( 100 / (($max_uav-$min_uav)*$repetitions) ))
# progress=0
for u in $(seq $min_uav $max_uav)
do 
    for r in "${ratios[@]}"
    do 
        for i in $(seq 1 $repetitions)
        do 

            echo "Experiment: $u uavs | $r ratio | sample: $i"

            python3 ../../anti_flocking.py -d "$folder_num_uavs/results" -f $u"-uav-"$r"-ratio-"$i -n $u -m "unique" -r $r --noplot  >> $summary_file

        done
    done
done


ENDTIMESCRIPT=$(date +%s) #time in seconds

echo "End of for loops"
echo "---------------------"
echo "This script took $(($ENDTIMESCRIPT - $STARTTIMESCRIPT)) seconds to complete..."
