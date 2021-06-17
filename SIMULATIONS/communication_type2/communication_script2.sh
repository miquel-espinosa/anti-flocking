#!/bin/bash

# ------------------------
# Shell script - Simulations COMMUNICATION RANGES: special case for 2, 4 and 5 UAVs 
# ------------------------

STARTTIMESCRIPT=$(date +%s) #time in seconds
echo "Starting script ..."

mypath=$(pwd) # variable mypath: root path desde el cual estoy ejecutando el script

folder_num_uavs="communication_range"

min_uav=2
uav_step=2
max_uav=4
min_range=15
max_range=45
range_step=15
repetitions=10


# ----------------------------- SUMMARY FILE -------------------------------------
mkdir -p $folder_num_uavs
summary_file="$folder_num_uavs/summary_num_uavs_simulation.txt"
touch $summary_file
echo "exec_time,NUM_UAVS,MODE,ALWAYS_COMMUNICATION,total_iter,total_cov_area,average_inst_cov_area,mission_time,comm-type" > $summary_file

# ----------------------------------------------------------------------------------
# ---------------------------- DIFFERENT NUMBER OF UAVS ----------------------------
# ----------------------------------------------------------------------------------

# step=$(( 100 / (($max_uav-$min_uav)*$repetitions) ))
# progress=0
for u in $(seq $min_uav $uav_step $max_uav)
do 
    for c in $(seq $min_range $range_step $max_range)
    do 
        for i in $(seq 1 $repetitions)
        do 

            echo "Experiment: $u uavs | $c range | sample: $i"

            python3 ../../anti_flocking.py -d "$folder_num_uavs/results" -f $u"-uav-"$c"-range-"$i -n $u -m "unique" -c $c --noplot  >> $summary_file

        done
    done
    for i in $(seq 1 $repetitions)
    do 

        echo "Experiment: $u uavs| always communication | sample $i"

        python3 ../../anti_flocking.py -d "$folder_num_uavs/results" -f $u"-uav-alwayscomm-"$i -n $u -m "unique" --noplot --alwayscomm >> $summary_file

    done
done

for c in $(seq $min_range $range_step $max_range)
do 
    for i in $(seq 1 $repetitions)
    do 

        echo "Experiment: 5 uavs | $c range | sample: $i"

        python3 ../../anti_flocking.py -d "$folder_num_uavs/results" -f "5-uav-"$c"-range-"$i -n 5 -m "unique" -c $c --noplot  >> $summary_file

    done
done
for i in $(seq 1 $repetitions)
    do 

        echo "Experiment: 5 uavs| always communication | sample $i"

        python3 ../../anti_flocking.py -d "$folder_num_uavs/results" -f "5-uav-alwayscomm-"$i -n 5 -m "unique" --noplot --alwayscomm >> $summary_file

    done

ENDTIMESCRIPT=$(date +%s) #time in seconds

echo "End of for loops"
echo "---------------------"
echo "This script took $(($ENDTIMESCRIPT - $STARTTIMESCRIPT)) seconds to complete..."
