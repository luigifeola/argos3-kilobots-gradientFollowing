#!/bin/bash

### How it works for me ###
# in ARGoS folder run the following:
# ./src/examples/experiments/batch/heterogeneousswarm.sh /src/examples/experiments/batch heterogeneousswarm.argos

if [ "$#" -ne 2 ]; then
    echo "Usage: experiment.sh (from src folder) <config_dir> <argos_fileName>"
    exit 11
fi

wdir=`pwd`
base_config=.$1/$2
echo "base_config:" $base_config
if [ ! -e $base_config ]; then
    base_config=$wdir$1/$2
    if [ ! -e $base_config ]; then
        echo "Error: missing configuration file '$base_config'" 1>&2
        exit 1
    fi
fi

res_dir=$wdir/"results/scalability/3-bits"
if [[ ! -e $res_dir ]]; then
    cmake -E make_directory $res_dir
# else
#     echo "Error: directory '$res_dir' already exists"
#     exit 1
fi

echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1


###################################
# experiment_length is in seconds #
###################################
date_time=`date "+%Y-%m-%d"`
# experiment_length="2"
experiment_length="5000"
RUNS=100

socialrobots="0 0.2 0.4 0.6 0.8"
numrobots="25 50 100"
arena_size="0.7 1 1.25 1.5 1.75"



for nrob in $numrobots; do
    for a_size in $arena_size; do
        for nsocial in $socialrobots; do
            
            float1=0.02
            float_asize=$(echo "$a_size" | bc -l)
            posdistr=$(echo "$float_asize / 2.0 - $float1" | bc -l)

            echo "The pos distribution is: $posdistr"

            float_nrob=$(echo "$nrob" | bc -l)
            float_nsocial=$(echo "$nsocial" | bc -l)
            social_size=$(echo "$float_nrob * $float_nsocial" | bc -l)
            

            echo "The total robots are: $nrob"
            echo "The arena size is: $a_size"
            echo "The social robots are: $social_size"

            param_dir=$res_dir/"heterogeneous#"$date_time"_arenasize#"$(echo "$a_size * 100" | bc -l)"_numrobots#"$nrob"_socialRobots#"$social_size"_seconds#"$experiment_length
            if [[ ! -e $param_dir ]]; then
                cmake -E make_directory $param_dir
            fi

            for it in $(seq 1 $RUNS); do
                config=`printf 'config_seed%03d.argos' $it`
                cp $base_config $config
                sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $config
                sed -i "s|__SEED__|$it|g" $config
                sed -i "s|__SOCIALROBOTS__|$social_size|g" $config
                sed -i "s|__NUMROBOTS__|$nrob|g" $config
                sed -i "s|__ARENASIZE__|$a_size|g" $config
                sed -i "s|__POSDISTR__|$posdistr|g" $config

                robot_positions_file="seed#${it}_kiloLOG.tsv"
                echo $robot_positions_file
                sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $config



                echo "Running next configuration seed $it with $social_size  social robots"
                echo "argos3 -c $1$config"
                argos3 -c './'$config


                mv *.tsv $param_dir
                rm *.argos
            done
        done
    done
done
