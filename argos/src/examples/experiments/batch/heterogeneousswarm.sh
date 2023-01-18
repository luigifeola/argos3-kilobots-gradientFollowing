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

res_dir=$wdir/"results/heterogeneous/4-bits"
if [[ ! -e $res_dir ]]; then
    cmake -E make_directory $res_dir
# else
#     echo "Error: directory '$res_dir' already exists"
#     exit 1
fi

echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1

socialrobots="0 5 10 15 20 25"

###################################
# experiment_length is in seconds #
###################################
experiment_length="5000"
date_time=`date "+%Y-%m-%d"`
RUNS=100



for nrob in $socialrobots; do
    
    param_dir=$res_dir/"heterogeneous#"$date_time"_socialRobots#"$nrob"_seconds#"$experiment_length
    if [[ ! -e $param_dir ]]; then
        cmake -E make_directory $param_dir
    fi

    for it in $(seq 1 $RUNS); do
        config=`printf 'config_seed%03d.argos' $it`
        cp $base_config $config
        sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $config
        sed -i "s|__SEED__|$it|g" $config
        sed -i "s|__SOCIALROBOTS__|$nrob|g" $config

        robot_positions_file="seed#${it}_kiloLOG.tsv"
        echo $robot_positions_file
        sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $config



        echo "Running next configuration seed $it with $nrob  social robots"
        echo "argos3 -c $1$config"
        argos3 -c './'$config


        mv *.tsv $param_dir
        rm *.argos
    done
done
