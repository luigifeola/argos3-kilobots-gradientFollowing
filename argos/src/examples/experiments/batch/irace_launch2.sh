#!/bin/bash

### How it works for me ###
# in ARGoS folder run the following:
# ./src/examples/experiments/batch/social_behavior.sh /src/examples/experiments/batch socialBehavior.argos

cd ..

wdir=`pwd`
base_config=.$1/$2

if [ ! -e $base_config ]; then
    base_config=$wdir$1/$2
    if [ ! -e $base_config ]; then
        exit 1
    fi
fi

res_dir=$wdir/"results/social_behavior"
if [[ ! -e $res_dir ]]; then
    cmake -E make_directory $res_dir
# else
#     echo "Error: directory '$res_dir' already exists"
#     exit 1
fi

# base_dir=`dirname $base_config_s`
# echo base_dir $base_dir


SEED=$3
INSTANCE=$4
t0=$6
t1=$8
a0=${10}
a1=${12}
r0=${14}
r1=${16}
std=${18}

echo "$SEED"
echo "$INSTANCE"
echo "$t0"
echo "$t1"
echo "$a0"

#here modify params
base_cfileconfig=src/examples/behaviors/social_behavior_base2.c
cfileconfig=src/examples/behaviors/social_behavior.c

cp $base_cfileconfig $cfileconfig

sed -i "s|__std__|$std|g" $cfileconfig
sed -i "s|__t0__|$t0|g" $cfileconfig
sed -i "s|__t1__|$t1|g" $cfileconfig
sed -i "s|__a0__|$a0|g" $cfileconfig
sed -i "s|__a1__|$a1|g" $cfileconfig
sed -i "s|__r0__|$r0|g" $cfileconfig
sed -i "s|__r1__|$r1|g" $cfileconfig

cd build
cmake ../src
make
cd ..

numrobots="25"

###################################
# experiment_length is in seconds #
###################################
experiment_length="3600"
RUNS=1


param_dir=$res_dir/"temp"
# param_dir=$res_dir/"irace_behavior2_config_10"

if [[ ! -e $param_dir ]]; then
    cmake -E make_directory $param_dir
fi

for it in $(seq 1 $RUNS); do

    config=`printf 'config_seed%03d.argos' $it`
    cp $base_config $config
    sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $config
    sed -i "s|__SEED__|$SEED|g" $config
    sed -i "s|__NUMROBOTS__|$numrobots|g" $config

    robot_positions_file="seed#${it}_kiloLOG.tsv"
    sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $config

    argos3 -c './'$config

    mv *.tsv $param_dir
    rm *.argos
done

#call the python script to output something
python_output=$(python3.8 plots/irace_metric.py)
echo "$python_output"

cd tuning
