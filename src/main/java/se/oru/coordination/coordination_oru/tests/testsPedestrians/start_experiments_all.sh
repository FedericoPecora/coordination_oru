#!/bin/sh
export P_TIME="t1"
export S_TYPE="corridor"
mkdir journal_logs/corridor-t1
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"

export P_TIME="t2"
export S_TYPE="corridor"
mkdir journal_logs/corridor-t2
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"

export P_TIME="t3"
export S_TYPE="corridor"
mkdir journal_logs/corridor-t3
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"

export P_TIME="t1"
export S_TYPE="corridor2"
mkdir journal_logs/corridor2-t1
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"

export P_TIME="t2"
export S_TYPE="corridor2"
mkdir journal_logs/corridor2-t2
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"

export P_TIME="t3"
export S_TYPE="corridor2"
mkdir journal_logs/corridor2-t3
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"

export P_TIME="t1"
export S_TYPE="corridor3"
mkdir journal_logs/corridor3-t1
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"

export P_TIME="t2"
export S_TYPE="corridor3"
mkdir journal_logs/corridor3-t2
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"

export P_TIME="t3"
export S_TYPE="corridor3"
mkdir journal_logs/corridor3-t3
./start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"