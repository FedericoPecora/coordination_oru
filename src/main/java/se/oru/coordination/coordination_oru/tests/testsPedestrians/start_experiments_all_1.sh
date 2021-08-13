#!/bin/sh
export P_TIME="t1"
export S_TYPE="corridor1"
mkdir journal_logs/corridor1-t1
./src/main/java/se/oru/coordination/coordination_oru/tests/testsPedestrians/start_experiments.sh
echo "*********************************"
echo "*********************************"
echo "************* DONE **************"
echo "*********************************"
echo "*********************************"
