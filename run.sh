#!/bin/bash
# Eg ./run.sh -c 4 -p 0 -r myagent -h 127.0.0.1 -f fname

# Default values
C4_CONFIG="../Labs/rmi-2223/C4-config.xml"
C4_LAB="../Labs/rmi-2223/C4-lab.xml"
C4_GRID="../Labs/rmi-2223/C4-grid.xml"


# Parse command-line options h,p,r are inputs for MainRob.py
# c is the confic,lab and grid files. 4 uses the c4 files 
# f is the file name for the output file
while getopts ":c:p:r:h:f:" opt; do
  case $opt in;
    c) C4_CONFIG=$OPTARG;;
    p) C4_LAB=$OPTARG;;
    r) C4_GRID=$OPTARG;;
    h) HOST=$OPTARG;;
    f) FILE_NAME=$OPTARG;;

    \?) echo "Invalid option -$OPTARG" >&2;;
  esac
done

ARGS="--param $C4_CONFIG"
ARGS+=" --lab $C4_LAB"
ARGS+=" --grid $C4_GRID"

# Start the agent
(cd simulator; ./simulator $ARGS) &

# Wait for the agent to start
sleep 2

python3 pClient/MainRob.py -n $AGENT_NAME -h $HOST -f $FILE_NAME &
# Start the viewer with autoconnect
(cd Viewer; ./Viewer --autoconnect)

# Terminate the simulator
killall simulator

echo "Bye"
