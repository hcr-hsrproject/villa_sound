#!/bin/bash

if [ -z "$1" ]
  then
    echo "Pass in output directory as argument."
    exit 1
fi

OUTPUT_DIR="$( readlink -e "$1" )"
PARENT_DIR="$( dirname "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" )"
cd "$PARENT_DIR"

echo "Note: You may have to change the audio device number"
echo "Ctrl-C to quit.\n"

for (( ; ; ))
do
    echo "Enter degrees [e.g. 010]: "
    read DEG
    # You may have to change --audio
    # Check with python audio_io_view.py
    python imp_tsp.py --length 16384 --sampfreq 16000 --sync 16 --input 4 --wave 1 --playrec 1 --audio 9
    mv tsp_in.wav "${OUTPUT_DIR}/r100_d${DEG}.wav"
done
