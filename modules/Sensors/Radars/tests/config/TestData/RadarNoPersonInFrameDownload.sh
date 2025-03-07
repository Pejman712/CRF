#!/bin/bash

#removing square brackets from copied path
pathToFile=${1#"["}
pathToFile=${pathToFile%"]"}

#adding TestData folder
pathToFile+="/bin/TestData/Sensors/Radar"
mkdir -p $pathToFile

#enter directory
cd $pathToFile

FILE=RadarNoPersonInFrame.dat
if [ ! -f "$FILE" ]; then
    #download process
    wget https://cernbox.cern.ch/index.php/s/QSPiE7hWWQEpmq6/download -O $FILE
else
      echo "$FILE already exists"
fi
