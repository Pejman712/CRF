#!/bin/sh
cd ~/cpproboticframework/build

sudo ../scripts/setCAN.sh 0 1000000 &

../bin/CERNBot2CommunicationPoint --can_port can0 --configuration ../modules/Robots/CERNBot2/config/cernBot2Config.json --protocol tcp --port 3001 &
