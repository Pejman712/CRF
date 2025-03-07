#!/bin/sh

# Move to base directory
cd ~/Projects/Master/cpproboticframework

# Enable the different CAN ports
sudo ./scripts/setCAN.sh 0 1000000
sudo ./scripts/setCAN.sh 1 1000000

# Launch the robot base communication point
./bin/CERNBotCommunicationPoint --can_port can0 --configuration ./modules/Robots/CERNBot/config/cernBotConfig.json --protocol tcp --port 3001 &

# Launch the robot arm communication points
./bin/SchunkArmControllerPoint --can_port can1 --configuration ./modules/Robots/SchunkArm/configuration/SchunkLWA4P.json --protocol tcp --port 3002 &
