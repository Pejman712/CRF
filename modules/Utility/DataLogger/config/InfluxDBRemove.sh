#!/bin/sh

echo "##########################################################################"
echo "Removing Influxd"
echo "##########################################################################"

sudo apt-get purge --auto-remove influxdb -y

sudo pkill -U influxdb
