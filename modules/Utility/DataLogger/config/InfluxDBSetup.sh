#!/bin/sh

echo "##########################################################################"
echo "Setting up Influx Server"
echo "##########################################################################"
echo "Adding the InfluxData repository "
echo "##########################################################################"

sudo curl -sL https://repos.influxdata.com/influxdb.key | sudo apt-key add -
sudo echo "deb https://repos.influxdata.com/ubuntu bionic stable" | sudo tee /etc/apt/sources.list.d/influxdb.list

echo "##########################################################################"
echo "Downloading Influxdb And Influxd-CLI "
echo "##########################################################################"

sudo apt update && sudo apt install influxdb

sudo systemctl stop influxdb
sudo systemctl start influxdb
sudo systemctl enable --now influxdb
sudo systemctl is-enabled influxdb

sudo systemctl status influxdb

echo "q"

echo "##########################################################################"
echo "Running configurations "
echo "##########################################################################"

sudo sed -i '247s/# enabled = true/  enabled = true/' /etc/influxdb/influxdb.conf

sudo sed -i '250s/# flux-enabled = false/  flux-enabled = true/' /etc/influxdb/influxdb.conf

sudo sed -i '253s/# flux-log-enabled = false/  flux-log-enabled = true/' /etc/influxdb/influxdb.conf

sudo sed -i '256s/# bind-address = ":8086"/  bind-address = ":8086"/' /etc/influxdb/influxdb.conf

sudo sed -i '80s/# cache-max-memory-size = "1g"/   cache-max-memory-size = "2g"/' /etc/influxdb/influxdb.conf

echo "##########################################################################"
echo "configurations updated **"
echo "##########################################################################"
echo "database started"
echo "##########################################################################"
