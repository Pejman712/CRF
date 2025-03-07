sudo ip link set can$1 up type can bitrate $2 restart-ms 100
sudo ip link set can$1 txqueuelen 1000
