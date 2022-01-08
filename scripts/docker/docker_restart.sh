#!/bin/bash

echo '[INFO] Shut down docker service!'
PASSWORD=<YOUR-PASSWORD>
echo $PASSWORD | sudo -S systemctl stop docker

echo '[INFO] Restart docker service'
sudo systemctl daemon-reload
sudo systemctl start docker

echo '[INFO] Docker status'
sudo systemctl status docker
