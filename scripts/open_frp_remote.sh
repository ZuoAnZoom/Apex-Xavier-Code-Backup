#!/bin/bash

set -e

PASSWORD=<YOUR-PASSWORD>
#echo $PASSWORD | sudo -S systemctl restart supervisor
echo $PASSWORD | sudo -S systemctl start frpc.service
sudo systemctl status frpc.service
