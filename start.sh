#!/bin/bash
echo "Enter IP"
read ip
password = "password"
sshpass -p $password ssh pi@$ip

