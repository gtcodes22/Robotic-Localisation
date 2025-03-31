#!/bin/bash

PASSWORD="turtlebot"

gnome-terminal -- bash -c "
    sshpass -p '$PASSWORD' ssh ubuntu@172.20.10.3;
    exec bash"