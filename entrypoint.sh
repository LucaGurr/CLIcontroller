#!/bin/bash
echo "Waiting for GPIO simulator..."
while [ ! -f "/gpio/gpio17/value" ]; do
    sleep 0.1
done
echo "GPIO simulator ready. Starting control program..."
exec ./control
