#!/usr/bin/env bash

echo "Removing old library..."
rm -r common/
rm -r minimal/
rm -r standard/
rm -r checksum.h
rm -r mavlink_*
rm -r protocol.h

echo "Downloading or updating MAVLink sources..."

if [ -d "mavlink-src/.git" ]; then
    cd mavlink-src || return 1
    git fetch
    git checkout origin/master
    cd ../
else
    git clone https://github.com/mavlink/mavlink.git --recursive mavlink-src
fi

PYTHONPATH="$(pwd)/mavlink-src"

echo "Running MAVLink generator..."
python -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=. mavlink-src/message_definitions/v1.0/common.xml --no-validate

echo "Removing inlines..."
sed -i 's/ inline//' protocol.h
