@echo off

echo Removing old library...
del /q common\
del /q minimal\
del /q standard\
del /q checksum.h
del /q mavlink_*
del /q protocol.h

echo Downloading or updating MAVLink sources...

if exist mavlink-src\.git\ (
    cd mavlink-src
    git fetch
    git checkout origin/master
    cd ../
) else (
    git clone https://github.com/mavlink/mavlink.git --recursive mavlink-src
)

set PYTHONPATH=%CD%\mavlink-src

echo Running MAVLink generator...
python -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=. mavlink-src/message_definitions/v1.0/common.xml --no-validate