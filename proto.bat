@echo off
python Protobuf\generate_arduino.py -s Arm -p arm
python Protobuf\generate_arduino.py -s Arm -p version
python Protobuf\generate_arduino.py -s Arm -p utils

python Protobuf\generate_arduino.py -s Gripper -p arm
python Protobuf\generate_arduino.py -s Gripper -p version
python Protobuf\generate_arduino.py -s Gripper -p utils
