#!/bin/sh
set -e

grep -rl 'fcyclomatic-complexity' --exclude=deploy.sh . | xargs -r sed -i 's/ -fcyclomatic-complexity//g'
cmake --build build/Release

arm-none-eabi-objcopy -O binary "build/Release/DBMS2.elf" "build/Release/DBMS2.bin"
python3 upload.py build/Release/DBMS2.bin --device dbms