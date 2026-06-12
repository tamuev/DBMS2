#!/bin/sh
set -e

grep -rl 'fcyclomatic-complexity' --exclude=deploy.sh . | xargs -r sed -i 's/ -fcyclomatic-complexity//g'
make -C Release all

arm-none-eabi-objcopy -O binary "Release/DBMS2.elf" "Release/DBMS2.bin"
python3 upload.py Release/DBMS2.bin --device dbms