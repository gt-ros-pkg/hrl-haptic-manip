#!/bin/bash

echo "Starting M3SKIN compile."

make proto
sudo make proto_install
make hardware
sudo make hardware_install
make printer
sudo make printer_install
sudo make python_install
sudo make install
echo "compile finished."

exit 0

