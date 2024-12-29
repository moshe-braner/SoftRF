# Steps to build SoftRF_MB for T-Beam with Arduino-IDE under Linux Nix(OS) 

1. Working from within the git clone, provide a symlink to the static libaries from the repo: 
```
mv /home/nrb/Arduino/libraries /home/nrb/Arduino/libraries.old; \
ln -s $(git rev-parse --show-toplevel)/software/firmware/source/libraries /home/nrb/Arduino/
```
2. Install Arduino-IDE with [a package manager](https://en.wikipedia.org/wiki/Package_manager), e.g. Nix and run:
```
nix-shell -p python3 python3Packages.virtualenv 
virtualenv venv
source ./venv/bin/activate
pip install pyserial
arduino-ide
```
3. In Arduino-IDE, point to the folder containing the sketch: ```SoftRF.ino``` 
```File -> Open -> <.../software/firmware/source/SoftRF>``` Opens a new window.
4. Pick the boards manager icon on the left hand side, filter by "espressif", install version 2.0.3.
5. Check the board is connected: ```Tools -> Get Board Info  [OK]``` and see T-Beam on /dev/ttyACM0 or similar at bottom right of window.
6. Compile the sketch (will take some time)
```
Sketch -> Verify/Compile
```
7. Copy and paste the output for future comparison in: ```Arduino-IDE_Sketch_Verify-Compile_Output.txt```
8. Generate the binaries (will take some time)
```
Sketch -> Export Compiled Binary
```
9. Get image info for future comparison:
```
nix-shell -p esptool 
esptool.py image_info --version 2  ./software/firmware/source/SoftRF/build/esp32.esp32.esp32/SoftRF.ino.bin  >./software/firmware/source/SoftRF/esptool_image_info.txt
```