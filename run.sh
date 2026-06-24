#!/bin/bash
echo "Entering the build folder"
cd build
echo "Making the project."
if ls; then
    echo "Loading the uf2 file to the pico."
    # Find the first .uf2 file (you can remove `head -n 1` if you want all of them)
    UF2_FILE=$(find ./ -maxdepth 1 -type f -name "*.uf2" | head -n 1)
    if [ -n "$UF2_FILE" ]; then
        echo "Found UF2 file: $UF2_FILE"
        picotool load "$UF2_FILE" -f
        echo "waiting 1 sec for pico to reboot."
        cd ..
        ./slab_viewer
    else
        echo "No UF2 file found in $FOLDER"
    fi
else
    echo "Build Failed!"
fi
#make -j$(nproc) && sudo picotool load "CROX.uf2" -f && sleep 2 && sudo picocom -b 115200 /dev/ttyACM0
