# Copy main compiled file to rp2040-zero
cp "$PWD/build/src/main.uf2" "/media/$USER/RPI-RP2/main.uf2"
# Open serial with delay
sleep 10
minicom -b 9600 -o -D /dev/ttyACM0
