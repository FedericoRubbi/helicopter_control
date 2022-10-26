# Copy main compiled file to rp2040-zero
echo "Loading program..."
if cp "$PWD/build/src/main.uf2" "/media/$USER/RPI-RP2/main.uf2" ; then
    echo "Program loaded."
else
    echo "Error in loading."
fi
# Open serial
echo "Opening serial..."
sleep 3
minicom -b 9600 -o -D /dev/ttyACM0
