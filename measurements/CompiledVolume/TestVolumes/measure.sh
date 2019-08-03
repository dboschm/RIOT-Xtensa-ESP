for f in ../tests/** ; do
    echo $f
    cd $f
    make BOARD=arduino-mega2560
    cd ../../measurements
done;