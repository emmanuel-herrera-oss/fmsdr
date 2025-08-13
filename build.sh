clang -O2 filter_coefficients.c main.c -lm -lrtlsdr -lncurses -o fmsdr
if [ $? -eq 0 ]; then
    echo "Build succeeded."
else
    echo "Build failed."
fi
