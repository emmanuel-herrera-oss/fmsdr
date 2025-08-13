# fmsdr
A simple FM tuner for RTL SDR devices

## Dependencies
1. rtl-sdr-devel
2. ncurses
3. alsa

If you're building on Linux, you probably already have 2 and 3. 

## Building
Run the build.sh script on a system with clang or change it to use whatever compiler you're using.

## Controls
- Left and Right arrows to change tune 
- 1 to toggle Overlap Add
- 2 to toggle the FM Filter
- 3 to toggle the Audio Filter
- 4 to toggle yielding of excess cpu time

## Demo
[emmanuel-herrera-oss.github.io/real-time-dsp/](emmanuel-herrera-oss.github.io/real-time-dsp/) at the bottom of the page
