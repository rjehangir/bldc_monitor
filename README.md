BLDC Monitor
========

The BLDC Monitor is an Arduino compatible board with voltage, current, and RPM sensors for a three-phase brushless motor. The board has components to monitor two motors simultaneously. The software is written in C++ and communicates with a Python program through serial. An ncurses interface allows the motor to be controlled with the arrow keys on the keyboard. Data is displayed in the terminal and also plotted live through Plot.ly.

##Features

* Atmega328p microcontroller
* RPM sensor that uses motor wire for input. An optoisolator protects the rest of the circuit and an active low-pass filter removes high frequency PWM and converts the EMF wave to a binary signal.
* Allegro hall effect current sensors for accurate current measurements
* Measures 2 motors simultaneously

##Interface

The interface is written in Python. It uses the curses, crcmod, serial, and plotly libraries. To run the interface:

```bash
./bldc_monitor.py -p [serial port] -o [csv file to write (optional)]
```

##Releases

TBD
