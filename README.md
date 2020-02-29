# Timer2_test

This code and text are Copyright 2020 by Michael Anthony Schwager, aka "GreyGnome". This code is release under the Apache 2.0 license.

See https://forum.arduino.cc/index.php?topic=665510.0

Testing of timer2. This is using an Arduino Due as a clock source for an ATmega328.

I am testing the Valentine's card (also in my repos), and as I did the work, I discovered that:

* When using FastPWM mode on timer2,
* When the counter is set to 255,
* I can't tell the order of events when the blasted ATmega328p overflow and counter compare interrupts take place.

Setup: Two microcontrollers, and Arduino and a bare ATmega328p, connected together by various wires as described.

The Arduino has its builtin FTDI serial chip for programming. Including a bootloader.

The ATmega328p has its builtin 8 MHz clock, and is programmed using a USBASP device. I am connecting a USB hub and programming them using two separate instances of the Arduino IDE, each with different setup under "Tools". The ATmega328p is:
* Board: ATmega328
* Variant: 328P/328PA
* Bootloader: None
* BOD: Disabled
* Clock: 8 MHz Internal
* Compiler LTO: LTO Disabled
* Port: /dev/ttyS4
* Programmer: USBasp

Fun!

With this code, and my Valentine's Card code, you can watch the timer2 procedure take place like this:

* The ATmega328p is using its internal clock, set to 8MHz (script included in this repo for those Linux types).
* The following pins on the ATmega328p are connected to an Arduino Uno-like board (I have the Duemilanove). The pin numbers show are the Arduino pin numbers:
* Pin 20 to pin 12 on the Arduino.
* Pin 15 to the - side of an LED. + of the LED to an appropriate current-limiting resistor. Other side of resistor to Vcc.
* Pin 6 to pin 2 on the Arduino.
* Pin 7 to pin 3 on the Arduino.
* Pin 11 to a 150 ohm (or so) resistor. Other side of the resister will go the the + side of a set of LEDs.
* Pin 0 to the - side of an LED. + side to the above resistor.
* Pin 1 to the - side of another LED. + side to the above resistor.
* Pin 19 to the - side of another LED. + side to the above resistor.

On the Arduino, three momentary pushbutton switches SPST (Single Pole, Single Throw) like this:

* One side to ground (negative). The other side of them as follows:
* One switch to pin 4. This will be the slow clock advance.
* One switch to pin 7. This will be the fast clock advance.
* One switch to pin 8. This will be the clock counter reset button.

The system works together to send a clock signal to the ATmega328p, to count the clock pulses, and to report on the state of the ATmega328p processor. In this way, you can see the internals of the Timer 2 system relatively easily.

I have included the "Valentines_Card.ino" sketch here, even though in its other github location I will advance past this point. Today, however, it has the appropriate code for making the above test work.

This code is "Hackware". In other words, I was hacking around and used it for my own nefarious purposes. If you can use it for something, be it good or nefarious, that's great. But it's not clean, it's not pretty, it's not supported. It's only published because I'd rather not keep it hidden away on my own machines. It's out here on Github so that others can find it and may find a useful gem in it one day.
