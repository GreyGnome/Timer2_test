# Timer2_test
Testing of timer2. This is using an Arduino Due as a clock source for an ATmega328.

I am testing the Valentine's card, and as I did the work, I discovered that

* When using FastPWM mode on timer2,
* When the counter is set to 255,
* I can't tell the order of events when the blasted ATmega328p overflow and counter compare interrupts take place.
