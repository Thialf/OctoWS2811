OctoWS2811
==========

Control thousands of WS2811/2812 LEDs at video refresh speeds

http://www.pjrc.com/teensy/td_libs_OctoWS2811.html

https://www.youtube.com/watch?v=M5XQLvFPcBM

Modifications from Thialf:
No "helper" pins are needed. Pins 3,4,15,16 are not needed anymore. All port-interrupts are usable. No disable_interrupt call, not even for initialization. The DMA triggering is done with the FTM0 timer and its compare registers 0..2. It's easy to change them to any other compare registers, if they are needed for PWM output. But PWM output frequency is fixed to 400 or 740kHz for those pins that are using FTM0.

Because I still had flickering at the end of my 240x WS2812B LED strip, I had to change the frequency to 740kHz. 

The output port can be changed between A, B, C, D, E. Pins 0..7 are used, but if you don't need all 8 pins, you can configure less than 8. The other pins can be used as Inputs for other purposes. But you can't use them for other GPIO Output functionality, because the DMA controller is writing to it. 

It's tested with Teensy 3.x at 48..96MHz. 
