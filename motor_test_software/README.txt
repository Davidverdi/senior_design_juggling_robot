These are various test scripts for running motors through PWM on the
Raspbery Pi using the PIGPIO libraries. To compile, first download the
PIGPIO libraries at http://abyz.co.uk/rpi/pigpio/index.html

Compile using: g++ -Wall -pthread -o foobar foobar.cpp -lpigpio -lrt

Note that for the encoder test script, you need to compile three files:
rotary_encoder_sample.hpp
rotary_encoder_sample.cpp
test_rotary_encoder_sample.cpp


