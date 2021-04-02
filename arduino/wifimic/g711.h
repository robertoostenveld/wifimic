// See http://www.speech.kth.se/cost250/refsys/v1.0/src/g711.c

#ifndef _G711_H_
#define _G711_H_

unsigned char linear2ulaw(short);
unsigned char linear2alaw(short);

short ulaw2linear(unsigned char);
short alaw2linear(unsigned char);
  
unsigned char ulaw2alaw(unsigned char);
unsigned char alaw2ulaw(unsigned char);
  
#endif
