#include "avr/pgmspace.h"

// Retrieval buffer - make sure it's larger than the largest string you wish to store
char p_buffer[80];

// Wrap statically declared strings in P() to save them to Program Memory
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)