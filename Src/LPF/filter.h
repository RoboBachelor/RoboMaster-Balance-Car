#ifndef FILTER_H
#define FILTER_H

#include "main.h"
#include "math.h"

typedef struct{
	float freq;
	float a;
	float out;
} filter_t;

void filter_init(filter_t *filter, float freq);	
float filter_calc(filter_t *filter, float current);

#endif
