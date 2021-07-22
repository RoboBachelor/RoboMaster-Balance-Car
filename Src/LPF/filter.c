#include "filter.h"

void filter_init(filter_t *filter, float freq){
	filter->freq = freq;
	filter->a = exp(-0.001 * freq);
}
	
float filter_calc(filter_t *filter, float current){
	//filter->a = exp(-0.001 * filter->freq);
	return filter->out = filter->a * filter->out + (1 - filter->a) * current;
}
