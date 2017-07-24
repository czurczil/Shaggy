#include "utils.h"

float trim(float a, float min, float max)
{
    if(a<min) return min;
    if(a>max) return max;
    return a;
}
