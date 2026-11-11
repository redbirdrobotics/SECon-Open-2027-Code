#ifndef COORDPROCESS_H_
#define COORDPROCESS_H_



#include "pico/stdlib.h"
#include "zsl/matrices.h"

/*
*These two functions will convert the coordinates of objects between global and local frames of references
*The functions will depend on the recorded displacement in the x and y direction and the angle rotation,
*/
static zsl_mtx UpdateGlobalToLocalMtx(float t_x, float t_y, float phi);
static zsl_mtx UpdateLocalToGlobalMtx(float t_x, float t_y, float phi);
static zsl_mtx CartesianToPolar(float x, float y);
static zsl_mtx PolarToCartesian(float r, float theta);

#endif