#include "zsl.h"
#include "CoordProcess.hpp"
#include <math.h>
/*
*Creates the matrix relation between cartesian and polar coordinates of form
* [
* cos(theta) -sin(theta)
* sin(theta) cos(theta)
*]
*/
ZSL_MATRIX_DEF(conversion_matrix, 2, 2);
ZSL_MATRIX_DEF(inverse_conversion_matrix, 2, 2);


int setup_conversion_matrices(float theta){
    zsl_real_t c_data[4] = {Math.cos(theta), -Math.sin(theta),
                            Math.sin(theta), Math.cos(theta)};
    zsl_mtx_from_arr(&conversion_matrix, c_data);
    zsl_mtx_init(&inverse_conversion_matrix, NULL);
    int rc = zsl_mtx_inv(&conversion_matrix, &inverse_conversion_matrix);

    if(rc != 0){
        /*If this occurs, then the matrix has a determinant of 0, which means that the matrix is singular and cannot be inverted. 
        *This should not happen with a matrices were dealing with, so we will return an error code
        */
        return rc;
    }
    return 0;
}

static zsl_mtx UpdateGlobalToLocalMtx(zsl_mtx coordinate_matrix, float t_x, float t_y, float phi){
    zsl_mtx result;
    return result;
};
static zsl_mtx UpdateLocalToGlobalMtx(float t_x, float t_y, float phi){
    zsl_mtx result;
    return result;
};
static zsl_mtx CartesianToPolar(float x, float y){
    zsl_mtx result;
    return result;
};
static zsl_mtx PolarToCartesian(float r, float theta){

    zsl_mtx result;
    return result;
};