#include "zsl.h"
#include "CoordProcess.hpp"
#include <cmath>
#include <assert.h>
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
    zsl_real_t c_data[4] = {cos(theta), -sin(theta),
                            sin(theta), cos(theta)};
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


/*


*/
static zsl_mtx UpdateLocalToGlobalMtx(float t_x, float t_y, float phi){
    zsl_mtx result;
    return result;
}

static zsl_mtx PolarToCartesian(zsl_mtx &polar_matrix){

    // creates and initalizes the cartesian matrix
    int result;
    ZSL_MATRIX_DEF(cartesian, 2, 1);
    zsl_mtx_init(&cartesian, zsl_mtx_entry_fn_empty);

    // gets magnitude and theta of polar matrix
    zsl_real_t magnitude, theta;
    result = zsl_mtx_get(&polar_matrix, 0, 0, &magnitude);
    if(result) return cartesian;
    result = zsl_mtx_get(&polar_matrix, 1, 0, &theta);
    if(result) return cartesian;

    zsl_real_t x = magnitude * cos(theta);
    zsl_real_t y = magnitude * sin(theta);

    // sets calculated x & y coords into cartesian matrix
    zsl_mtx_set(&cartesian, 0, 0, x);        
    zsl_mtx_set(&cartesian, 1, 0, y);        

    return cartesian;
}