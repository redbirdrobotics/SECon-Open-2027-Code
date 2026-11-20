/*
A test file for the CoordProcess.cpp file, 
which contains functions for coordinate 
processing and matrix operations. This 
test file includes unit tests to verify 
the correctness of the functions in CoordProcess.cpp, 
ensuring that they behave as expected under various scenarios.
*/


#include <cassert>
#include <cmath>
#include "CoordProcess.cpp"
#include <cstdio>
#include <zsl/zsl.h>
#include <limits>

using namespace std;

bool approx_equal(float a, float b, float tol = 1e-4f) {
    return std::fabs(a - b) <= tol;
}

//Test 1: setup_conversion_matrices
void test_setup_conversion_matrices() {
    float theta = atan(1.0f); // 45 degrees
    int rc = setup_conversion_matrices(theta);
    assert(rc == 0); // Makes sure the function returns 0 for success

    zsl_real_t val = 0.0f;
    float expected_val = sqrt(2.0f) / 2.0f; // cos(45 degrees) = sin(45 degrees) = sqrt(2)/2

    //Verify the conversion matrix values:
    //[ cos(theta)  -sin(theta) ]
    //[ sin(theta)   cos(theta) ]
    zsl_mtx_get(&conversion_matrix, 0, 0, &val);
    assert(approx_equal(val, expected_val));

    zsl_mtx_get(&conversion_matrix, 0, 1, &val);
    assert(approx_equal(val, -expected_val));

    zsl_mtx_get(&conversion_matrix, 1, 0, &val);
    assert(approx_equal(val, expected_val));

    zsl_mtx_get(&conversion_matrix, 1, 1, &val);
    assert(approx_equal(val, expected_val));

    //Verifies the inverse conversion matrix values:
    //[  cos(theta)   sin(theta) ]
    //[ -sin(theta)   cos(theta) ]
    zsl_mtx_get(&inverse_conversion_matrix, 0, 1, &val);
    assert(approx_equal(val, expected_val));

    zsl_mtx_get(&inverse_conversion_matrix, 1, 0, &val);
    assert(approx_equal(val, -expected_val));

    //conversion matrix * inverse_conversion_matrix should yield the identity matrix
    ZSL_MATRIX_DEF(identity, 2, 2);
    zsl_mtx_mult(&conversion_matrix, &inverse_conversion_matrix, &identity);

    //Check that the identity matrix is correct
    // [ 1 0 ]
    // [ 0 1 ]
    zsl_mtx_get(&identity, 0, 0, &val);
    assert(approx_equal(val, 1.0f));
    zsl_mtx_get(&identity, 0, 1, &val);
    assert(approx_equal(val, 0.0f));
    zsl_mtx_get(&identity, 1, 0, &val);
    assert(approx_equal(val, 0.0f));
    zsl_mtx_get(&identity, 1, 1, &val);
    assert(approx_equal(val, 1.0f));

    printf("Test 1 passed: setup_conversion_matrices\n");
}

int main() {
    test_setup_conversion_matrices();
    return 0;
}