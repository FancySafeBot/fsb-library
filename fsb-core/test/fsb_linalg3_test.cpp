
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_linalg3.h"
#include "fsb_types.h"

TEST_SUITE_BEGIN("linalg3");

TEST_CASE("Eigenvalues of 3x3 symmetic positive definite" * doctest::description("[fsb::mat3_posdef_symmetric_eigenvalues]"))
{
    const fsb::Mat3Sym input_mat = {
         1.12, 0.5, 2.0, 0.32, 0.11, 0.125
    };

    const bool expected_is_pd = true;
    const fsb::Vec3 expected_eigenvalues = {0.36123590771763087, 1.2266495577565546, 2.0321145345258147};

    fsb::Vec3 actual_eigenvalues = {};
    const bool actual_is_pd = fsb::mat3_posdef_symmetric_eigenvalues(input_mat, actual_eigenvalues);

    REQUIRE(expected_is_pd == actual_is_pd);
    REQUIRE(actual_eigenvalues.x == FsbApprox(expected_eigenvalues.x));
    REQUIRE(actual_eigenvalues.y == FsbApprox(expected_eigenvalues.y));
    REQUIRE(actual_eigenvalues.z == FsbApprox(expected_eigenvalues.z));
}

TEST_CASE("Eigenvectors of 3x3 symmetic positive definite" * doctest::description("[fsb::mat3_posdef_symmetric_eigenvectors]"))
{
    const fsb::Mat3Sym input_mat = {
         1.12, 0.5, 2.0, 0.32, 0.11, 0.125
    };

    const bool expected_is_pd = true;
    const fsb::Vec3 expected_eigenvalues = {0.36123590771763087, 1.2266495577565546, 2.0321145345258147};
    const fsb::Vec3 expected_eigenvec0 = {0.3826966631180945, -0.922792064744923, 0.044699768255297756};
    const fsb::Vec3 expected_eigenvec1 = {0.9102711546680972, 0.3683483036046671, -0.18901310063266208};
    const fsb::Vec3 expected_eigenvec2 = {0.15795470558829355, 0.11302359256087062, 0.9809566649486356};

    fsb::Vec3 actual_eigenvalues = {};
    fsb::Vec3 actual_eigenvec0 = {};
    fsb::Vec3 actual_eigenvec1 = {};
    fsb::Vec3 actual_eigenvec2 = {};
    const bool actual_is_pd = fsb::mat3_posdef_symmetric_eigenvectors(input_mat, actual_eigenvalues, actual_eigenvec0, actual_eigenvec1, actual_eigenvec2);

    REQUIRE(expected_is_pd == actual_is_pd);
    REQUIRE(actual_eigenvalues.x == FsbApprox(expected_eigenvalues.x));
    REQUIRE(actual_eigenvalues.y == FsbApprox(expected_eigenvalues.y));
    REQUIRE(actual_eigenvalues.z == FsbApprox(expected_eigenvalues.z));

    REQUIRE(actual_eigenvec0.x == FsbApprox(expected_eigenvec0.x));
    REQUIRE(actual_eigenvec0.y == FsbApprox(expected_eigenvec0.y));
    REQUIRE(actual_eigenvec0.z == FsbApprox(expected_eigenvec0.z));

    REQUIRE(actual_eigenvec1.x == FsbApprox(expected_eigenvec1.x));
    REQUIRE(actual_eigenvec1.y == FsbApprox(expected_eigenvec1.y));
    REQUIRE(actual_eigenvec1.z == FsbApprox(expected_eigenvec1.z));

    REQUIRE(actual_eigenvec2.x == FsbApprox(expected_eigenvec2.x));
    REQUIRE(actual_eigenvec2.y == FsbApprox(expected_eigenvec2.y));
    REQUIRE(actual_eigenvec2.z == FsbApprox(expected_eigenvec2.z));
}

TEST_SUITE_END();
