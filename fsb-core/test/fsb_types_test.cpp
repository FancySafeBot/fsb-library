
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_types.h"

TEST_SUITE("types") {

// ---------------------------------------------------------------------------
// Span
// ---------------------------------------------------------------------------

TEST_CASE("Span default construction" * doctest::description("[fsb::Span]"))
{
    const fsb::Span<fsb::Real> span{};
    REQUIRE(span.data() == nullptr);
    REQUIRE(span.size() == 0U);
    REQUIRE(span.empty());
}

TEST_CASE("Span construction from pointer and length" * doctest::description("[fsb::Span]"))
{
    fsb::Real buf[4] = {1.0, 2.0, 3.0, 4.0};
    fsb::Span<fsb::Real> span(buf, 4U);

    REQUIRE(span.data() == buf);
    REQUIRE(span.size() == 4U);
    REQUIRE(!span.empty());
}

TEST_CASE("Span element access" * doctest::description("[fsb::Span]"))
{
    fsb::Real buf[3] = {10.0, 20.0, 30.0};
    fsb::Span<fsb::Real> span(buf, 3U);

    REQUIRE(span[0] == FsbApprox(10.0));
    REQUIRE(span[1] == FsbApprox(20.0));
    REQUIRE(span[2] == FsbApprox(30.0));

    span[1] = 99.0;
    REQUIRE(buf[1] == FsbApprox(99.0));
}

TEST_CASE("Span const element access" * doctest::description("[fsb::Span]"))
{
    const fsb::Real buf[3] = {5.0, 6.0, 7.0};
    const fsb::Span<const fsb::Real> span(buf, 3U);

    REQUIRE(span[0] == FsbApprox(5.0));
    REQUIRE(span[1] == FsbApprox(6.0));
    REQUIRE(span[2] == FsbApprox(7.0));
}

TEST_CASE("Span iteration" * doctest::description("[fsb::Span]"))
{
    fsb::Real buf[4] = {1.0, 2.0, 3.0, 4.0};
    fsb::Span<fsb::Real> span(buf, 4U);

    fsb::Real sum = 0.0;
    for (const fsb::Real v : span)
    {
        sum += v;
    }
    REQUIRE(sum == FsbApprox(10.0));
}

TEST_CASE("Span begin and end pointers" * doctest::description("[fsb::Span]"))
{
    fsb::Real buf[2] = {3.0, 4.0};
    fsb::Span<fsb::Real> span(buf, 2U);

    REQUIRE(span.begin() == buf);
    REQUIRE(span.end()   == buf + 2);
}

// ---------------------------------------------------------------------------
// Array::active
// ---------------------------------------------------------------------------

TEST_CASE("Array active span size" * doctest::description("[fsb::Array]"))
{
    fsb::Array<8U> arr = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const fsb::Span<fsb::Real> span = arr.active(3U);

    REQUIRE(span.size() == 3U);
    REQUIRE(span.data() == arr.data());
    REQUIRE(span[0] == FsbApprox(1.0));
    REQUIRE(span[1] == FsbApprox(2.0));
    REQUIRE(span[2] == FsbApprox(3.0));
}

TEST_CASE("Array const active span" * doctest::description("[fsb::Array]"))
{
    const fsb::Array<4U> arr = {7.0, 8.0, 9.0, 0.0};
    const fsb::Span<const fsb::Real> span = arr.active(3U);

    REQUIRE(span.size() == 3U);
    REQUIRE(span[2] == FsbApprox(9.0));
}

TEST_CASE("Array active saturates at MaxDim" * doctest::description("[fsb::Array]"))
{
    fsb::Array<4U> arr = {1.0, 2.0, 3.0, 4.0};
    // request more than max — should clamp to 4
    const fsb::Span<fsb::Real> span = arr.active(99U);
    REQUIRE(span.size() == 4U);
}

} // TEST_SUITE
