
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_work.h"

TEST_SUITE("work") {

TEST_CASE("allocate_deallocate_basic" * doctest::description("[fsb::WorkArray]"))
{
    fsb::WorkArray<int, 8U> work;

    REQUIRE(0U == work.get_used());
    REQUIRE(8U == work.get_remaining());

    fsb::WorkBlock<int> a;
    auto status = work.allocate(3U, a);
    REQUIRE(fsb::WorkArrayStatus::SUCCESS == status);
    REQUIRE(a);
    REQUIRE(3U == a.size);
    REQUIRE(work.data() == a.data);
    REQUIRE(0U == a.offset);
    REQUIRE(3U == work.get_used());
    REQUIRE(5U == work.get_remaining());

    fsb::WorkBlock<int> b;
    status = work.allocate(5U, b);
    REQUIRE(fsb::WorkArrayStatus::SUCCESS == status);
    REQUIRE(b);
    REQUIRE(5U == b.size);
    REQUIRE((work.data() + 3U) == b.data);
    REQUIRE(3U == b.offset);
    REQUIRE(8U == work.get_used());
    REQUIRE(0U == work.get_remaining());

    fsb::WorkBlock<int> c;
    status = work.allocate(1U, c);
    REQUIRE(fsb::WorkArrayStatus::FULL == status);
    REQUIRE(!c);

    // Strict LIFO: cannot free 'a' before 'b'
    status = work.deallocate(a);
    REQUIRE(fsb::WorkArrayStatus::OUT_OF_ORDER == status);
    REQUIRE(8U == work.get_used());
    REQUIRE(0U == work.get_remaining());

    status = work.deallocate(b.data, b.size);
    REQUIRE(fsb::WorkArrayStatus::SUCCESS == status);

    status = work.deallocate(a);
    REQUIRE(fsb::WorkArrayStatus::SUCCESS == status);

    REQUIRE(0U == work.get_used());
    REQUIRE(8U == work.get_remaining());
}

TEST_CASE("invalid_args" * doctest::description("[fsb::WorkArray]"))
{
    fsb::WorkArray<int, 4U> work;

    fsb::WorkBlock<int> block;
    auto status = work.allocate(0U, block);
    REQUIRE(fsb::WorkArrayStatus::INVALID_ARGUMENT == status);

    status = work.allocate(5U, block);
    REQUIRE(fsb::WorkArrayStatus::INVALID_ARGUMENT == status);

    status = work.deallocate(nullptr, 1U);
    REQUIRE(fsb::WorkArrayStatus::INVALID_ARGUMENT == status);

    status = work.deallocate(work.data(), 0U);
    REQUIRE(fsb::WorkArrayStatus::INVALID_ARGUMENT == status);
}

TEST_CASE("lifo_and_marker_restore" * doctest::description("[fsb::WorkArray]"))
{
    fsb::WorkArray<int, 8U> work;

    fsb::WorkBlock<int> a;
    fsb::WorkBlock<int> b;

    auto status = work.allocate(2U, a);
    REQUIRE(fsb::WorkArrayStatus::SUCCESS == status);
    REQUIRE(work.data() == a.data);

    const auto marker = work.get_marker();

    status = work.allocate(2U, b);
    REQUIRE(fsb::WorkArrayStatus::SUCCESS == status);
    REQUIRE((work.data() + 2U) == b.data);

    // Out-of-order free rejected
    status = work.deallocate(a);
    REQUIRE(fsb::WorkArrayStatus::OUT_OF_ORDER == status);

    // Restore marker should bulk-free allocations after marker.
    status = work.restore(marker);
    REQUIRE(fsb::WorkArrayStatus::SUCCESS == status);
    REQUIRE(2U == work.get_used());

    // After restore, we can free 'a'
    status = work.deallocate(a);
    REQUIRE(fsb::WorkArrayStatus::SUCCESS == status);

    REQUIRE(0U == work.get_used());
    REQUIRE(8U == work.get_remaining());
}

} // TEST_SUITE
