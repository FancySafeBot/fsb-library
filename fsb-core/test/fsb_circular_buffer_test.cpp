
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_circular_buffer.h"

TEST_SUITE("circular_buffer") {


TEST_CASE("Create" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 10;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_size());
    REQUIRE(0 == buffer.get_filled());

}


TEST_CASE("reset_buffer" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 3;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_size());
    REQUIRE(0 == buffer.get_filled());

    fsb::CircularBufferStatus status = buffer.push(1);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    status = buffer.push(2);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    status = buffer.push(3);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(0 == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_filled());

    buffer.reset();
    REQUIRE(buf_size == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_size());
    REQUIRE(0 == buffer.get_filled());
}


TEST_CASE("fill_buffer" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 3;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_size());
    REQUIRE(0 == buffer.get_filled());

    fsb::CircularBufferStatus status = buffer.push(1);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == buffer.get_remaining());
    REQUIRE(1 == buffer.get_filled());

    status = buffer.push(2);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(1 == buffer.get_remaining());
    REQUIRE(2 == buffer.get_filled());

    status = buffer.push(3);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(0 == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_filled());

    status = buffer.push(4);
    REQUIRE(fsb::CircularBufferStatus::FULL == status);
    REQUIRE(0 == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_filled());

    int popped_value = -1;
    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(1 == popped_value);
    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == popped_value);
    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(3 == popped_value);
}


TEST_CASE("force_fill_buffer" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 3;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_size());
    REQUIRE(0 == buffer.get_filled());

    fsb::CircularBufferStatus status = buffer.force_push(1);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == buffer.get_remaining());
    REQUIRE(1 == buffer.get_filled());

    status = buffer.force_push(2);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(1 == buffer.get_remaining());
    REQUIRE(2 == buffer.get_filled());

    status = buffer.force_push(3);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(0 == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_filled());

    status = buffer.force_push(4);
    REQUIRE(fsb::CircularBufferStatus::OVERWRITE == status);
    REQUIRE(0 == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_filled());

    int popped_value = -1;
    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == popped_value);
    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(3 == popped_value);
    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(4 == popped_value);
}


TEST_CASE("empty_buffer" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 3;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_size());
    REQUIRE(0 == buffer.get_filled());

    fsb::CircularBufferStatus status = buffer.push(1);
    status = buffer.push(2);
    status = buffer.push(3);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(0 == buffer.get_remaining());
    REQUIRE(buf_size == buffer.get_filled());

    int popped_value = -1;
    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(1 == popped_value);
    REQUIRE(1 == buffer.get_remaining());
    REQUIRE(2 == buffer.get_filled());

    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == popped_value);
    REQUIRE(2 == buffer.get_remaining());
    REQUIRE(1 == buffer.get_filled());

    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(3 == popped_value);
    REQUIRE(buf_size == buffer.get_remaining());
    REQUIRE(0 == buffer.get_filled());

    popped_value = -1;
    status = buffer.pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::EMPTY == status);
    REQUIRE(-1 == popped_value);
    REQUIRE(buf_size == buffer.get_remaining());
    REQUIRE(0 == buffer.get_filled());
}

} // TEST_SUITE
