
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_circular_buffer.h"

TEST_SUITE_BEGIN("circular_buffer");


TEST_CASE("Create" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 10;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetSize());
    REQUIRE(0 == buffer.GetFilled());

}


TEST_CASE("reset_buffer" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 3;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetSize());
    REQUIRE(0 == buffer.GetFilled());

    fsb::CircularBufferStatus status = buffer.Push(1);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    status = buffer.Push(2);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    status = buffer.Push(3);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(0 == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetFilled());

    buffer.Reset();
    REQUIRE(buf_size == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetSize());
    REQUIRE(0 == buffer.GetFilled());
}


TEST_CASE("fill_buffer" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 3;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetSize());
    REQUIRE(0 == buffer.GetFilled());

    fsb::CircularBufferStatus status = buffer.Push(1);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == buffer.GetRemaining());
    REQUIRE(1 == buffer.GetFilled());

    status = buffer.Push(2);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(1 == buffer.GetRemaining());
    REQUIRE(2 == buffer.GetFilled());

    status = buffer.Push(3);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(0 == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetFilled());

    status = buffer.Push(4);
    REQUIRE(fsb::CircularBufferStatus::FULL == status);
    REQUIRE(0 == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetFilled());

    int popped_value = -1;
    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(1 == popped_value);
    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == popped_value);
    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(3 == popped_value);
}


TEST_CASE("force_fill_buffer" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 3;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetSize());
    REQUIRE(0 == buffer.GetFilled());

    fsb::CircularBufferStatus status = buffer.ForcePush(1);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == buffer.GetRemaining());
    REQUIRE(1 == buffer.GetFilled());

    status = buffer.ForcePush(2);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(1 == buffer.GetRemaining());
    REQUIRE(2 == buffer.GetFilled());

    status = buffer.ForcePush(3);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(0 == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetFilled());

    status = buffer.ForcePush(4);
    REQUIRE(fsb::CircularBufferStatus::OVERWRITE == status);
    REQUIRE(0 == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetFilled());

    int popped_value = -1;
    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == popped_value);
    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(3 == popped_value);
    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(4 == popped_value);
}


TEST_CASE("empty_buffer" * doctest::description("[fsb::CircularBuffer]"))
{
    static const size_t buf_size = 3;
    // global object
    fsb::CircularBuffer<int, buf_size> buffer;
    REQUIRE(buf_size == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetSize());
    REQUIRE(0 == buffer.GetFilled());

    fsb::CircularBufferStatus status = buffer.Push(1);
    status = buffer.Push(2);
    status = buffer.Push(3);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(0 == buffer.GetRemaining());
    REQUIRE(buf_size == buffer.GetFilled());

    int popped_value = -1;
    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(1 == popped_value);
    REQUIRE(1 == buffer.GetRemaining());
    REQUIRE(2 == buffer.GetFilled());

    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(2 == popped_value);
    REQUIRE(2 == buffer.GetRemaining());
    REQUIRE(1 == buffer.GetFilled());

    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::SUCCESS == status);
    REQUIRE(3 == popped_value);
    REQUIRE(buf_size == buffer.GetRemaining());
    REQUIRE(0 == buffer.GetFilled());

    popped_value = -1;
    status = buffer.Pop(popped_value);
    REQUIRE(fsb::CircularBufferStatus::EMPTY == status);
    REQUIRE(-1 == popped_value);
    REQUIRE(buf_size == buffer.GetRemaining());
    REQUIRE(0 == buffer.GetFilled());
}

TEST_SUITE_END();
