
#include <unistd.h>
#include <array>
#include <memory>
#include <thread>

#include <ctime>
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_queue.h"

TEST_SUITE_BEGIN("queue");

TEST_CASE("Queue" * doctest::description("[fsb::Queue]"))
{
    fsb::Queue<int, 5> queue;

    // Test push
    REQUIRE(queue.Push(1) == fsb::QueueStatus::SUCCESS);

    // Test pop
    int value;
    REQUIRE(queue.Pop(value) == fsb::QueueStatus::SUCCESS);
    REQUIRE(value == 1);

    // Test push to full queue
    for (int i = 0; i < 5; ++i)
    {
        REQUIRE(queue.Push(i) == fsb::QueueStatus::SUCCESS);
    }
    REQUIRE(queue.Push(5) == fsb::QueueStatus::FULL);

    // Test force push
    REQUIRE(queue.ForcePush(5) == fsb::QueueStatus::OVERWRITE);

    // Test reset
    queue.Reset();
}

TEST_CASE("Queue Multithreading" * doctest::description("[fsb::Queue]"))
{
    // clock ID for timeout
    clockid_t clock_id = CLOCK_MONOTONIC;
    #ifdef __APPLE__
    clock_id = CLOCK_REALTIME;
    #endif

    constexpr int num_items = 5;
    // shared queue
    auto queue = std::make_shared<fsb::Queue<int, num_items>>();
    auto queue_cons = queue;

    // threads
    std::thread prod_thread = {};
    std::thread cons_thread = {};

    // Producer thread
    prod_thread = std::thread([queue]() {
        for (int i = 0; i < num_items; ++i) {
            const fsb::QueueStatus status = queue->Push(i);
            REQUIRE(status == fsb::QueueStatus::SUCCESS);
        }
    });

    // Consumer thread
    cons_thread = std::thread([queue_cons, clock_id, num_items]() {
        std::array<int, num_items> vals = {};
        size_t total_popped = 0;
        while (total_popped < num_items) {
            // 100 milliseconds timeout
            timespec ts = {};
            clock_gettime(clock_id, &ts);
            ts.tv_nsec += 100 * 1000 * 1000;  // 100 milliseconds
            // wait and pop
            size_t num_popped = 0;
            const fsb::QueueStatus status = queue_cons->PopWait(vals, num_popped, ts);
            REQUIRE(status == fsb::QueueStatus::SUCCESS);
            REQUIRE(num_popped > 0);
            for (size_t j = 0; j < num_popped; ++j)
            {
                REQUIRE(vals[j] == static_cast<int>(total_popped));
                total_popped += 1;
            }
        }
        REQUIRE(total_popped == num_items);
    });

    prod_thread.join();
    cons_thread.join();
}

TEST_SUITE_END();
