#include <gtest/gtest.h>
#include "triple_buffer.h"
#include <thread>
#include <atomic>
#include <cstring>

struct TestData {
    int value;
    double array[3];
};

// ---------------------------------------------------------------------------
// 1. InitialState_HasNoData
// ---------------------------------------------------------------------------
TEST(TripleBufferTest, InitialState_HasNoData) {
    TripleBuffer<TestData> buf;
    EXPECT_FALSE(buf.hasData());
}

// ---------------------------------------------------------------------------
// 2. WriteAndPublish_HasData
// ---------------------------------------------------------------------------
TEST(TripleBufferTest, WriteAndPublish_HasData) {
    TripleBuffer<TestData> buf;

    auto& wb = buf.writeBuffer();
    wb.value = 42;
    buf.publish();

    EXPECT_TRUE(buf.hasData());
}

// ---------------------------------------------------------------------------
// 3. Read_ReturnsPublishedData
// ---------------------------------------------------------------------------
TEST(TripleBufferTest, Read_ReturnsPublishedData) {
    TripleBuffer<TestData> buf;

    auto& wb = buf.writeBuffer();
    wb.value = 99;
    wb.array[0] = 1.0;
    wb.array[1] = 2.0;
    wb.array[2] = 3.0;
    buf.publish();

    const auto& rd = buf.read();
    EXPECT_EQ(rd.value, 99);
    EXPECT_DOUBLE_EQ(rd.array[0], 1.0);
    EXPECT_DOUBLE_EQ(rd.array[1], 2.0);
    EXPECT_DOUBLE_EQ(rd.array[2], 3.0);
}

// ---------------------------------------------------------------------------
// 4. MultiplePublishes_ReadReturnsLatest
// ---------------------------------------------------------------------------
TEST(TripleBufferTest, MultiplePublishes_ReadReturnsLatest) {
    TripleBuffer<TestData> buf;

    // Publish first value
    auto& wb1 = buf.writeBuffer();
    wb1.value = 10;
    buf.publish();

    // Publish second value
    auto& wb2 = buf.writeBuffer();
    wb2.value = 20;
    buf.publish();

    // Publish third value
    auto& wb3 = buf.writeBuffer();
    wb3.value = 30;
    buf.publish();

    // Reader should get the latest
    const auto& rd = buf.read();
    EXPECT_EQ(rd.value, 30);
}

// ---------------------------------------------------------------------------
// 5. ReadDoesNotBlockWrite
// ---------------------------------------------------------------------------
TEST(TripleBufferTest, ReadDoesNotBlockWrite) {
    TripleBuffer<TestData> buf;

    // Write and publish
    auto& wb1 = buf.writeBuffer();
    wb1.value = 1;
    buf.publish();

    // Read
    const auto& rd1 = buf.read();
    EXPECT_EQ(rd1.value, 1);

    // Write again while reader has a reference
    auto& wb2 = buf.writeBuffer();
    wb2.value = 2;
    buf.publish();

    // Previous read reference should still be valid (rd1.value == 1)
    EXPECT_EQ(rd1.value, 1);

    // New read should return 2
    const auto& rd2 = buf.read();
    EXPECT_EQ(rd2.value, 2);
}

// ---------------------------------------------------------------------------
// 6. SnapshotNeverTornRead (multithreaded stress test)
// ---------------------------------------------------------------------------
TEST(TripleBufferTest, SnapshotNeverTornRead) {
    struct BigData {
        int sequence;
        int values[100];
    };

    TripleBuffer<BigData> buf;
    std::atomic<bool> running{true};
    std::atomic<int> torn_count{0};

    // Writer thread: continuously publish incrementing sequences
    std::thread writer([&]() {
        int seq = 0;
        while (running.load(std::memory_order_relaxed)) {
            auto& wb = buf.writeBuffer();
            wb.sequence = seq;
            for (int i = 0; i < 100; i++) {
                wb.values[i] = seq;
            }
            buf.publish();
            seq++;
        }
    });

    // Reader thread: read and verify consistency
    std::thread reader([&]() {
        int reads = 0;
        while (reads < 10000 && running.load(std::memory_order_relaxed)) {
            if (!buf.hasData()) continue;

            const auto& rd = buf.read();
            int expected_seq = rd.sequence;
            for (int i = 0; i < 100; i++) {
                if (rd.values[i] != expected_seq) {
                    torn_count.fetch_add(1, std::memory_order_relaxed);
                    break;
                }
            }
            reads++;
        }
    });

    // Let it run for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    running.store(false, std::memory_order_relaxed);

    writer.join();
    reader.join();

    EXPECT_EQ(torn_count.load(), 0) << "Detected torn reads in triple buffer";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
