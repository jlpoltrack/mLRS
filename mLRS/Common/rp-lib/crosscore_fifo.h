//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// cross-core lockless SPSC FIFO
// safe for one core writing, one core reading
//*******************************************************
#ifndef CROSSCORE_FIFO_H
#define CROSSCORE_FIFO_H
#pragma once

#include <cstdint>
#include <cstring>


// lockless single-producer single-consumer FIFO
// SIZE must be a power of 2
template<typename T, uint16_t SIZE>
class tCrossCoreFifo
{
    static_assert((SIZE & (SIZE - 1)) == 0, "SIZE must be a power of 2");
    static constexpr uint16_t MASK = SIZE - 1;

  public:
    void Init(void) {
        head = 0;
        tail = 0;
    }

    // writer side
    bool Put(T c) {
        uint16_t next = (head + 1) & MASK;
        if (next == tail) return false;  // full
        buf[head] = c;
        __dmb();  // arm data memory barrier
        head = next;
        return true;
    }

    // bulk copy with single memory barrier
    uint16_t PutBuf(const T* data, uint16_t len) {
        uint16_t h = head;
        uint16_t t = tail;
        uint16_t free = (t > h) ? (t - h - 1) : (SIZE - h + t - 1);
        if (len > free) len = free;
        if (len == 0) return 0;

        // copy in up to two chunks (wrap-around)
        uint16_t to_end = SIZE - h;
        uint16_t first = (len <= to_end) ? len : to_end;
        memcpy(&buf[h], data, first * sizeof(T));
        if (first < len) {
            memcpy(&buf[0], data + first, (len - first) * sizeof(T));
        }
        __dmb();  // single barrier after all writes
        head = (h + len) & MASK;
        return len;
    }

    bool HasSpace(uint16_t count) {
        uint16_t h = head;
        uint16_t t = tail;
        uint16_t used = (h - t) & MASK;
        return (SIZE - 1 - used) >= count;
    }

    // reader side
    bool Available(void) {
        return head != tail;
    }

    uint16_t AvailableCount(void) {
        return (head - tail) & MASK;
    }

    T Get(void) {
        if (head == tail) return 0;
        T c = buf[tail];
        __dmb();
        tail = (tail + 1) & MASK;
        return c;
    }

    // bulk read with single memory barrier
    uint16_t GetBuf(T* data, uint16_t len) {
        uint16_t h = head;
        uint16_t t = tail;
        uint16_t avail = (h - t) & MASK;
        if (len > avail) len = avail;
        if (len == 0) return 0;

        // read in up to two chunks (wrap-around)
        uint16_t to_end = SIZE - t;
        uint16_t first = (len <= to_end) ? len : to_end;
        memcpy(data, &buf[t], first * sizeof(T));
        if (first < len) {
            memcpy(data + first, &buf[0], (len - first) * sizeof(T));
        }
        __dmb();  // single barrier after all reads
        tail = (t + len) & MASK;
        return len;
    }

    void Flush(void) {
        tail = head;
    }

  private:
    volatile uint16_t head;
    volatile uint16_t tail;
    T buf[SIZE];
};


#endif // CROSSCORE_FIFO_H
