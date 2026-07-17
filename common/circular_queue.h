// 循环队列，固定容量，用于维护驿站的待配送任务队列
// 其实用<queue>更好用，但是文档中提到了要实现这个数据结构，就有了这份手写板
#pragma once
#include <vector>
#include <stdexcept>

template <typename T>
class CircularQueue
{
private:
    std::vector<T> data;
    int head, tail;  // head 指向队首，tail 指向下一个可写入位置
    int count;
    int capacity;

public:
    CircularQueue(int cap) : data(cap), head(0), tail(0), count(0), capacity(cap) {}

    bool empty() const { return count == 0; }
    bool full()  const { return count == capacity; }
    int  size()  const { return count; }

    void push(const T& x)
    {
        if (full())
            throw std::runtime_error("CircularQueue::push: queue full");
        data[tail] = x;
        tail = (tail + 1) % capacity;
        ++count;
    }

    T pop()
    {
        if (empty())
            throw std::runtime_error("CircularQueue::pop: queue empty");
        T val = data[head];
        head = (head + 1) % capacity;
        --count;
        return val;
    }

    T& front()
    {
        if (empty())
            throw std::runtime_error("CircularQueue::front: queue empty");
        return data[head];
    }

    const T& front() const
    {
        if (empty())
            throw std::runtime_error("CircularQueue::front: queue empty");
        return data[head];
    }
};