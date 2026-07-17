// 手写最小堆实现
#pragma once
#include <vector>
#include <stdexcept>
#include <limits>

// 手写最小堆，供 Dijkstra 使用（不用 std::priority_queue）
// 支持 decrease_key 操作，通过 pos 数组 O(1) 定位节点在堆中的位置
class MinHeap
{
public:
    struct HeapNode
    {
        int id;
        double dist;
    };

private:
    std::vector<HeapNode> heap;   // 1-indexed，heap[0] 占位
    std::vector<int> pos;         // pos[node_id] = 在 heap 中的下标，-1 表示不在堆中
    int n;                        // 最大节点数

    void sift_up(int i)
    {
        while (i > 1 && heap[i].dist < heap[i / 2].dist)
        {
            std::swap(pos[heap[i].id], pos[heap[i / 2].id]);
            std::swap(heap[i], heap[i / 2]);
            i /= 2;
        }
    }

    void sift_down(int i)
    {
        int sz = (int)heap.size() - 1;
        while (2 * i <= sz)
        {
            int child = 2 * i;
            if (child + 1 <= sz && heap[child + 1].dist < heap[child].dist)
                ++child;
            if (heap[i].dist <= heap[child].dist)
                break;
            std::swap(pos[heap[i].id], pos[heap[child].id]);
            std::swap(heap[i], heap[child]);
            i = child;
        }
    }

public:
    MinHeap(int max_nodes) : n(max_nodes)
    {
        heap.reserve(max_nodes + 1);
        heap.push_back({ -1, 0.0 }); // 占位
        pos.assign(max_nodes, -1);
    }

    void push(int id, double dist)
    {
        if (pos[id] != -1)
            throw std::runtime_error("MinHeap::push: node already in heap");
        heap.push_back({ id, dist });
        int idx = (int)heap.size() - 1;
        pos[id] = idx;
        sift_up(idx);
    }

    HeapNode top() const
    {
        if (heap.size() <= 1)
            throw std::runtime_error("MinHeap::top: heap empty");
        return heap[1];
    }

    void pop()
    {
        if (heap.size() <= 1)
            throw std::runtime_error("MinHeap::pop: heap empty");
        pos[heap[1].id] = -1;
        if (heap.size() == 2)
        {
            heap.pop_back();
        }
        else
        {
            heap[1] = heap.back();
            pos[heap[1].id] = 1;
            heap.pop_back();
            sift_down(1);
        }
    }

    bool empty() const
    {
        return heap.size() <= 1;
    }

    int size() const
    {
        return (int)heap.size() - 1;
    }

    // 将 id 的距离更新为 new_dist（必须比原值小，即 decrease）
    void decrease_key(int id, double new_dist)
    {
        int idx = pos[id];
        if (idx == -1)
            throw std::runtime_error("MinHeap::decrease_key: node not in heap");
        if (new_dist > heap[idx].dist)
            throw std::runtime_error("MinHeap::decrease_key: new_dist > old_dist");
        heap[idx].dist = new_dist;
        sift_up(idx);
    }

    bool contains(int id) const
    {
        return pos[id] != -1;
    }
};