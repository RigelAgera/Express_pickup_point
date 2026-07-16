#pragma once
#include <vector>
#include <string>
#include <algorithm>

// 邻接表存储，因为边比较稀疏

struct node
{
    int id;
    double x, y;
    bool is_station;
};

struct edge
{
    node u, v;
    double w;
    edge(node a, node b, int c) : u(a), v(b), w(c) {}

};

struct adj_edge
{
    int u, v;   //仅记录id
    double w;
};


class graph
{
    private:
        std::vector<node> nodes;
        std::vector<std::vector<adj_edge>> adj;
        int n, m;
    public:
        graph() : n(0), m(0) {}
        graph(int a, int b) : n(a), m(b), nodes(a), adj(a) {}
        void set_node(int id, double x, double y, bool is_station)
        {
            nodes[id] = {id, x, y, is_station};
        }
        void add_edges(int u, int v, double w)
        {
            adj[u].push_back({u, v, w});
            adj[v].push_back({v, u, w});
        }
        const std::vector<node>& get_nodes() const { return nodes; }
        const std::vector<std::vector<adj_edge>>& get_adj() const { return adj; }
        int get_n() const { return n; }
        int get_m() const { return m; }
};