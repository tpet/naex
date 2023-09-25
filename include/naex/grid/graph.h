#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <naex/iterators.h>
#include "grid.h"

namespace naex
{
namespace grid
{

typedef CellId VertexId;
typedef CellId EdgeId;

typedef ValueIterator<VertexId> VertexIter;
typedef ValueIterator<EdgeId> EdgeIter;

Cell neighbor4(const Cell& source, int i)
{
    Cell target = source;
    switch (i)
    {
    case 0:
        target.x += 1;
        break;
    case 1:
        target.y += 1;
        break;
    case 2:
        target.x -= 1;
        break;
    case 3:
        target.y -= 1;
        break;
    default:
        assert(false);
    }
    return target;
}

Cell neighbor8(const Cell& source, int i)
{
    Cell target = source;
    switch (i)
    {
    case 0:
        target.x += 1;
        break;
    case 1:
        target.x += 1;
        target.y += 1;
        break;
    case 2:
        target.y += 1;
        break;
    case 3:
        target.x -= 1;
        target.y += 1;
        break;
    case 4:
        target.x -= 1;
        break;
    case 5:
        target.x -= 1;
        target.y -= 1;
        break;
    case 6:
        target.y -= 1;
        break;
    case 7:
        target.x += 1;
        target.y -= 1;
        break;
    default:
        assert(false);
    }
    return target;
}

Cost distance8(int i)
{
    switch (i)
    {
    case 0:
    case 2:
    case 4:
    case 6:
        return 1.0;
    case 1:
    case 3:
    case 5:
    case 7:
        return std::sqrt(2.f);
    default:
        assert(false);
    }
    return 0.0;
}

/** https://www.boost.org/doc/libs/1_75_0/libs/graph/doc/adjacency_list.html */
class Graph
{
public:
    static constexpr Cost INF = std::numeric_limits<Cost>::infinity();

    Graph(const Grid& grid,
          const uint8_t neighborhood = 8,
          const Costs& max_costs = Costs()):
        grid_(grid),
        neighborhood_(neighborhood),
        max_costs_(max_costs)
    {
        assert(neighborhood == 4 || neighborhood == 8);
    }
    Graph():
        Graph(Grid())
    {}
    inline VertexId num_vertices() const
    {
        return grid_.size();
    }
    inline EdgeId num_edges() const
    {
        return neighborhood_ * num_vertices();
    }
    inline std::pair<VertexIter, VertexIter> vertices() const
    {
        return {VertexIter(0), VertexIter(num_vertices())};
    }
    inline std::pair<EdgeIter, EdgeIter> out_edges(const VertexId& u) const
    {
        return {EdgeIter(neighborhood_ * u), EdgeIter(neighborhood_ * (u + 1))};
    }
    inline EdgeId out_degree(const VertexId& u) const
    {
        return neighborhood_;
    }
    inline VertexId source(const EdgeId& e) const
    {
        return e / neighborhood_;        
    }
    inline VertexId target_index(const EdgeId& e) const
    {
        return e % neighborhood_;
    }
    inline VertexId target(const EdgeId& e) const
    {
        // Construct target based on cell coordinates, neighborhood type, and
        // neighbor index. No new cell should be created here.
        auto s = source(e);
        auto cell = grid_.cell(s);
        auto i = target_index(e);
        if (neighborhood_ == 8)
        {
            cell = neighbor8(cell, i);
        }
        else if (neighborhood_ == 4)
        {
            cell = neighbor4(cell, i);
        }

        if (grid_.hasCell(cell))
        {
            return grid_.cellId(cell);
        }
        return s;
    }

    bool costsInBounds(const Costs& costs) const
    {
        for (size_t i = 0; i < 4; ++i)
        {
            // Stop on first invalid max cost.
            if (!std::isfinite(max_costs_[i]))
            {
                break;
            }
            if (!(costs[i] <= max_costs_[i]))
            {
                return false;
            }
        }
        return true;
    }

    inline Cost cost(const EdgeId& e) const
    {
        // Ensure all costs are in bounds if provided.
        const auto& c0 = grid_.costs(source(e));
        if (!costsInBounds(c0))
        {
            return INF;
        }
        const auto& c1 = grid_.costs(target(e));
        if (!costsInBounds(c1))
        {
            return INF;
        }

        auto cost = (c0.total() + c1.total()) / 2.f;

        if (neighborhood_ == 8)
        {
            cost *= distance8(target_index(e));
        }
        return cost;
    }

protected:
    const Grid& grid_;
    const uint8_t neighborhood_;
    const Costs max_costs_;
};

class EdgeCosts
{
public:
    EdgeCosts(const Graph& graph):
        graph_(graph)
    {}
    inline Cost operator[](const EdgeId& e) const
    {
        return graph_.cost(e);
    }
protected:
    const Graph& graph_;
};

}  // namespace grid
}  // namespace naex


using namespace naex::grid;

namespace boost
{
template<>
struct graph_traits<Graph>
{
    typedef VertexId vertex_descriptor;
    typedef VertexId vertices_size_type;
    typedef EdgeId edge_descriptor;
    typedef EdgeId edges_size_type;

    typedef directed_tag directed_category;
    // typedef undirected_tag directed_category;
    // typedef allow_parallel_edge_tag edge_parallel_category;
    typedef disallow_parallel_edge_tag edge_parallel_category;

    typedef bidirectional_traversal_tag traversal_category;
    typedef VertexIter vertex_iterator;
    typedef EdgeIter out_edge_iterator;
};

inline std::pair<VertexIter, VertexIter> vertices(const Graph& g)
{
    return g.vertices();
}

inline VertexId source(EdgeId e, const Graph& g)
{
    return g.source(e);
}

inline VertexId target(EdgeId e, const Graph& g)
{
    return g.target(e);
}

inline std::pair<EdgeIter, EdgeIter> out_edges(VertexId u, const Graph& g)
{
    return g.out_edges(u);
}

/*
inline VertexId num_vertices(const Graph& g)
{
    return g.num_vertices();
}

inline EdgeId out_degree(VertexId u, const Graph& g)
{
    return g.out_degree(u);
}
*/

template<>
class property_traits<naex::grid::EdgeCosts>
{
public:
    typedef EdgeId key_type;
    typedef Cost value_type;
    typedef readable_property_map_tag category;
};

inline Cost get(const EdgeCosts& map, const EdgeId& key)
{
    return map[key];
}

}  // namespace boost

/*
// Include dijkstra header once all used concepts are defined.
// https://groups.google.com/g/boost-developers-archive/c/G2qArovLKzk
// #include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
*/
