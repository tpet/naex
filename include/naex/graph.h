//
// Created by petrito1 on 11/26/20.
//

#ifndef NAEX_GRAPH_H
#define NAEX_GRAPH_H

//using namespace naex;

namespace boost
{
template<>
struct graph_traits<naex::Graph>
{
    typedef Vertex vertex_descriptor;
    typedef Vertex vertices_size_type;
    typedef Edge edge_descriptor;
    typedef Edge edges_size_type;

    typedef directed_tag directed_category;
    // typedef undirected_tag directed_category;
    // typedef allow_parallel_edge_tag edge_parallel_category;
    typedef disallow_parallel_edge_tag edge_parallel_category;

    typedef bidirectional_traversal_tag traversal_category;
    typedef VertexIter vertex_iterator;
    typedef EdgeIter out_edge_iterator;
};

inline Vertex num_vertices(const Graph& g)
{
    return g.num_vertices();
}

inline std::pair<VertexIter, VertexIter> vertices(const Graph& g)
{
    return g.vertices();
}

inline Vertex source(Edge e, const Graph& g)
{
    return g.source(e);
}

inline Vertex target(Edge e, const Graph& g)
{
    return g.target(e);
}

inline std::pair<EdgeIter, EdgeIter> out_edges(Vertex u, const Graph& g)
{
    return g.out_edges(u);
}

inline Edge out_degree(Vertex u, const Graph& g)
{
    return g.out_degree(u);
}

template<>
class property_traits<EdgeCosts>
{
public:
    typedef Edge key_type;
    typedef Cost value_type;
    typedef readable_property_map_tag category;
};

inline Cost get(const EdgeCosts& map, const Edge& key)
{
    return map[key];
}

}  // namespace boost

// Include dijkstra header once all used concepts are defined.
// https://groups.google.com/g/boost-developers-archive/c/G2qArovLKzk
// #include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>

#endif //NAEX_GRAPH_H
