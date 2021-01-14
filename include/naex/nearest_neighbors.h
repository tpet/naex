
#ifndef NAEX_NEAREST_NEIGHBORS_H
#define NAEX_NEAREST_NEIGHBORS_H

#include <flann/flann.hpp>
#include <naex/array.h>
#include <unordered_set>
#include <vector>

namespace naex
{

template<typename I>
I invalid_index();

template<>
int invalid_index<int>()
{
    return -1;
}

template<>
size_t invalid_index<size_t>()
{
    return std::numeric_limits<size_t>::max();
}

bool invalid_index(int i)
{
    return i < 0;
}

bool invalid_index(size_t i)
{
    return i == std::numeric_limits<size_t>::max();
}

template<typename V>
V invalid_distance();

template<>
float invalid_distance<float>()
{
    return std::numeric_limits<float>::infinity();
}

template<>
double invalid_distance<double>()
{
    return std::numeric_limits<double>::infinity();
}

template<>
long invalid_distance<long>()
{
    return 0L;
}

bool invalid_distance(float d)
{
    return std::isinf(d);
}

bool invalid_distance(double d)
{
    return std::isinf(d);
}

bool invalid_distance(long d)
{
    return d == 0L;
}

template<typename I, typename V>
class NearestNeighborTraits
{
public:
    I invalid_index()
    {
        return naex::invalid_index<I>();
    }
    V invalid_distance()
    {
        return naex::invalid_distance<V>();
    }
};

template<typename I, typename V>
class NearestNeighbors
{
public:
    NearestNeighbors(I n):
        nn_(n),
        dist_(n)
    {}
    std::vector<std::vector<I>> nn_;
    std::vector<std::vector<V>> dist_;
};

template<typename I, typename V>
class KNearestNeighbors
{
public:
    KNearestNeighbors(I n, I k):
        nn_buf_(n * k),
        dist_buf_(n * k),
        nn_(nn_buf_.begin(), n, k),
        dist_(dist_buf_.begin(), n, k)
    {}
//    Buffer<I> nn_buf_;
//    Buffer<T> dist_buf_;
    std::vector<I> nn_buf_;
    std::vector<V> dist_buf_;
    flann::Matrix<I> nn_;
    flann::Matrix<V> dist_;
};

template<typename T>
//template<typename Value, typename Index>
class Query
{
public:
    Query(const flann::Index<flann::L2_3D<T>>& index,
          const flann::Matrix<T>& queries,
          const int k = 1,
          const T radius = std::numeric_limits<T>::infinity()) :
        nn_buf_(queries.rows * k),
        dist_buf_(queries.rows * k),
        nn_(nn_buf_.begin(), queries.rows, k),
        dist_(dist_buf_.begin(), queries.rows, k)
    {
        flann::SearchParams params;
        params.checks = 64;
        params.cores = 0;
        params.sorted = true;
        if (radius < std::numeric_limits<T>::infinity())
        {
            params.max_neighbors = k;
            index.radiusSearch(queries, nn_, dist_, radius, params);
        }
        else
        {
            index.knnSearch(queries, nn_, dist_, k, params);
        }
    }
    Buffer<int> nn_buf_;
    Buffer<T> dist_buf_;
    flann::Matrix<int> nn_;
    flann::Matrix<T> dist_;
};

template<typename T>
class RadiusQuery
{
public:
    RadiusQuery(const flann::Index<flann::L2_3D<T>>& index,
                const flann::Matrix<T>& queries,
                const T radius) :
        nn_(queries.rows),
        dist_(queries.rows)
    {
        flann::SearchParams params;
        params.cores = 0;
        index.radiusSearch(queries, nn_, dist_, radius, params);
    }
    std::vector<std::vector<int>> nn_;
    std::vector<std::vector<T>> dist_;
};

//    template<typename T>
//    Query<T> query(const flann::Index<flann::L2_3D<T>>& index, const flann::Matrix<T>& queries, int k = 1)
//    {
//        return Query<T>(index, queries, k);
//    }

/*
//    template<typename V, typename T>
class KNearestNeighborGraph
{
public:
    KNearestNeighborGraph(Index k)
    //:
//        nn_array_(k)
    {
    }
    void add_points(const FlannMat& points);
    Value* get_point();
    // To mark neighboring points for update, NN should be symmetric.
    // This may be done outside?
    Value* remove_point(Index i);
    void update_dirty();

//    Query knn(const FlannMat& points)
//    {
//
//    }

//        std::map<Buffer<Type>> x_buf_;
//        std::vector<flann::Matrix<Type>> x_;
//    flann::Index index_;  // FLANN point index.
//        Buffer<Index> nn_buf_;
//        flann::Matrix<Index> nn_;  // N-by-K index array, K nearest neighbors for each of N points.
//        Buffer<bool> nn_dirty_;
    std::unordered_set<Index> dirty_nn_;
    Buffer<Index> nn_count_;
//        Array<1, Index> nn_count_;  // N-by-1, denotes number of valid nearest neighbors.
    Array<2, Index> nn_array_;  // N-by-K, K nearest neighbors for each of N points.
//        sensor_msgs::PointCloud2 meta_c
};
*/

}  // namespace naex

#endif //NAEX_NEAREST_NEIGHBORS_H
