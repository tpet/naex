
#ifndef NAEX_NN_SEARCH_H
#define NAEX_NN_SEARCH_H

#include <flann/flann.hpp>
#include <vector>

namespace naex
{
template<typename T>
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
        params.cores = 0;
        if (radius < std::numeric_limits<T>::infinity())
        {
            params.max_neighbors = k;
            index.radiusSearch(queries, nn_, dist_, radius, params);
            // Limit
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

}  // namespace naex

#endif //NAEX_NN_SEARCH_H
