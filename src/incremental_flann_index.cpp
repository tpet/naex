#include <naex/types.h>
#include <naex/nn_search.h>
// Summary:
// Indices of removed points are not reused.
// Pointer of removed point is nulled.
// veclen() gives point dimensions.
// size() gives number of points.
// size() doesn't correspond to next point index once some points are removed.

using namespace naex;

namespace
{
    template<typename T>
    std::string format3(const T* data, const std::string& delim = " ")
    {
        std::stringstream ss;
        if (data)
        {
            ss
                << data[0] << delim
                << data[1] << delim
                << data[2];
        }
        else
        {
            ss << "null";
        }
        return ss.str();
    }
}

int main (int argc, char *argv[])
{
    MatX mat1(3, 2);
    mat1 << 1., 1.,
            1., 2.,
            0., 0.;

    MatX mat2(3, 2);
    mat2 << 2., 2.,
            1., 2.,
            0., 0.;

    MatX mat3(3, 2);
    mat3 << 3., 3.,
            1., 2.,
            0., 0.;

    std::cout << "mat1:" << std::endl << mat1 << std::endl;
    std::cout << "mat2:" << std::endl << mat2 << std::endl;
    std::cout << "mat3:" << std::endl << mat3 << std::endl;

    FlannMat fmat1(mat1.data(), 2, 3);
    FlannMat fmat2(mat2.data(), 2, 3);
    FlannMat fmat3(mat3.data(), 2, 3);

    flann::KDTreeSingleIndexParams index_params;
    // Assertion with veclen fails on addPoints if index initialized empty.
    // FlannIndex index(index_params);
    // index.addPoints(fmat1);
    FlannIndex index(fmat1, index_params);
    std::cout << "index.veclen(): " << index.veclen() << std::endl;
    std::cout << "index.size(): " << index.size() << std::endl;

    index.addPoints(fmat2);
    std::cout << "index.veclen(): " << index.veclen() << std::endl;
    std::cout << "index.size(): " << index.size() << std::endl;

    index.removePoint(3);
    std::cout << "Point 3 removed." << std::endl;
    std::cout << "index.veclen(): " << index.veclen() << std::endl;
    std::cout << "index.size(): " << index.size() << std::endl;

    index.addPoints(fmat3);
    std::cout << "index.veclen(): " << index.veclen() << std::endl;
    std::cout << "index.size(): " << index.size() << std::endl;

    for (size_t i = 0; i < 6; ++i)
    {
        std::cout
            << "Point " << i << ": "
            << format3(index.getPoint(i))
            << std::endl;
    }

    auto query = fmat2;
    size_t k = index.size();
    Query<Elem> knn_query(index, query, int(k));
    for (size_t i = 0; i < query.rows; ++i)
    {
        for (size_t j = 0; j < k; ++j)
        {
            std::cout
                << format3(query[i]) << " "
                << j + 1 << "th neighbor: id "
                << knn_query.nn_[i][j] << ": "
                << format3(index.getPoint(size_t(knn_query.nn_[i][j])))
                << ", dist: " << knn_query.dist_[i][j]
                << std::endl;
        }
    }

    // Radius search with k limit (int / float).
    std::cout << std::endl << "Radius search with neighbor limit." << std::endl;
    Buffer<int> nn_buf(query.rows * k);
    Buffer<Elem> dist_buf(query.rows * k);
    flann::Matrix<int> nn(nn_buf.begin(), query.rows, k);
    flann::Matrix<Elem> dist(dist_buf.begin(), query.rows, k);
    // Initial NN doesn't seem to be used at all.
    // -1 is set to last valid within radius, (apparently) anything after that.
    std::fill(nn_buf.begin(), nn_buf.end(), -1);
    // Infinite distance used at last valid, initial (anything?) after that.
    // std::fill(dist_buf.begin(), dist_buf.end(), std::numeric_limits<Elem>::infinity());
    std::fill(dist_buf.begin(), dist_buf.end(), std::numeric_limits<Elem>::quiet_NaN());

    flann::SearchParams params;
    params.cores = 0;
    params.max_neighbors = int(k);
    Elem radius = 1.5;
    index.radiusSearch(query, nn, dist, radius, params);

    for (size_t i = 0; i < query.rows; ++i)
    {
        for (size_t j = 0; j < k; ++j)
        {
            Elem* pt = std::isfinite(dist[i][j])
                    ? index.getPoint(size_t(nn[i][j]))
                    : nullptr;
            std::cout
                << format3(query[i]) << " "
                << j + 1 << "th neighbor: id "
                << nn[i][j] << ": "
                << format3(pt)
                << ", dist: " << dist[i][j]
                << std::endl;
        }
    }
}
