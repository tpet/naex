#include <naex/types.h>
#include <naex/nearest_neighbors.h>
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
    // Compiles:
    //     int, float or double  (invalid: -1, inf)
    //     int, long  (invalid: -1, 0)
    //     size_t, float or double  (invalid: max, inf)
    //     size_t, long (invalid: max, 0)
    // Doesn't compile:
    //     int, int
    //     uint32_t, int
    typedef int I;
//    typedef uint32_t I;
//    typedef size_t I;
    // long compiles, int not
//    typedef float T;
//    typedef float D;
//    typedef double T;
//    typedef double D;
//    typedef int T;
//    typedef int D;
    typedef long T;
    typedef long D;
    typedef Eigen::Matrix<I, 3, Eigen::Dynamic, Eigen::DontAlign> IMat;
    typedef Eigen::Matrix<T, 3, Eigen::Dynamic, Eigen::DontAlign> TMat;
    typedef Eigen::Matrix<D, 3, Eigen::Dynamic, Eigen::DontAlign> DMat;
    typedef flann::Matrix<I> FlannIMat;
    typedef flann::Matrix<T> FlannTMat;
    typedef flann::Matrix<D> FlannDMat;
    typedef flann::Index<flann::L2_3D<D>> FlannDIndex;

    TMat mat1(3, 2);
    mat1 << 1, 1,
            1, 2,
            1, 1;

    TMat mat2(3, 2);
    mat2 << 2, 2,
            1, 2,
            2, 2;

    TMat mat3(3, 2);
    mat3 << 3, 3,
            1, 2,
            3, 3;

    std::cout << "mat1:" << std::endl << mat1 << std::endl;
    std::cout << "mat2:" << std::endl << mat2 << std::endl;
    std::cout << "mat3:" << std::endl << mat3 << std::endl;

    FlannTMat fmat1(mat1.data(), 2, 3);
    FlannTMat fmat2(mat2.data(), 2, 3);
    FlannTMat fmat3(mat3.data(), 2, 3);

    flann::KDTreeSingleIndexParams index_params;
    // Assertion with veclen fails on addPoints if index initialized empty.
    // FlannIndex index(index_params);
    // index.addPoints(fmat1);
    FlannDIndex index(fmat1, index_params);
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

    // Radius search with k limit (int / float).
    std::cout << std::endl << "Radius search with neighbor limit." << std::endl;
    std::cout << "Max index: " << std::numeric_limits<I>::max() << std::endl;
    std::cout << "Max value: " << std::numeric_limits<T>::max() << std::endl;
    std::cout << "Max distance: " << std::numeric_limits<D>::max() << std::endl;
    Buffer<I> nn_buf(query.rows * k);
    Buffer<D> dist_buf(query.rows * k);
    FlannIMat nn(nn_buf.begin(), query.rows, k);
    FlannDMat dist(dist_buf.begin(), query.rows, k);
    // Initial NN doesn't seem to be used at all.
    // -1 is set to last valid within radius, (apparently) anything after that.
//    std::fill(nn_buf.begin(), nn_buf.end(), std::numeric_limits<I>::max());
    std::fill(nn_buf.begin(), nn_buf.end(), 100);
    // Infinite distance used at last valid, initial (anything?) after that.
    // std::fill(dist_buf.begin(), dist_buf.end(), std::numeric_limits<Elem>::infinity());
//    std::fill(dist_buf.begin(), dist_buf.end(), std::numeric_limits<D>::max());
    std::fill(dist_buf.begin(), dist_buf.end(), 100);

    flann::SearchParams params;
    params.cores = 0;
    params.max_neighbors = int(k);
    D radius = 3;
    index.radiusSearch(query, nn, dist, radius, params);

    for (size_t i = 0; i < query.rows; ++i)
    {
        for (size_t j = 0; j < k; ++j)
        {
            T* pt = dist[i][j] != std::numeric_limits<D>::max()
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
