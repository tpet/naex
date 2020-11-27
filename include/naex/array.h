#ifndef NAEX_ARRAY_H
#define NAEX_ARRAY_H

#include <cstdarg>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <naex/buffer.h>



namespace naex
{

template<size_t D, typename T, typename B = T>
class Array
{
public:
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> EigenMatrix;
    typedef Eigen::Map<EigenMatrix> EigenMatrixView;

//    static size_t to_buffer(size_t i)
//    {
//        return i * sizeof(T) / sizeof(T);
//    }
    Array(size_t dim_0):
        size_{dim_0},
        stride_{sizeof(T)},
//        data_(dim_0)
        data_(numel())
//        data_(dim_0 * sizeof(T) / sizeof(B) + (dim_0 * sizeof(T) % sizeof(B) != 0))
    {
        static_assert(D == 1, "D must be 1.");
//        size_[0] = dim_0;
    }
    Array(size_t dim_0, size_t dim_1):
        size_{dim_0, dim_1},
        stride_{sizeof(T) * dim_1, sizeof(T)},
//        data_(dim_0 * dim_1)
        data_(numel())
    {
        static_assert(D == 2, "D must be 2.");
//        size_[0] = dim_0;
//        size_[1] = dim_1;
    }
    Array(size_t dim_0, size_t dim_1, size_t dim_2):
        size_{dim_0, dim_1, dim_2},
        stride_{sizeof(T) * dim_2 * dim_1, sizeof(T) * dim_2, sizeof(T)},
//        data_(dim_0 * dim_1 * dim_2)
        data_(numel())
    {
        static_assert(D == 3, "D must be 3.");
//        size_[0] = dim_0;
//        size_[1] = dim_1;
//        size_[1] = dim_1;
    }
    size_t numel() const
    {
        size_t n = 1;
        for (size_t dim = 0; dim < D; ++dim)
        {
            n *= size_[dim];
        }
        return n;
    }
    flann::Matrix<T> flann_matrix_view()
    {
        static_assert(D == 2, "D must be 2.");
        return flann::Matrix<T>(data_.data(), size_[0], size_[1]);
    }
    EigenMatrixView eigen_matrix_view()
    {
        static_assert(D == 2, "D must be 2.");
        return EigenMatrixView(data_.data(), size_[0], size_[1]);
    }
//    EigenMatrixView eigen_matrix3x_view()
//    {
//        static_assert(D == 2, "D must be 2.");
//        assert(size_[0] == 3);
//        return EigenMatrix3XView(data_.data(), size_[0], size_[1]);
//    }
    size_t linear(size_t i_0) const
    {
        return i_0 * stride_[0];
    }
    size_t linear(size_t i_0, size_t i_1) const
    {
        return i_0 * stride_[0] + i_1 * stride_[1];
    }
    size_t linear(size_t i_0, size_t i_1, size_t i_2) const
    {
        return i_0 * stride_[0] + i_1 * stride_[1] + i_2 * stride_[2];
    }
    size_t linear(size_t i_0, ...) const
    {
        std::va_list args;
        va_start(args, i_0);
        i_0 *= stride_[0];
        for (size_t dim = 1; dim < D; ++dim)
        {
            i_0 += va_arg(args, size_t) * stride_[dim];
        }
        return i_0;
    }
    T& value(size_t i)
    {
        return reinterpret_cast<T&>(reinterpret_cast<uint8_t*>(data_.begin())[i]);
    }
    const T& value(size_t i) const
    {
        return reinterpret_cast<T&>(reinterpret_cast<const uint8_t*>(data_.begin())[i]);
    }
    T& operator[](size_t i_0)
    {
        return value(linear(i_0));
    }
//    T& operator[](size_t i_0, size_t i_1)
//    {
//        return value(linear(i_0, i_1));
//    }
//    T& operator[](size_t i_0, size_t i_1, size_t i_2)
//    {
//        return value(linear(i_0, i_1, i_2));
//    }
    T& operator()(size_t i_0)
    {
        return value(linear(i_0));
    }
    const T& operator()(size_t i_0) const
    {
        return value(linear(i_0));
    }
    T& operator()(size_t i_0, size_t i_1)
    {
        return value(linear(i_0, i_1));
    }
    const T& operator()(size_t i_0, size_t i_1) const
    {
        return value(linear(i_0, i_1));
    }
    T& operator()(size_t i_0, size_t i_1, size_t i_2)
    {
        return value(linear(i_0, i_1, i_2));
    }
    const T& operator()(size_t i_0, size_t i_1, size_t i_2) const
    {
        return value(linear(i_0, i_1, i_2));
    }
public:
    size_t size_[D];
    size_t stride_[D];  // Stride in bytes
    Buffer<T> data_;  // Must come after size_ to use numel() in initializer.
};

typedef Array<2, int8_t> Array2Int8;
typedef Array<2, uint8_t> Array2UInt8;
typedef Array<2, int16_t> Array2Int16;
typedef Array<2, uint16_t> Array2UInt16;
typedef Array<2, int32_t> Array2Int32;
typedef Array<2, uint32_t> Array2UInt32;
typedef Array<2, int64_t> Array2Int64;
typedef Array<2, uint64_t> Array2UInt64;
typedef Array<2, float> Array2Float32;
typedef Array<2, double> Array2Float64;

//template<>
//template<typename T>
//class Array<2, T>
//{
//    Array(size_t dim_0, size_t dim_1):
//        data_(dim_0 * dim_1)
//    {
//        size_[0] = dim_0;
//        size_[1] = dim_1;
//    }
//};

//template<typename T>
//Array<2, T>::Array(
//    T* data,
//    size_t dim_0, size_t dim_1,
//    size_t stride_0 = 0, size_t stride_1 = 0):
//
//    data_(data),
//    size_{dim_0, dim_1},
//    stride_{stride_0 ? stride_0 : sizeof(T) * dim_1,
//            stride_1 ? stride_1 : sizeof(T)}
//{
//    size_[0] = dim_0;
//    size_[1] = dim_1;
//    stride_[0] = stride_0 ? stride_0 : sizeof(T) * dim_1;
//    stride_[1] = stride_1 ? stride_1 : sizeof(T);
//}

//template<typename T>
//Array<2, T>::Array(size_t dim_0, size_t dim_1):
//
//    data_(dim_0 * dim_1),
//    size_{dim_0, dim_1},
//    stride_{sizeof(T) * dim_1, sizeof(T)}
//{
//    size_[0] = dim_0;
//    size_[1] = dim_1;
//    stride_[0] = stride_0 ? stride_0 : sizeof(T) * dim_1;
//    stride_[1] = stride_1 ? stride_1 : sizeof(T);
//}

//template<typename T>
//flann::Matrix<T> Array<2, T>::flann_matrix_view()
//{
//    return flann::Matrix<T>(data_.data(), size_[0], size_[1], stride_[0]);
//}

//template<typename T>
//EigenMatrixView Array<2, T>::eigen_matrix_view()
//{
//    assert(stride_[0] % sizeof(T) == 0);
//    return EigenMatrixView(data_.data(), size_[0], size_[1]);
//}
//
//template<typename T>
//EigenMatrixView Array<2, T>::eigen_matrix3x_view()
//{
//    assert(size_[0] == 3);
//    assert(stride_[0] % sizeof(T) == 0);
//    return EigenMatrix3XView(data_.data(), size_[0], size_[1]);
//}

}
#endif //NAEX_ARRAY_H
