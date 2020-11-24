#ifndef NAEX_ARRAY_H
#define NAEX_ARRAY_H

#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <naex/buffer.h>



namespace naex
{

//namespace
//{
//typedef flann::Matrix<T> FlannMatrixView;
//typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> EigenMatrix;
//typedef Eigen::Map<EigenMatrix> EigenMatrixView;
//typedef Eigen::Matrix<T, 3, Eigen::Dynamic, Eigen::DontAlign> EigenMatrix3X;
//typedef Eigen::Map<EigenMatrix3> EigenMatrix3XView;
//}

template<size_t D, typename T>
class Array
{
    Array(size_t dim_0, size_t dim_1):
        data_(dim_0 * dim_1),
        size_{dim_0, dim_1},
        stride_{sizeof(T) * dim_1, sizeof(T)}
    {
        static_assert(D == 2, "D must be 2.");
        size_[0] = dim_0;
        size_[1] = dim_1;
    }
    flann::Matrix<T> flann_matrix_view()
    {
        static_assert(D == 2, "D must be 2.");
        return flann::Matrix<T>(data_.data(), size_[0], size_[1]);
    }
//    EigenMatrixView eigen_matrix_view()
//    {
//        static_assert(D == 2, "D must be 2.");
//        return EigenMatrixView(data_.data(), size_[0], size_[1]);
//    }
//    EigenMatrixView eigen_matrix3x_view()
//    {
//        static_assert(D == 2, "D must be 2.");
//        assert(size_[0] == 3);
//        return EigenMatrix3XView(data_.data(), size_[0], size_[1]);
//    }
public:
    Buffer<T> data_;
    size_t size_[D];
    size_t stride_[D];
};

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
