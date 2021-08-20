
#ifndef NAEX_ITERATORS_H
#define NAEX_ITERATORS_H

#include <naex/types.h>

namespace naex
{
template<typename It>
class Strided
{
public:
    typedef typename std::iterator_traits<It>::difference_type difference_type;
    typedef typename std::iterator_traits<It>::iterator_category iterator_category;
    typedef typename std::iterator_traits<It>::pointer pointer;
    typedef typename std::iterator_traits<It>::reference reference;
    typedef typename std::iterator_traits<It>::value_type value_type;

    typedef Strided<It> Same;
    typedef std::shared_ptr<Same> Ptr;
    typedef std::shared_ptr<const Same> ConstPtr;

    Strided(const It& iter, const difference_type stride):
        iter_(iter),
        stride_(stride)
    {}
    Strided(const Same& other):
        Strided(other.iter_, other.stride_)
    {}
    reference operator*() const
    {
        return *iter_;
    }
    Same& operator++()
    {
        iter_ += stride_;
        return *this;
    }
    Same operator++(int)
    {
        Same copy(*this);
        operator++();
        return copy;
    }
    Same& operator--()
    {
        iter_ -= stride_;
        return *this;
    }
    Same operator--(int)
    {
        Same copy(*this);
        operator--();
        return copy;
    }
    bool operator==(const Same& other)
    {
        return (iter_ == other.iter_);
    }
    bool operator!=(const Same& other)
    {
        return !this->operator==(other);
    }
    bool operator<(const Same& other)
    {
        return (iter_ < other.iter_);
    }
    bool operator<=(const Same& other)
    {
        return (iter_ <= other.iter_);
    }
    bool operator>(const Same& other)
    {
        return (iter_ > other.iter_);
    }
    bool operator>=(const Same& other)
    {
        return (iter_ >= other.iter_);
    }
    Same& operator+=(const difference_type diff)
    {
        iter_ += stride_ * diff;
        return *this;
    }
    Same& operator-=(const difference_type diff)
    {
        iter_ -= stride_ * diff;
        return *this;
    }
    Same operator+(const difference_type diff)
    {
        return Same(*this) += diff;
    }
    Same operator-(const difference_type diff)
    {
        return Same(*this) -= diff;
    }
    reference operator[](const difference_type diff)
    {
        return *(iter_ + stride_ * diff);
    }
    friend difference_type operator-(const Same it1, const Same it0)
    {
        assert(it0.stride_ == it1.stride_);
        return (it1.iter_ - it0.iter_) / it1.stride_;
    }
private:
    It iter_;
    difference_type stride_;
};

/**
 * Primitive type value iterator.
 *
 * @tparam V Value type
 */
template<typename V>
class ValueIterator
{
public:
    ValueIterator(const V &val) :
        value_(val)
    {
    }
    ValueIterator &
    operator++()
    {
        value_++;
        return *this;
    }
    ValueIterator &
    operator--()
    {
        value_--;
        return *this;
    }
    bool
    operator!=(const ValueIterator<V> &other)
    {
        return value_ != other.value_;
    }
    V &
    operator*()
    {
        return value_;
    }
    const V &
    operator*() const
    {
        return value_;
    }
private:
    V value_;
};

typedef ValueIterator<Vertex> VertexIter;
typedef ValueIterator<Edge> EdgeIter;

}  // namespace naex

#endif //NAEX_ITERATORS_H
