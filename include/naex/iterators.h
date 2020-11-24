
#ifndef NAEX_ITERATORS_H
#define NAEX_ITERATORS_H

namespace naex
{
template<typename It>
class Strided
{
public:
    typedef typename std::iterator_traits<It>::difference_type difference_type;
    typedef typename std::iterator_traits<It>::iterator_category
        iterator_category;
    typedef typename std::iterator_traits<It>::pointer pointer;
    typedef typename std::iterator_traits<It>::reference reference;
    typedef typename std::iterator_traits<It>::value_type value_type;

    Strided(const It &iter, const difference_type stride) :
        iter_(iter), stride_(stride)
    {
    }
    Strided(const Strided &other) : iter_(other.iter_), stride_(other.stride_)
    {
    }
    reference
    operator*() const
    {
        return *iter_;
    }
    Strided<It> &
    operator++()
    {
        iter_ += stride_;
        return *this;
    }
    Strided<It>
    operator++(int)
    {
        Strided<It> copy(*this);
        operator++();
        return copy;
    }
    Strided<It> &
    operator--()
    {
        iter_ -= stride_;
        return *this;
    }
    Strided<It>
    operator--(int)
    {
        Strided<It> copy(*this);
        operator--();
        return copy;
    }
    bool
    operator==(const Strided &other)
    {
        return (iter_ == other.iter_);
    }
    bool
    operator!=(const Strided &other)
    {
        return !this->operator==(other);
    }
    bool
    operator<(const Strided &other)
    {
        return (iter_ < other.iter_);
    }
    bool
    operator<=(const Strided &other)
    {
        return (iter_ <= other.iter_);
    }
    bool
    operator>(const Strided &other)
    {
        return (iter_ > other.iter_);
    }
    bool
    operator>=(const Strided &other)
    {
        return (iter_ >= other.iter_);
    }
    Strided<It> &
    operator+=(const difference_type diff)
    {
        iter_ += stride_ * diff;
        return *this;
    }
    Strided<It> &
    operator-=(const difference_type diff)
    {
        iter_ -= stride_ * diff;
        return *this;
    }
    reference
    operator[](const difference_type diff)
    {
        return *(iter_ + stride_ * diff);
    }
    friend difference_type
    operator-(const Strided<It> it1, const Strided<It> it0)
    {
        assert(it0.stride_ == it1.stride_);
        return (it1.iter_ - it0.iter_) / it1.stride_;
    }
private:
    It iter_;
    difference_type stride_;
};

template<typename It>
Strided<It>
operator+(Strided<It> iter, const typename Strided<It>::difference_type diff)
{
    return (iter += diff);
}
template<typename It>
Strided<It>
operator-(Strided<It> iter, const typename Strided<It>::difference_type diff)
{
    return (iter -= diff);
}

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
}  // namespace naex

#endif //NAEX_ITERATORS_H
