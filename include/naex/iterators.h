
#ifndef NAEX_ITERATORS_H
#define NAEX_ITERATORS_H

namespace naex
{
    /**
     * Primitive type value iterator.
     *
     * @tparam V Value type
     */
    template<typename V>
    class ValueIterator
    {
    public:
        ValueIterator(const V& val):
                value_(val)
        {
        }
        ValueIterator& operator++()
        {
            value_++;
            return *this;
        }
        ValueIterator& operator--()
        {
            value_--;
            return *this;
        }
        bool operator!=(const ValueIterator<V>& other)
        {
            return value_ != other.value_;
        }
        V& operator*()
        {
            return value_;
        }
        const V& operator*() const
        {
            return value_;
        }
    private:
        V value_;
    };

}  // namespace naex

#endif //NAEX_ITERATORS_H
