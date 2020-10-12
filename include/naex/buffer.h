#ifndef NAEX_BUFFER_H
#define NAEX_BUFFER_H

#include <cassert>
#include <functional>
#include <memory>

namespace naex
{

template<typename T>
void noop(T*)
{}

template<typename T>
class Buffer
{
public:
//    typedef T Type;
//    typedef std::shared_ptr<T> TypePtr;
//    typedef std::function<void(T*)> Delete;

    Buffer():
            begin_(),
            size_(0)
    {}

    Buffer(const Buffer<T>& other):
            begin_(other.begin_),
            size_(other.size_)
    {}

    Buffer(std::shared_ptr<T> other):
            begin_(other),
            size_(1)
    {
        // TODO: Check the deleter?
    }

    Buffer(size_t size):
//            buffer_(std::make_shared<T[]>(size)),
//            buffer_(std::make_shared<T[]>(size)),
            begin_(new T[size], std::default_delete<T[]>()),
//            end_(begin_.get() + size)
            size_(size)
    {}

    Buffer(T* begin, size_t size):
//            begin_(begin, [](T*){}),
            begin_(begin, noop<T>),
//            end_(begin + size)
            size_(size)
    {
        assert(begin != nullptr);
        assert(size > 0);
//        buffer_ = std::shared_ptr<T>(begin, []{});
//        size_ = size;
    }

    Buffer(std::shared_ptr<T> begin, size_t size):
            begin_(begin),
            size_(size)
    {
        assert(begin.get() != nullptr);
        assert(size > 0);
    }

    Buffer(T* begin, T* end):
//            begin_(begin, [](T*){}),
            begin_(begin, noop<T>),
//            end_(end)
            size_(end - begin)
//            Buffer(begin)
    {
        assert(begin != nullptr);
        assert(end != nullptr);
        assert(begin <= end);
//        buffer_ = std::shared_ptr<T>(begin, []{});
//        size_ = end - begin;
//        Buffer(begin, end - begin);
    }

    Buffer(T* begin, T* end, std::function<void(T*)> deleter):
//            begin_(begin, [](T*){}),
            begin_(begin, deleter),
//            end_(end)
            size_(end - begin)
    {
        assert(begin != nullptr);
        assert(end != nullptr);
        assert(begin <= end);
//        buffer_ = std::shared_ptr<T>(begin, []{});
//        size_ = end - begin;
//        Buffer(begin, end - begin);
    }

    size_t size() const
    {
        return size_;
//        if (!begin_)
//        {
//            return 0;
//        }
//        assert(begin_.get() <= end_);
//        return (end_ - begin_.get());
    }

    bool empty() const
    {
        return size() == 0;
//        return !begin_;
    }

    void resize(size_t size)
    {
        begin_ = std::shared_ptr<T>(new T[size], std::default_delete<T[]>());
//        end_ = begin_.get() + size;
        size_ = size;
    }

    Buffer<T> copy(size_t size)
    {
        Buffer<T> out(size);
        std::copy(begin(), begin() + std::min(this->size(), size), out.begin());
        return out;
    }

    T* begin()
    {
        return begin_.get();
    }

    const T* begin() const
    {
        return begin_.get();
    }

    T* end()
    {
        if (empty())
        {
            return nullptr;
        }
//        assert(begin_ != nullptr);
        return begin_.get() + size_;
//        return end_;
    }

    const T* end() const
    {
        if (empty())
        {
            return nullptr;
        }
//        assert(begin_ != nullptr);
        return begin_.get() + size_;
//        return end_;
    }

    T& operator[](size_t i)
    {
//        assert(i < size_);
        assert(i < size());
        return *(begin_.get() + i);
    }

    const T& operator[](size_t i) const
    {
//        assert(i < size_);
        assert(i < size());
        return *(begin_.get() + i);
    }

    template<typename D>
    D* data()
    {
        reinterpret_cast<D*>(begin_.get());
    }

    template<typename D>
    explicit operator D*()
    {
        return data<D>();
    }

    explicit operator std::string() const
    {
        if (empty())
        {
            return {};
        }
        std::string(data<char>(), size());
    }

protected:
//    std::shared_ptr<T[]> buffer_;
//    std::shared_ptr<T*> buffer_;
    std::shared_ptr<T> begin_;
    // TODO: Share end ptr or size too?
//    T* end_;
    size_t size_;
};

}
#endif //NAEX_BUFFER_H
