#pragma once

#include <memory>
#include <vector>

namespace naex
{

template<typename T>
class Filter
{
public:
    typedef Filter<T> Same;
    typedef std::shared_ptr<Same> Ptr;
    typedef std::shared_ptr<const Same> ConstPtr;

    virtual ~Filter() = default;

    /// Filter the input to output.
    virtual void filter(const T& input, T& output) = 0;

    /// Get child type name.
    const char* type_name() const
    {
        return typeid(*this).name();
    }
};

template<typename T>
class FilterChain: public Filter<T>
{
public:
    typedef std::vector<typename Filter<T>::Ptr> Filters;

    explicit FilterChain(const Filters& filters):
        filters_(filters)
    {}
    virtual ~FilterChain() = default;

    void filter(const T& input, T& output)
    {
        // Pretend the input is an output of some previous filter.
        T in, out = input;
        for (auto& f: filters_)
        {
            // With swap, we may reuse the already allocated buffers for following output.
            std::swap(in, out);
            f->filter(in, out);
        }
        output = std::move(out);
    }

protected:
    Filters filters_;
};

template<typename T>
class Processor
{
public:
    typedef Processor<T> Same;
    typedef std::shared_ptr<Same> Ptr;
    typedef std::shared_ptr<const Same> ConstPtr;

    virtual ~Processor() = default;

    /// Filter the input in-place.
    virtual void process(T& obj) = 0;

    /// Get child type name.
    const char* type_name() const
    {
        return typeid(*this).name();
    }
};

template<typename T>
class ProcessorChain: public Processor<T>
{
public:
    typedef std::vector<typename Processor<T>::Ptr> Processors;

    explicit ProcessorChain(const Processors& processors):
        processors_(processors)
    {}
    virtual ~ProcessorChain() = default;

    void process(T& obj)
    {
        for (auto& p: processors_)
        {
            p->process(obj);
        }
    }

protected:
    Processors processors_;
};

template<typename T>
class FilterFromProcessor: public Filter<T>
{
public:
    FilterFromProcessor(typename Processor<T>::Ptr processor):
        processor_(processor)
    {}
    virtual ~FilterFromProcessor() = default;

    virtual void filter(const T& input, T& output) override
    {
        output = input;
        processor_->process(output);
    }
protected:
    typename Processor<T>::Ptr processor_;
};

template<typename T>
class ProcessorFromFilter: public Processor<T>
{
public:
    ProcessorFromFilter(typename Filter<T>::Ptr filter):
        filter_(filter)
    {}
    virtual ~ProcessorFromFilter() = default;

    virtual void process(T& obj) override
    {
        T in = obj;
        filter_->filter(in, obj);
    }
protected:
    typename Filter<T>::Ptr filter_;
};

}  // namespace naex
