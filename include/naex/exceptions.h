
#ifndef NAEX_EXCEPTIONS_H
#define NAEX_EXCEPTIONS_H

#define BOOST_STACKTRACE_USE_BACKTRACE

#include <boost/stacktrace.hpp>
#include <exception>

namespace naex
{
class Exception: public std::runtime_error
{
public:
    Exception(const char* what):
            std::runtime_error(what)
    {
        std::stringstream ss;
        ss << boost::stacktrace::stacktrace();
        stacktrace_ = ss.str();
    }
    const std::string& stacktrace() const
    {
        return stacktrace_;
    }
private:
    std::string stacktrace_;
};

class NotInitialized: public Exception
{
public:
    NotInitialized(const char* what):
            Exception(what)
    {}
};
}

#endif  // NAEX_EXCEPTIONS_H
