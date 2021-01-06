
#ifndef NAEX_EXCEPTIONS_H
#define NAEX_EXCEPTIONS_H

namespace naex
{
    class Exception: public std::runtime_error
    {
    public:
        Exception(const char* what):
            std::runtime_error(what)
        {}
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
