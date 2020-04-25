//
// Created by damian on 25.04.2020.
//

#ifndef UTILS_MAPEXCEPTION_H
#define UTILS_MAPEXCEPTION_H
#include <exception>
#include <string>
namespace util {
    class MapException : virtual public std::exception {
    public:
        explicit MapException(std::string message) : std::exception(), message_(message) {}

        virtual const char *what() const noexcept {
            return message_.c_str();
        }

        virtual ~MapException() {}

    protected:
        std::string message_;
    };

    class OutOfBoundsException : public MapException {
    public:
        explicit OutOfBoundsException(std::string message) : MapException(message) {}
        virtual ~OutOfBoundsException() {}
    };
}

#endif //UTILS_MAPEXCEPTION_H
