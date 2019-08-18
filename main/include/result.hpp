#ifndef RESULT_HPP__
#define RESULT_HPP__

#include <cstdint>
#include <type_traits>
#include "freertos_util.hpp"


// Result -------------------------

template<typename T>
struct Success
{
    T value;
    
    Success(const T& value) : value(value) {}
    Success(T&& value) : value(std::forward<T>(value)) {}
};

template<> struct Success<void>{};


template<typename E>
struct Failure
{
    E error_code;
    
    Failure(const E& error_code) : error_code(error_code) {}
};

template<typename T, typename E, E SuccessCode = static_cast<E>(0)>
struct Result
{
    typedef Success<T> SuccessType;
    typedef Failure<E> FailureType;

    T value;
    bool is_success;
    E error_code;

    Result() = default;
    Result(const SuccessType& success) : value(success.value), is_success(true), error_code() {}
    Result(SuccessType&& success) : value(std::move(success.value)), is_success(true), error_code() {}
    Result(const FailureType& failure) : value(), is_success(false), error_code(failure.error_code) {}

    operator bool() const { return this->is_success; }

    template<typename Func>
    typename std::result_of<Func(T&&)>::type then(Func&& func)
    {
        if( this->is_success ) {
            return func(std::move(this->value));
        }
        else {
            return typename std::result_of<Func(T&&)>::type::FailureType(this->error_code);
        }
    }

};

template<typename E, E SuccessCode>
struct Result<void, E, SuccessCode>
{
    typedef Success<void> SuccessType;
    typedef Failure<E> FailureType;

    bool is_success;
    E error_code;

    Result() : is_success(true), error_code(SuccessCode) {};
    Result(const SuccessType&) : is_success(true), error_code(SuccessCode) {};
    Result(const FailureType& failure) : is_success(false), error_code(failure.error_code) {}
    Result(E error_code) : is_success(error_code == SuccessCode), error_code(error_code) {}
    operator bool() const { return this->is_success; }

    template<typename Func>
    typename std::result_of<Func()>::type then(Func&& func)
    {
        if( this->is_success ) {
            return func();
        }
        else {
            return typename std::result_of<Func()>::type::FailureType(this->error_code);
        }
    }
};

static Success<void> success(void) { return Success<void>(); }
template<typename T> Success<T> success(const T& value) { return Success<T>(value); }
template<typename T> Success<T> success(T&& value) { return Success<T>(std::forward<T>(value)); }
template<typename T, typename E, E SuccessCode> Success<T> success(Result<T, E, SuccessCode>&& success_result) { return Success<T>(std::move(success_result.value)); }
template<typename E> Failure<E> failure(const E& error_code) { return Failure<E>(error_code); }
template<typename T, typename E, E SuccessCode> Failure<E> failure(const Result<T, E, SuccessCode>& failure_result) { return Failure<E>(failure_result.error_code); }

#define RESULT_TRY(expr) do { auto result__ = (expr); if( !result__ ) return failure(result__); } while(0)
#endif //RESULT_HPP__