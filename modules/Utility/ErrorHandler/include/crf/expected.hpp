/*
 * © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021 following guidance from std::optional
 *
 *  ==================================================================================================
*/
#pragma once

#include <type_traits>
#include <stdexcept>
#include <utility>
#include <initializer_list>

#include "crf/ResponseCode.hpp"

namespace crf {

/**
 * @ingroup group_error_handler
 * @brief Class bad expected access used when you access the wrong object (payload or error),
 *        when it's not available.
 */
class bad_expected_access: public std::exception {
 public:
    bad_expected_access() {}
    const char* what() const noexcept override {
        return "bad expected access";
    }
    virtual ~bad_expected_access() noexcept = default;
};

/**
 * @brief Template class for the container "expected". It works as container that expects to
 * receive a type T. The user can return the object or an error code to indicate why the object
 * is not available. It also offers the posibility to return both in case there is some
 * non-critical problem.
 *
 * @tparam T Type to store in the container
 */
template<typename T>
class expected {
 private:
    /**
     * @brief This union is neccessary in case the payload does not have a
     *        default CTor in which case we need to make sure the template
     *        CTor does not call it. (jplayang)
     */
    struct Empty_ {};
    union {
        Empty_ empty_;
        T payload_;
    };
    bool engaged_ = false;
    ResponseCode response_;

    /**
     * @brief Create a function to construct a new payload T object in the address asinged
     *        to the previous one. We don't need a delete since it's the same memory as before,
     *        the DTor will manage it. Preserves the pointers created to the payload by other
     *        containers such as std::vector. If not implemented can cause SegFault,
     *        munmap_chunk(): invalid pointer, data corruption, double free.
     *        (jplayang)
     */
    template<typename... PayloadParams>
    void constructNewPayload(PayloadParams&&... __args) noexcept(
        std::is_nothrow_constructible<std::remove_const_t<T>, PayloadParams...>()) {
        // Placement new (address) T
        // they cast __addressof to void* to go to the first memory address of the obj and allocate from there    NOLINT
        ::new ((void *) std::__addressof(this->payload_)) std::remove_const_t<T>(std::forward<PayloadParams>(__args)...);  // NOLINT
        engaged_ = true;
    }

 public:
    // Default CTor
    constexpr expected() noexcept:
        empty_(),
        response_(Code::Empty) {}

    // Copy CTor
    constexpr expected(const expected& other) {
        if (other) {
            constructNewPayload(other.payload_);
        }
        response_ = other.response_;
    }

    // Move CTor
    constexpr expected(expected&& other) noexcept(std::is_nothrow_move_constructible<T>()) {
        if (other) {
            constructNewPayload(std::move(other.payload_));
        }
        response_ = std::move(other.response_);
    }

    // DTor
    ~expected() {
        if (engaged_) payload_.~T();  // explicit call to DTor
    }

    // Other CTors
    /**
     * @brief With this we assure that structures like crf::expected<crf::Code> are not
     *        allowed. Although, even without it, they wouldn't compile.
     *        (jplayang)
     */
    template <typename U = T,
        std::enable_if_t<
            std::__and_<
                std::__not_<std::is_same<U, crf::ResponseCode>>,
                std::__not_<std::is_same<U, crf::Code>>
            >::value, bool> = true>
    constexpr expected(const T& payload, Code code = Code::OK) noexcept {  // NOLINT
        if (!engaged_) {
            constructNewPayload(payload);
        } else {
            payload_ = payload;
        }
        engaged_ = true;
        response_ = ResponseCode(code);
    }
    template <typename U = T,
        std::enable_if_t<
            std::__and_<
                std::__not_<std::is_same<U, crf::ResponseCode>>,
                std::__not_<std::is_same<U, crf::Code>>
            >::value, bool> = true>
    constexpr expected(T&& payload, Code code = Code::OK) noexcept {  // NOLINT
        if (!engaged_) {
            constructNewPayload(std::move(payload));
        } else {
            payload_ = std::move(payload);
        }
        response_ = ResponseCode(code);
    }
    constexpr expected(const ResponseCode& response) noexcept {  // NOLINT
        empty_ = Empty_();
        engaged_ = false;
        response_ = response;
    }
    constexpr expected(ResponseCode&& response) noexcept {  // NOLINT
        empty_ = Empty_();
        engaged_ = false;
        response_ = std::move(response);
    }
    constexpr expected(const Code& code) noexcept {  // NOLINT
        empty_ = Empty_();
        engaged_ = false;
        response_ = ResponseCode(code);
    }
    constexpr expected(Code&& code) noexcept {  // NOLINT
        empty_ = Empty_();
        engaged_ = false;
        response_ = ResponseCode(std::move(code));
    }

    /**
     * @brief Constructors to allow calling the CTor of the payload
     * directly from the CTor of expected
     *
     * @tparam U Type of the initializer list neccesary for T
     * (ex: in crf::expected<std::vector<float>>, T is std::vector<float>, and U is float)
     */
    template<typename U>
    constexpr expected(std::initializer_list<U> list):  // NOLINT
        payload_(list),
        engaged_(true),
        response_(Code::OK) {}
    template<typename ...PayloadParameters>
    constexpr expected(PayloadParameters... params):  // NOLINT
        payload_(std::forward<PayloadParameters>(params)...),
        engaged_(true),
        response_(Code::OK) {}
    template<typename U, typename... PayloadParameters>
    constexpr expected(std::initializer_list<U> list, PayloadParameters&&... params):
        payload_(list, std::forward<PayloadParameters>(params)...),
        engaged_(true),
        response_(Code::OK) {}

    /**
     * @brief Method to change response code without changing
     * the payload
     *
     * @param newCode The new code to attach to the payload
     * @param detail In case a new detail wants to be added as well
     *
     */
    constexpr void attach(const Code& newCode, const int64_t& detail = 0) noexcept {
        response_ = ResponseCode(newCode, detail);
    }
    /**
     * @brief Method to change response code without changing
     * the payload
     *
     * @param newCode The new code to attach to the payload
     *
     */
    constexpr void attach(const ResponseCode& response) noexcept {
        response_ = response;
    }

    /**
     * @brief Method to return the response of the object
     *
     * @return ResponseCode The response attached to the payload
     */
    constexpr ResponseCode get_response() const noexcept {
        return response_;
    }

    /**
     * @brief Method to return the expected payload
     *
     * @return Payload placed in the container
     */
    // Value for const lvalue obj
    constexpr const T& value() const& {
        if (engaged_) return payload_;
        throw bad_expected_access();
    }
    // Value for non-const obj lvalue
    constexpr T& value()& {
        if (engaged_) return payload_;
        throw bad_expected_access();
    }
    // value for rvalue obj non-const
    constexpr T&& value()&& {
        if (engaged_) return std::move(payload_);
        throw bad_expected_access();
    }
    // value for rvalue const obj
    constexpr const T&& value() const&& {
        if (engaged_) return std::move(payload_);
        throw bad_expected_access();
    }

    /**
     * @brief Method to know if container has a payload. Useful in case
     * this object is inside another container and it's not allowed
     * to use the bool operator
     *
     * @return true if a payload is attached
     * @return false if there is no payload
     */
    constexpr bool is_engaged() const noexcept {
        return engaged_;
    }

    /**
     * @brief Method to reset the container. It deletes the current payload and
     * sets the code to empty
     *
     */
    constexpr void reset() noexcept {
        response_ = ResponseCode(Code::Empty);
        if (engaged_) payload_.~T();
        engaged_  = false;
        empty_ = Empty_();
    }

    /**
     * @brief Operator bool to know if the container is engaged
     *
     * @return constexpr operator bool
     */
    constexpr operator bool() const noexcept {
        return engaged_;
    }

    expected& operator=(const expected& other) {
        if (engaged_ && other.engaged_) {
            payload_ = other.payload_;
        } else {
            if (other.engaged_) {
                constructNewPayload(other.payload_);
            } else {
                reset();
            }
        }
        response_ = other.response_;
        return *this;
    }
    expected& operator=(expected&& other) noexcept(
        std::__and_<std::is_nothrow_move_constructible<T>,
        std::is_nothrow_move_assignable<T>>()) {
        if (engaged_ && other.engaged_) {
            payload_ = std::move(other.payload_);
        } else {
            if (other.engaged_) {
                constructNewPayload(std::move(other.payload_));
            } else {
                reset();
            }
        }
        response_ = other.response_;
        return *this;
    }

    constexpr expected& operator=(const ResponseCode& response) noexcept {
        reset();
        response_ = response;  // What do I do with the payload, force it empty? leave it?
        return *this;
    }
    constexpr expected& operator=(const Code& code) noexcept {
        reset();
        response_ = ResponseCode(code);  // What do I do with the payload, force it empty? leave it?
        return *this;
    }

    constexpr expected& operator=(const T& payload) noexcept {
        if (engaged_) {
            payload_ = payload;
        } else {
            constructNewPayload(payload);
        }
        response_ = ResponseCode(Code::OK);
        return *this;
    }
    constexpr expected& operator=(T&& payload) noexcept {
        if (engaged_) {
            payload_ = std::move(payload);
        } else {
            constructNewPayload(std::move(payload));
        }
        response_ = ResponseCode(Code::OK);
        return *this;
    }

    // Operators for dereferencing and pointers
    constexpr const T* operator->() const {
        return std::__addressof(payload_);
    }
    T* operator->() {
        return std::__addressof(payload_);
    }
    constexpr const T& operator*() const& {
        return payload_;
    }
    constexpr T& operator*()& {
        return payload_;
    }
    constexpr T&& operator*()&& {
        return std::move(payload_);
    }
    constexpr const T&& operator*() const&& {
        return std::move(payload_);
    }

    // Comparisons
    constexpr bool operator==(const expected<T>& other) {
        if (engaged_ && other.engaged_) {
            if (std::is_same_v<decltype(payload_), decltype(other.payload_)>) {
                if (payload_ == other.payload_ && response_ == other.response_) return true;
            }
        }
        return false;
    }
    constexpr bool operator!=(const expected<T>& other) {
        return !operator==(other);
    }
};

}  // namespace crf
