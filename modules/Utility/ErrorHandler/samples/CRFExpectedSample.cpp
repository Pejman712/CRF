/*
 * © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/

#include <iostream>
#include <string>

#include "crf/expected.hpp"

class ExampleClass {
 private:
    int a_;
    float b_;
    std::string text_;

 public:
    ExampleClass() = delete;

    ExampleClass(int a, float b, std::string text):
        a_(a),
        b_(b),
        text_(text) {
    }

    crf::expected<int> getInt() const {
        return a_;
    }
    crf::expected<int> notGetInt() const {
        return crf::Code::NotFound;
    }

    crf::expected<float> getFloat() const {
        return b_;
    }
    crf::expected<float> notGetFloat() const {
        return crf::Code::NotFound;
    }

    crf::expected<std::string> getString() const {
        return text_;
    }
    crf::expected<std::string> notGetString() const {
        return crf::Code::NotFound;
    }
};

class ExampleClass2 {
 private:
    ExampleClass obj_;

 public:
    ExampleClass2() = delete;

    explicit ExampleClass2(ExampleClass obj):
        obj_(obj) {
    }

    crf::expected<float> complexMethodNotWorking() const {
        crf::expected<float> var = obj_.notGetFloat();
        if (!var) {
            return var.get_response();
        }
        // Even thought returning var.value() is allowed, it won't retain the response message.
        // If you are not going to send a message yourself it's better to return everything
        return var;
    }

    crf::expected<float> complexMethodWorking() const {
        crf::expected<float> var = obj_.getFloat();
        if (!var) {
            return var.get_response();
        }
        // You can change the message for another one while keeping the payload
        var.attach(crf::Code::SwitchingProtocols);
        return var;
    }
};

int main() {
    crf::expected<bool> expectBool;

    // In this case, we just generated the instance, but it's empty
    // If we try to access the value inside we'll get an exception
    try {
        expectBool.value();
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }

    crf::expected<bool> emptyVariable;

    // We can use the boolean operator to check if the variable has something
    // inside, in case it's empty like this one we know that an access will
    // force an exception

    if (!emptyVariable) {
        std::puts("I'm empty!");
    }

    // In this case we can still check the answer that we received as to why there is no result
    std::cout << "Response Code: " << emptyVariable.get_response() << std::endl;

    // We can assign the new variable with a new element
    emptyVariable = true;

    // Now the variable is full! We have something inside and we can access it
    if (emptyVariable) {
        std::puts("I'm full!");
    }

    // The function value return the element inside the container.
    // In this case, since it'a a boolean we can check it deirectly
    if (emptyVariable.value()) {
        std::puts("The variable inside is a \"true\"!");
    }

    // When the message is full, we can also check the response
    std::cout << "Response Code: " << emptyVariable.get_response() << std::endl;
    // By default, if nothing gets specified, the response is 200 (OK)


    // This container can also be assigned as output of classes and functions
    // Here are some examples

    ExampleClass obj = ExampleClass(1, 3.14f, "Example text");

    crf::expected<int> var = obj.getInt();

    if (var) {
        std::cout << "The value is: " << var.value() << std::endl;
        std::cout << "Response Code: " << var.get_response() << std::endl;
    }

    var = obj.notGetInt();

    if (!var) {
        // The variable is empty, let's check the error message
        std::cout << "Response Code: " << var.get_response() << std::endl;
    }

    // This errors are easy to propagate up within the hierarchy
    ExampleClass2 exp = ExampleClass2(obj);

    crf::expected<float> var20;

    var20 = exp.complexMethodWorking();

    if (!var20) {
        // The error propagated from the previous class
        std::cout << "Response Code: " << var20.get_response() << std::endl;
    }

    var20 = exp.complexMethodWorking();

    if (var20) {
        std::cout << "The value is: " << var20.value() << std::endl;
        std::cout << "Response Code: " << var20.get_response() << std::endl;
    }

    std::cout << "Secondary code test" << std::endl;

    crf::expected<bool> resultado;

    resultado = true;

    std::cout << resultado.get_response() << std::endl;

    resultado = crf::Code::Accepted;

    std::cout << resultado.get_response() << std::endl;

    resultado = crf::ResponseCode(crf::Code::KJ_OtherError, 0xFF);

    std::cout << std::hex << resultado.get_response() << std::endl;
}
