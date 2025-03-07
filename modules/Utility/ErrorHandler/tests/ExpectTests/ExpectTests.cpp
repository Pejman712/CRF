/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "crf/expected.hpp"

class ExpectedShould: public ::testing::Test {
 protected:
    ExpectedShould():
      logger_("ExpectedShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~ExpectedShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    template<typename T>
    crf::expected<T> returnExpectedFunctionWith(T param) {
        return param;
    }

    template<typename T>
    T receiveExpectedFunctionWith(crf::expected<T> exp) {
        return exp.value();
    }

    // Class with deleted default CTors that throws in the CTor and in a method
    class EntityX {
     private:
        int a_;
     public:
        EntityX() = delete;
        EntityX(const EntityX& other) = delete;
        EntityX(EntityX&& other) = delete;
        explicit EntityX(int a):
            a_(a) {
                if (a_ == 0) throw std::runtime_error("CTor");
        }
        constexpr int get() const {
            if (a_ == 10) throw std::runtime_error("Method");
            return a_;
        }
        ~EntityX() {
        }
    };

    // Class with CTor and DTor implicit
    class EntityA {
     private:
        int a_;
     public:
        explicit EntityA(int a):
            a_(a) {
        }
        bool operator==(const EntityA& other) {
            return a_ == other.a_;
        }
        bool operator!=(const EntityA& other) {
            return a_ != other.a_;
        }
    };

    // Class with a non-trivial DTor and trivial default CTors
    class EntityB {
     private:
        int a_;
     public:
        explicit EntityB(int a):
            a_(a) {
        }
        constexpr int get() const noexcept {
            return a_;
        }
        ~EntityB() {
        }
    };

    // Class with a non-trivial DTor, constexpr, and trivial default CTors
    class EntityC {
     private:
        int a_ = 10;
     public:
        constexpr explicit EntityC(int a):
            a_(a) {
        }
        constexpr int get() const noexcept {
            return a_;
        }
        ~EntityC() {
        }
    };

    // Class with a non-trivial CopyCTor, MoveCTor, and DTor, and operator(operator= is needed)
    class EntityD {
     private:
        int a_;

     public:
        // Default
        constexpr EntityD():
            a_(0) {}
        // Copy CTor
        constexpr EntityD(const EntityD& other):
            a_(other.a_) {}
        // Move CTor
        constexpr EntityD(EntityD&& other):
            a_(std::move(other.a_)) {}
        constexpr EntityD& operator=(EntityD&& other) {
            a_ = std::move(other.a_);
            return *this;
        }
        constexpr explicit EntityD(int a):
            a_(a) {}
        constexpr int get() const noexcept {
            return a_;
        }

        ~EntityD() {}
    };

    // Class several input par
    class EntityE {
     private:
        int a_;
        int b_;
        int c_;

     public:
        constexpr EntityE(int a, int b, int c):
            a_(a),
            b_(b),
            c_(c) {
        }
        constexpr EntityE() = delete;
        constexpr EntityE(const EntityE& other) = delete;
        constexpr EntityE(EntityE&& other) = delete;

        constexpr int get() const noexcept {
            return a_+b_+c_;
        }
        ~EntityE() {
        }
    };

    crf::utility::logger::EventLogger logger_;
};

TEST_F(ExpectedShould, throwExceptionWhenAccessedWrong) {
    crf::expected<bool> sut;

    ASSERT_THROW(sut.value(), std::exception);
    ASSERT_EQ(sut.get_response(), crf::ResponseCode(crf::Code::Empty));

    sut = true;

    ASSERT_TRUE(sut.value());
    ASSERT_EQ(sut.get_response(), crf::ResponseCode(crf::Code::OK));

    sut = crf::ResponseCode(crf::Code::OK);

    ASSERT_THROW(sut.value(), std::exception);

    sut = crf::expected<bool>();

    ASSERT_THROW(sut.value(), std::exception);
}

TEST_F(ExpectedShould, workWhenDecalredAndInitialized) {
    crf::expected<bool> sut = true;

    ASSERT_TRUE(sut.value());
    ASSERT_EQ(sut.get_response(), crf::ResponseCode(crf::Code::OK));
}

TEST_F(ExpectedShould, workWhenAccessedThroughAFunction) {
    crf::expected<bool> sut = returnExpectedFunctionWith<bool>(true);

    ASSERT_NO_THROW(sut.value());
    ASSERT_EQ(sut.get_response(), crf::ResponseCode(crf::Code::OK));
}

TEST_F(ExpectedShould, workWhenAccessedAsAnRValue) {
    bool i = receiveExpectedFunctionWith<bool>(returnExpectedFunctionWith<bool>(true));
}

TEST_F(ExpectedShould, compileWhenDeclaringAnExpectedWithANonDefaultCTorObject) {
    /*
     * This test looks weird but it's done to make sure we can declare
     * the expected class without trying to create an object
     * (jplayang)
     */

    crf::expected<EntityA> var0;
    crf::expected<EntityB> var1;
    crf::expected<EntityC> var2;
    crf::expected<EntityD> var3;
}

TEST_F(ExpectedShould, WorkWhenAFunctionHasNonTrivialDestructor) {
    crf::expected<EntityB> var1 = EntityB(1);

    ASSERT_TRUE(var1);
    ASSERT_EQ(var1.value().get(), 1);

    crf::expected<EntityC> var2 = EntityC(10);

    ASSERT_TRUE(var2);
    ASSERT_EQ(var2.value().get(), 10);

    crf::expected<EntityD> var3 = EntityD(1);

    ASSERT_TRUE(var3);
    ASSERT_EQ(var3.value().get(), 1);
}


TEST_F(ExpectedShould, shouldBeAbleToBePlacedInMostContainersAndBeUsedCorrectly) {
    // Vector
    {
        std::vector<crf::expected<float>> vect;

        crf::expected<float> var12(1.0f);

        vect.push_back(crf::ResponseCode(crf::Code::Empty));
        vect.push_back(var12);
        vect.push_back(2.0f);

        ASSERT_EQ(vect.size(), 3);

        ASSERT_FALSE(vect.at(0));
        ASSERT_TRUE(vect.at(1));
        ASSERT_TRUE(vect.at(2));

        ASSERT_EQ(vect.at(0).get_response(), crf::ResponseCode(crf::Code::Empty));
        ASSERT_EQ(vect.at(1).get_response(), crf::ResponseCode(crf::Code::OK));
        ASSERT_EQ(vect.at(2).get_response(), crf::ResponseCode(crf::Code::OK));

        ASSERT_THROW(vect.at(0).value(), std::exception);
        ASSERT_EQ(vect.at(1).value(), 1.0f);
        ASSERT_EQ(vect.at(2).value(), 2.0f);
    }

    // Array
    {
        std::array<crf::expected<bool>, 3> array;

        crf::ResponseCode resp = crf::ResponseCode(crf::Code::Empty);

        array[0] = crf::ResponseCode(crf::Code::OK);
        array[1] = true;
        array[2] = resp;

        ASSERT_FALSE(array.at(0));
        ASSERT_TRUE(array.at(1));
        ASSERT_FALSE(array.at(2));

        ASSERT_EQ(array.at(0).get_response(), crf::ResponseCode(crf::Code::OK));
        ASSERT_EQ(array.at(1).get_response(), crf::ResponseCode(crf::Code::OK));
        ASSERT_EQ(array.at(2).get_response(), crf::ResponseCode(crf::Code::Empty));

        ASSERT_THROW(array.at(0).value(), std::exception);
        ASSERT_TRUE(array.at(1).value());
        ASSERT_THROW(array.at(2).value(), std::exception);
    }

    // Map
    {
        std::map<std::string, crf::expected<bool>> map;

        crf::expected<bool> var1 = true;
        crf::expected<bool> var2 = crf::expected<bool>(crf::Code::OK);

        map.insert({"stringfunny.png", crf::expected<bool>()});
        map.insert({"stringveryfunny.png", var1});
        map.insert({"stringsuperfunny.png", var2});

        auto search = map.find("stringfunny.png");
        ASSERT_FALSE(search->second);
        ASSERT_THROW(search->second.value(), std::exception);


        search = map.find("stringveryfunny.png");
        ASSERT_TRUE(search->second);
        ASSERT_EQ(search->second.value(), true);

        search = map.find("stringsuperfunny.png");
        ASSERT_FALSE(search->second);
        ASSERT_THROW(search->second.value(), std::exception);
    }

    // Ptrs
    {
        std::shared_ptr<crf::expected<EntityD>> ptrshrd =
            std::make_shared<crf::expected<EntityD>>(EntityD(10));

        ASSERT_TRUE(ptrshrd->is_engaged());
        ASSERT_EQ(ptrshrd->value().get(), 10);

        std::unique_ptr<crf::expected<EntityD>> ptrunq =
            std::make_unique<crf::expected<EntityD>>(EntityD(10));

        ASSERT_TRUE(ptrunq->is_engaged());
        ASSERT_EQ(ptrunq->value().get(), 10);

        std::weak_ptr<crf::expected<EntityD>> ptrweak = ptrshrd;
    }
}

TEST_F(ExpectedShould, shouldBeAbleToIncorporateMostContainersAndBeUsedCorrectly) {
    // Vector
    {
        crf::expected<std::vector<EntityC>> expVec;

        ASSERT_FALSE(expVec);
        ASSERT_EQ(expVec.get_response(), crf::ResponseCode(crf::Code::Empty));

        expVec = crf::Code::OK;

        ASSERT_FALSE(expVec);
        ASSERT_EQ(expVec.get_response(), crf::ResponseCode(crf::Code::OK));

        expVec = std::vector<EntityC>();

        ASSERT_TRUE(expVec);

        expVec.value().push_back(EntityC(1));
        expVec.value().push_back(EntityC(100));
        expVec.value().push_back(EntityC(10));
        expVec.value().push_back(EntityC(1));

        ASSERT_EQ(expVec.value().at(0).get(), 1);
        ASSERT_EQ(expVec.value().at(1).get(), 100);
        ASSERT_EQ(expVec.value().at(2).get(), 10);
        ASSERT_EQ(expVec.value().at(3).get(), 1);
    }

    // Array
    {
        crf::expected<std::array<bool, 2>> expArr;

        expArr = crf::ResponseCode(crf::Code::OK);

        ASSERT_FALSE(expArr);

        expArr = std::array<bool, 2>();

        ASSERT_TRUE(expArr);

        std::array<bool, 2> ar;

        expArr = ar;

        ASSERT_TRUE(expArr);

        expArr.value()[0] = true;

        ASSERT_TRUE(expArr.value().at(0));
    }

    // Map
    {
        crf::expected<std::map<std::string, bool>> expMap;

        expMap = crf::ResponseCode(crf::Code::OK);

        ASSERT_FALSE(expMap);

        expMap = std::map<std::string, bool>();

        ASSERT_TRUE(expMap);

        expMap.value().insert({"stringfunny.png", false});
        expMap.value().insert({"stringveryfunny.png", true});
        expMap.value().insert({"omgimlaughinglotsplssomeoneendthispain.png", false});

        auto search = expMap.value().find("stringfunny.png");
        ASSERT_FALSE(search->second);

        search = expMap.value().find("stringveryfunny.png");
        ASSERT_TRUE(search->second);

        search = expMap.value().find("omgimlaughinglotsplssomeoneendthispain.png");
        ASSERT_FALSE(search->second);
    }

    // Ptrs
    {
        crf::expected<std::shared_ptr<EntityD>> expShrptr;

        ASSERT_FALSE(expShrptr);

        expShrptr = std::make_shared<EntityD>(10);

        ASSERT_TRUE(expShrptr);
        ASSERT_EQ(expShrptr.value()->get(), 10);

        crf::expected<std::unique_ptr<EntityD>> expUnqptr;

        ASSERT_FALSE(expUnqptr);

        expUnqptr = std::make_unique<EntityD>(10);

        ASSERT_TRUE(expUnqptr);
        ASSERT_EQ(expUnqptr.value()->get(), 10);

        crf::expected<std::weak_ptr<EntityD>> expWeakptr = expShrptr.value();
    }
}

TEST_F(ExpectedShould, constructCorrectlyWhenInitializedAndDeclared) {
    crf::expected<EntityA> var(100);

    crf::expected<EntityE> var1(100, 10, 1);

    ASSERT_TRUE(var1);
    ASSERT_EQ(var1.value().get(), 111);

    crf::expected<EntityX> var13(100);

    ASSERT_TRUE(var13);
    ASSERT_EQ(var13.value().get(), 100);
}


TEST_F(ExpectedShould, callAllTheInternalFunctionsCorrectlyAndBehaveCorrectly) {
    logger_->info("EmptyCTor");
    crf::expected<float> var1;
    ASSERT_FALSE(var1);
    ASSERT_EQ(var1.get_response(), crf::ResponseCode(crf::Code::Empty));

    logger_->info("CopyCTor");
    var1 = 3.4f;
    crf::expected<float> var2(var1);
    ASSERT_TRUE(var2);
    ASSERT_EQ(var2.value(), 3.4f);

    logger_->info("MoveCTor");
    crf::expected<float> var3(std::move(var1));
    ASSERT_TRUE(var3);
    ASSERT_EQ(var3.value(), 3.4f);

    logger_->info("CTor T rvalue");
    crf::expected<float> var4(3.78f);
    ASSERT_TRUE(var4);
    ASSERT_EQ(var4.value(), 3.78f);

    logger_->info("CTor T lvalue");
    float num = 3.78f;
    crf::expected<float> var5(num);
    ASSERT_TRUE(var5);
    ASSERT_EQ(var5.value(), 3.78f);

    logger_->info("CTor response rvalue");
    crf::expected<float> var6(crf::ResponseCode(crf::Code::OK));
    ASSERT_FALSE(var6);
    ASSERT_EQ(var6.get_response(), crf::ResponseCode(crf::Code::OK));

    logger_->info("CTor response lvalue");
    crf::ResponseCode resp = crf::ResponseCode(crf::Code::OK);
    crf::expected<float> var7(resp);
    ASSERT_FALSE(var6);
    ASSERT_EQ(var6.get_response(), crf::ResponseCode(crf::Code::OK));

    logger_->info("CTor code rvalue");
    crf::expected<float> var8(crf::Code::OK);
    ASSERT_FALSE(var6);
    ASSERT_EQ(var6.get_response(), crf::ResponseCode(crf::Code::OK));

    logger_->info("CTor code lvalue");
    crf::Code code = crf::Code::OK;
    crf::expected<float> var9(code);
    ASSERT_FALSE(var6);
    ASSERT_EQ(var6.get_response(), crf::ResponseCode(crf::Code::OK));

    logger_->info("CTor T parms");
    crf::expected<EntityE> var10(3, 4, 5);
    ASSERT_TRUE(var10);
    ASSERT_EQ(var10.value().get(), 12);

    logger_->info("CTor T initList");
    crf::expected<std::vector<int>> var11({3, 4, 5});
    ASSERT_TRUE(var11);
    ASSERT_EQ(var11.value().at(0), 3);

    logger_->info("DTor");
    {
        crf::expected<float> varDTor;
        ASSERT_FALSE(varDTor);
        ASSERT_EQ(varDTor.get_response(), crf::ResponseCode(crf::Code::Empty));
        // DTor called when exiting scope
    }

    logger_->info("operator=");

    // fancy way to make rvalue
    auto fun1 = [](crf::Code cod){
        return cod;
    };

    var1 = crf::Code::OK;
    var1 = crf::ResponseCode(crf::Code::OK);
    var1 = 1.9f;

    var2 = fun1(crf::Code::OK);;
    var2 = resp;
    var2 = num;
}

// Ridiculous but you can still do it and works correctly
TEST_F(ExpectedShould, constructWithCodeOrResponseOrExpected) {
    // Not allowed as definitions would be overloaded when T = code or T = response
    // crf::expected<crf::Code> var1;
    // crf::expected<crf::ResponseCode> var1;

    crf::expected<crf::expected<float>> var;

    crf::expected<float> expect = 3.4f;
    crf::expected<float> notexpect = crf::Code::OK;

    var = expect;

    ASSERT_TRUE(var);
    ASSERT_TRUE(var.value());
    ASSERT_EQ(var.value().value(), 3.4f);

    var = notexpect;

    ASSERT_TRUE(var);
    ASSERT_FALSE(var.value());
}

TEST_F(ExpectedShould, compileWhenAClassThrowsInCTorOrMethod) {
    crf::expected<EntityX> var(5);

    ASSERT_TRUE(var);
    ASSERT_EQ(var.value().get(), 5);

    ASSERT_THROW(crf::expected<EntityX> var1(0), std::runtime_error);

    crf::expected<EntityX> var2(10);

    ASSERT_TRUE(var2);
    ASSERT_THROW(var2.value().get(), std::runtime_error);
}

TEST_F(ExpectedShould, checkThatComparisonsAreWorking) {
    crf::expected<EntityA> var1(1);

    crf::expected<EntityA> var2(2);

    EntityA a(1);
    EntityA b(1);

    ASSERT_TRUE(a == b);
    ASSERT_FALSE(a != b);

    ASSERT_TRUE(var1 != var2);
    ASSERT_FALSE(var1 == var2);

    var2 = 1;

    ASSERT_TRUE(var1 == var2);
    ASSERT_FALSE(var1 != var2);
}

TEST_F(ExpectedShould, constructPayloadByInitializerListl) {
    crf::expected<std::vector<float>> var1({1.4f});
    crf::expected<std::vector<float>> var2({3.5f, 9.5f});

    ASSERT_TRUE(var1 != var2);
    ASSERT_FALSE(var1 == var2);

    var2.value().push_back(0.7);

    ASSERT_EQ(var1.value().at(0), 1.4f);

    ASSERT_EQ(var2.value().at(0), 3.5);
    ASSERT_EQ(var2.value().at(1), 9.5);
    ASSERT_EQ(var2.value().at(2), 0.7f);  // Explicit float because of how vector deals with types
}

TEST_F(ExpectedShould, constructPayloadByInitializerListWithOperatorEqual) {
    crf::expected<std::vector<float>> var1 = {1.4f};
    crf::expected<std::vector<float>> var2 = {2.0f, 9.0f};

    ASSERT_TRUE(var1 != var2);
    ASSERT_FALSE(var1 == var2);

    var2.value().push_back(0.7);

    ASSERT_EQ(var1.value().at(0), 1.4f);

    ASSERT_EQ(var2.value().at(0), 2);
    ASSERT_EQ(var2.value().at(1), 9);
    ASSERT_EQ(var2.value().at(2), 0.7f);
}

TEST_F(ExpectedShould, constructPayloadByInitializerList2) {
    crf::expected<std::vector<float>> var1{1.4f};
    crf::expected<std::vector<int>> var2{2, 5};

    ASSERT_TRUE(var1 != var2);
    ASSERT_FALSE(var1 == var2);

    var2.value().push_back(0);

    ASSERT_EQ(var1.value().at(0), 1.4f);

    ASSERT_EQ(var2.value().at(0), 2);
    ASSERT_EQ(var2.value().at(1), 5);
    ASSERT_EQ(var2.value().at(2), 0);
}

TEST_F(ExpectedShould, constructVectorPayloadByItsConstructor) {
    crf::expected<std::vector<double>> var1;
    crf::expected<std::vector<float>> var2(3, 9.5);

    ASSERT_TRUE(var1 != var2);
    ASSERT_FALSE(var1 == var2);

    ASSERT_EQ(var2.value().at(0), 9.5);
    ASSERT_EQ(var2.value().at(1), 9.5);
    ASSERT_EQ(var2.value().at(2), 9.5);
}

TEST_F(ExpectedShould, constructResponseCodeWithSecondaryErrorAndRetrieveIt) {
    auto fun = [](const bool& param) -> crf::expected<bool> {
        if (param) return false;
        return crf::ResponseCode(crf::Code::Accepted, 32);
    };

    ASSERT_TRUE(fun(true));  // Returns a bool
    ASSERT_FALSE(fun(true).value());  // The returned bool is false

    ASSERT_FALSE(fun(false));  // Returns a error
    ASSERT_EQ(fun(false).get_response(), crf::Code::Accepted);  // The error code is accepted
    ASSERT_EQ(fun(false).get_response().detail(), 32);  // The secpondary error is 32
}
