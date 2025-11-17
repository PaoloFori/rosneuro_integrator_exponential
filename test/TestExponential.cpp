#include "rosneuro_integrator_exponential/Exponential.h"
#include <gtest/gtest.h>

namespace rosneuro {
    namespace integrator {
        class TestGenericIntegratorSuite : public ::testing::Test {
            public:
                TestGenericIntegratorSuite() {}
                ~TestGenericIntegratorSuite() {}
                void SetUp() {
                    exponential = new Exponential();
                }
                void TearDown() {
                    delete exponential;
                }
                Exponential* exponential;
                const float fix_alpha = 0.98f;
        };

        TEST_F(TestGenericIntegratorSuite, TestConstructor) {
            EXPECT_EQ(exponential->name(), "exponential");
            EXPECT_EQ(exponential->data_, exponential->uniformVector(0.5f));
            EXPECT_TRUE(exponential->has_rejection_);
        }

        TEST_F(TestGenericIntegratorSuite, TestConfigure) {
            exponential->p_nh_.setParam("alpha", 0.5f);
            exponential->p_nh_.setParam("rejection", 0.5f);
            exponential->has_rejection_ = true;

            EXPECT_TRUE(exponential->configure());
            EXPECT_TRUE(exponential->has_rejection_);

            exponential->p_nh_.deleteParam("rejection");

            EXPECT_TRUE(exponential->configure());
            EXPECT_FALSE(exponential->has_rejection_);
        }

        TEST_F(TestGenericIntegratorSuite, TestApplyWarn) {
            Eigen::VectorXf input(1);
            input << 0.5f;
            EXPECT_EQ(exponential->apply(input), exponential->data_);
        }

        TEST_F(TestGenericIntegratorSuite, TestApplyNoRejection) {
            Eigen::VectorXf input(2), output(2);
            exponential->alpha_ = 0.3f;
            input << 0.3f, 0.3f;
            output << 0.36f, 0.36f;
            exponential->has_rejection_ = false;

            EXPECT_EQ(output, exponential->apply(input));
        }

        TEST_F(TestGenericIntegratorSuite, TestApplyRejection) {
            Eigen::VectorXf input(2);
            input << 0.5f, 0.5f;
            exponential->has_rejection_ = true;
            exponential->rejection_ = 0.5f;
            EXPECT_EQ(exponential->data_, exponential->apply(input));
        }

        TEST_F(TestGenericIntegratorSuite, TestReset) {
            Eigen::VectorXf data(2);
            data << 0.5f, 0.5f;
            exponential->data_ = data;
            EXPECT_EQ(exponential->reset(), true);
            EXPECT_EQ(exponential->data_, exponential->uniformVector(0.5f));
        }

        TEST_F(TestGenericIntegratorSuite, TestSetAlpha) {
            exponential->setAlpha(2.0f);
            EXPECT_EQ(exponential->alpha_, fix_alpha);

            exponential->setAlpha(-1.0f);
            EXPECT_EQ(exponential->alpha_, fix_alpha);

            exponential->setAlpha(0.5f);
            EXPECT_EQ(exponential->alpha_, 0.5f);
        }

        TEST_F(TestGenericIntegratorSuite, TestSetRejection) {
            rosneuro_config_exponential config;
            exponential->setRejection(2.0f);
            EXPECT_EQ(exponential->rejection_, 0.5f);

            exponential->setRejection(0.4f);
            EXPECT_EQ(exponential->alpha_, fix_alpha);
            EXPECT_FALSE(exponential->has_rejection_);

            exponential->setRejection(2.0f);
            EXPECT_EQ(exponential->alpha_, fix_alpha);
            EXPECT_FALSE(exponential->has_rejection_);
        }
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_generic_integrator");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}