#ifndef ROSNEURO_INTEGRATOR_EXPONENTIAL_H_
#define ROSNEURO_INTEGRATOR_EXPONENTIAL_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <gtest/gtest_prod.h>
#include "rosneuro_integrator/GenericIntegrator.h"
#include "rosneuro_integrator_exponential/ExponentialConfig.h"


namespace rosneuro {
	namespace integrator {
        using rosneuro_config_exponential = rosneuro_integrator_exponential::ExponentialConfig;
        using dyncfg_exponential          = dynamic_reconfigure::Server<rosneuro_config_exponential>;
        class Exponential : public GenericIntegrator {

            public:
                Exponential(void);
                ~Exponential(void);

                bool configure(void);
                Eigen::VectorXf apply(const Eigen::VectorXf& input);
                bool reset(void);
                void setAlpha(float value);
                void setRejection(std::vector<float> values);
                void setInitPercentual(std::vector<float> init_percentual);
                std::vector<float> getInitPrecentual(void);

            private:
                Eigen::VectorXf uniformVector(float value);
                void onRequestReconfigure(rosneuro_config_exponential &config, uint32_t level);

                ros::NodeHandle p_nh_;
                float alpha_;
                std::vector<float> rejections_;
                const float alpha_default_ = 0.98f;
                Eigen::Vector2f data_;
                std::vector<float> init_percentual_;

                dyncfg_exponential reconfigure_srv_;
                dyncfg_exponential::CallbackType reconfigure_callback_type_;

                FRIEND_TEST(TestGenericIntegratorSuite, TestConstructor);
                FRIEND_TEST(TestGenericIntegratorSuite, TestConfigure);
                FRIEND_TEST(TestGenericIntegratorSuite, TestApplyWarn);
                FRIEND_TEST(TestGenericIntegratorSuite, TestApplyNoRejection);
                FRIEND_TEST(TestGenericIntegratorSuite, TestApplyRejection);
                FRIEND_TEST(TestGenericIntegratorSuite, TestReset);
                FRIEND_TEST(TestGenericIntegratorSuite, TestSetAlpha);
                FRIEND_TEST(TestGenericIntegratorSuite, TestSetRejection);
        };
        PLUGINLIB_EXPORT_CLASS(rosneuro::integrator::Exponential, rosneuro::integrator::GenericIntegrator);
	}
}

#endif
