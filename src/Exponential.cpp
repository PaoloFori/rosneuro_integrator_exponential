#include "rosneuro_integrator_exponential/Exponential.h"

namespace rosneuro {
	namespace integrator {
        Exponential::Exponential(void) : p_nh_("~") {
            this->setName("integrator_exponential");
            this->data_ = this->uniformVector(0.5f);
        }

        Exponential::~Exponential(void) {
        }

        bool Exponential::configure(void) {
            float alpha;
            this->p_nh_.param<float>("alpha", alpha, this->alpha_default_);
            this->setAlpha(alpha);

            int nclasses;
            this->p_nh_.param<int>("nclasses", nclasses, 2);

            // thresholds rejection probabilities
            std::vector<float> ths_rejection;
            this->p_nh_.param<std::vector<float>>("thresholds_rejection", ths_rejection, std::vector<float>(2, 0.5f));
            if(ths_rejection.size() != nclasses){
                ROS_ERROR("[%s] Parameter 'thresholds_rejection' must have 2 values (2-class problem)", this->name().c_str());
                return false;
            }
            this->setRejection(ths_rejection);

            std::vector<float> init_percentual;
            this->p_nh_.param<std::vector<float>>("init_percentual", init_percentual, std::vector<float>(2, 0.5f));
            if(init_percentual.size() != nclasses){
                ROS_ERROR("[%s] Parameter 'init_percentual' must have 2 values (2-class problem)", this->name().c_str());
                return false;
            }
            if(static_cast<float>(std::accumulate(init_percentual.begin(), init_percentual.end(), 0.0)) != 1.0f){
                ROS_ERROR("[%s] Parameter 'init_percentual' must sum to 1.0, it is %f", this->name().c_str(), std::accumulate(init_percentual.begin(), init_percentual.end(), 0.0));
                return false;
            }
            this->setInitPercentual(init_percentual);

            this->reconfigure_callback_type_ = boost::bind(&Exponential::onRequestReconfigure, this, _1, _2);
            this->reconfigure_srv_.setCallback(this->reconfigure_callback_type_);

            return true;
        }

        Eigen::VectorXf Exponential::apply(const Eigen::VectorXf& input) {
            
            if(input.size() != 2) {
                ROS_WARN("[%s] Input size is not 2: only 2-class input is allowed", this->name().c_str());
                return this->data_;
            }

            Eigen::Index maxIndex;
            input.maxCoeff(&maxIndex);
            if(input(maxIndex) > this->rejections_.at(maxIndex))
                return this->data_;


            Eigen::VectorXf new_input = Eigen::VectorXf::Zero(2);
            new_input(maxIndex) = 1.0f;

            this->data_ = this->data_ * this->alpha_ + new_input * (1 - this->alpha_);
            return this->data_;
        }

        Eigen::VectorXf Exponential::getData(void) {
            return this->data_;
        }

        bool Exponential::reset(void) {
            this->data_ = this->uniformVector(0.5f);
            return true;
        }

        Eigen::VectorXf Exponential::uniformVector(float value) {
            return Eigen::Vector2f::Constant(value);
        }

        void Exponential::setAlpha(float value) {
            if(value < 0.0f | value > 1.0f) {
                this->alpha_ = this->alpha_default_;
                ROS_INFO("[%s] Alpha value is not legal (alpha=%f). Alpha set to the default value (alpha=%f)",
                         this->name().c_str(), value, this->alpha_);
            } else {
                this->alpha_ = value;
                ROS_INFO("[%s] Alpha set to %f", this->name().c_str(), this->alpha_);
            }
        }

        void Exponential::setRejection(std::vector<float> values) {
            bool valid_values = true;
            for(auto val : values){
                if(val < 0.5f | val > 1.0f){
                    valid_values = false;
                    ROS_ERROR("[%s] Rejection value is not legal (rejection=%f)", this->name().c_str(), val);
                    break;
                }
            }
            if(valid_values){
                this->rejections_ = values;
            }
        }

        void Exponential::onRequestReconfigure(rosneuro_config_exponential &config, uint32_t level) {
            if( std::fabs(config.alpha - this->alpha_) > 0.00001) {
                this->setAlpha(config.alpha);
            }
        }

        void Exponential::setInitPercentual(std::vector<float> init_percentual){
            this->init_percentual_ = init_percentual;
        }

        std::vector<float> Exponential::getInitPrecentual(void){
            return this->init_percentual_;
        }
	}
}
