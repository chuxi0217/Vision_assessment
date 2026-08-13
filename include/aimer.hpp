#ifndef AUTO_AIM_AIMER_HPP
#define AUTO_AIM_AIMER_HPP

#include <Eigen/Dense>
#include "armor.hpp"

namespace auto_aim{
    class Aimer{
        private:
            double yaw_ = 0.0;
            double pitch_ = 0.0;
        public:
            double yaw() const;//get_yaw
            double pitch() const;//get_pitch
            void update(const Armor &armor);
    };
}



#endif //AUTO_AIM_AIMER_HPP