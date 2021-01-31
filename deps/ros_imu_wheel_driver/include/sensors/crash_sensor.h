
#ifndef SINEVA_SLAM_VSLAM_MODEL_SENSORS_CRASH_SENSOR_H_
#define SINEVA_SLAM_VSLAM_MODEL_SENSORS_CRASH_SENSOR_H_

#include <boost/date_time.hpp>

using namespace boost::posix_time;

using ptime = boost::posix_time::ptime;

namespace vslam {
    namespace sensors {
        class Crash_Sensor {
          public:
            Crash_Sensor(int left, int right, double sample_td, double sync_td): left_(left), right_(right) {
                sample_td_ = static_cast<double>(sample_td) / 1000.0;
                sync_td_ = static_cast<double>(sync_td) / 1000.0;
                ros_current_time_ = ros::Time::now();
            }

            void SetVel(int left, int right, int sample_td, int sync_td) {
                left_ = left;
                right_ = right;
                sample_td_ = static_cast<double>(sample_td) / 1000.0;
                sync_td_ = static_cast<double>(sync_td) / 1000.0;
                ros_current_time_ = ros::Time::now();
            }

            const ros::Time GetSyncTime() {
                ros::Time t(sync_td_);
                return t;
            }
            double GetLeftVel() {
                return left_;
            }
            double GetRightVel() {
                return right_;
            }
          private:
            int left_;
            int right_;
            ptime UTC_;
            ros::Time ros_current_time_;
            double sample_td_;
            double sync_td_;
        };
    }
}

#endif //SINEVA_SLAM_VSLAM_MODEL_SENSORS_CRASH_SENSOR_H_
