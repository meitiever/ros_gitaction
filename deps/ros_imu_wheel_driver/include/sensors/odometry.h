
#ifndef SINEVA_SLAM_VSLAM_MODEL_SENSORS_ODOMETRY_SIMPLE_H_
#define SINEVA_SLAM_VSLAM_MODEL_SENSORS_ODOMETRY_SIMPLE_H_

#include <boost/date_time.hpp>

using namespace boost::posix_time;

using ptime = boost::posix_time::ptime;

namespace vslam {
	namespace sensors {
		class Odometry_Simple {
      public:
        /// Param: speed_mode
        /// 0 - linear speed
        /// 1 - angular speed
        Odometry_Simple(double dis_wheels, double radius_wheels, int speed_mode) : dis_wheels_(dis_wheels), radius_wheels_(radius_wheels) { 
          if(!speed_mode) WHEEL_PERIMETER = 220;
        };
        void SetVel(int vel_left, int ldir, int vel_right, int rdir, int sample_td, int sync_td) {
          vel_left_ = vel_left;
          vel_right_ = vel_right;
          if(std::abs(rdir) > 0.001) r_dir_ = 1;
          if(std::abs(ldir) > 0.001) l_dir_ = 1;
          sample_td_ = static_cast<double>(sample_td) / 1000.0;
          sync_td_ = static_cast<double>(sync_td) / 1000.0;
          ros_current_time_ = ros::Time::now();
        }
        const ros::Time GetSyncTime()
        {
          ros::Time t(sync_td_);
          return t;
        }
        double GetLeftVel() {
          if(!vel_left_) return 0.0;
          return l_dir_ * vel_left_ * WHEEL_PERIMETER * SAMPLE_FREQ / WHEEL_ENCODE_COUNT;
        }
        double GetRightVel() {
          if(!vel_right_) return 0.0;          
          return r_dir_ * vel_right_ * WHEEL_PERIMETER * SAMPLE_FREQ / WHEEL_ENCODE_COUNT;
        }
      private:
        float scale = 1.0;
        double dis_wheels_, radius_wheels_;
        int vel_left_, vel_right_, r_dir_ = -1, l_dir_ = -1;
        ptime UTC_;
        ros::Time ros_current_time_;
        double sample_td_;
        double sync_td_;
        double WHEEL_PERIMETER = 2 * M_PI; // radius - 35mm
        const int WHEEL_ENCODE_COUNT = 260;        
        const int SAMPLE_FREQ = 100;
		};
	}
}

#endif //SINEVA_SLAM_VSLAM_MODEL_SENSORS_ODOMETRY_SIMPLE_H_
