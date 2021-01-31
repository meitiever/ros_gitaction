
#ifndef SINEVA_SLAM_VSLAM_MODEL_SENSORS_IMU_SIMPLE_H_
#define SINEVA_SLAM_VSLAM_MODEL_SENSORS_IMU_SIMPLE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/date_time.hpp>

using namespace boost::posix_time;

using ptime = boost::posix_time::ptime;

namespace vslam {
    namespace sensors {
        //加速度
        class Accelerator {
          public:
            Accelerator() {}
            Accelerator(float ax, float ay, float az)
                : ax_(ax), ay_(ay), az_(az) { }
            float GetAx() { return ax_; }
            float GetAy() { return ay_; }
            float GetAz() { return az_; }
            void SetAx(float value) { ax_ = value; }
            void SetAy(float value) { ay_ = value; }
            void SetAz(float value) { az_ = value; }
          private:
            float ax_, ay_, az_;
        };
        //角速度
        class AngularVelocity {
          public:
            AngularVelocity() {}
            AngularVelocity(float wx, float wy, float wz)
                : wx_(wx), wy_(wy), wz_(wz) { }
            float GetWx() { return wx_; }
            float GetWy() { return wy_; }
            float GetWz() { return wz_; }
            void SetWx(float value) { wx_ = value; }
            void SetWy(float value) { wy_ = value; }
            void SetWz(float value) { wz_ = value; }
          private:
            float wx_, wy_, wz_;
        };
        //欧拉角
        class EulerAngle {
          public:
            EulerAngle() {}
            EulerAngle(float roll, float pitch, float yaw)
                : pitch_(pitch), roll_(roll), yaw_(yaw) { }
            float GetPitch() { return pitch_; }
            float GetRoll() { return roll_; }
            float GetYaw() { return yaw_; }
            void SetPitcch(float value) { pitch_ = value; }
            void SetRoll(float value) { roll_ = value; }
            void SetYaw(float value) { yaw_ = value; }
          private:
            float pitch_, roll_, yaw_;
        };

        class IMU_Simple {
          public:
            IMU_Simple() :
                acc_(scale * 0, scale * 0, scale * 0), elag_(M_PI * 0 / (1000000.0 * 180), M_PI * 0 / (1000000.0 * 180), M_PI * 0 / (1000000.0 * 180))
            {   }
            //初始化加速度和角度
            IMU_Simple(int ax, int ay, int az,
                       int roll, int pitch, int yaw,
                       int sample_td, int sync_td) :
                acc_(scale * ax, scale * ay, scale * az), elag_(M_PI * roll / (1000000.0 * 180), M_PI * pitch / (1000000.0 * 180), M_PI * yaw / (1000000.0 * 180)) {

                sample_td_ = static_cast<double>(sample_td) / 1000.0;
                sync_td_ = static_cast<double>(sync_td) / 1000.0;
                ros_current_time_ = ros::Time::now();
                yaw_ = yaw;
                roll_ = roll;
                pitch_ = pitch;
            }
            //初始化加速度和角速度
            IMU_Simple(int ax, int ay, int az,
                       int aglv_x, int aglv_y, int aglv_z, int temp,
                       int sample_td, int sync_td) :
                acc_(scale1 * ax, scale1 * ay, scale1 * az), aglv_(aglv_x * M_PI / (1000000 * 180), aglv_y * M_PI / (1000000 * 180), aglv_z * M_PI / (1000000 * 180)), temp_(temp) {
                sample_td_ = static_cast<double>(sample_td) / 1000.0;
                sync_td_ = static_cast<double>(sync_td) / 1000.0;
                ros_current_time_ = ros::Time::now();

                wx_ = aglv_x;
                wy_ = aglv_y;
                wz_ = aglv_z;
            }

            Accelerator GetAcceleration() {
                return acc_;
            }
            AngularVelocity GetAngularVelocity() {
                return aglv_;
            }
            EulerAngle GetEulerAngle() {
                return elag_;
            }

            int getYaw() {
                return yaw_;
            }
            int getRoll() {
                return roll_;
            }
            int getPitch() {
                return pitch_;
            }

            double getSyncTd() {
                return sync_td_;
            }
            const ros::Time GetSyncTime() {
                ros::Time t(sync_td_);
                return t;
            }
          private:
            float scale = 9.8 / 10000000;
            float scale1 = 9.8 / 10000000;
            Accelerator acc_;
            AngularVelocity aglv_;
            EulerAngle elag_;
            Eigen::Quaterniond q_;
            ptime UTC_;
            ros::Time ros_current_time_;
            double sample_td_;
            double sync_td_;
            int roll_, pitch_, yaw_;
            int wx_, wy_, wz_, temp_;
        };
    }
}

#endif //SINEVA_SLAM_VSLAM_MODEL_SENSORS_IMU_SIMPLE_H_
