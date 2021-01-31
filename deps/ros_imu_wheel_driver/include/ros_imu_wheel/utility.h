#ifndef SINEVA_SLAM_VSLAM_YESENSE_UTILITY_H
#define SINEVA_SLAM_VSLAM_YESENSE_UTILITY_H

#include <thread>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/hex.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensors/imu-simple.h>
#include <sensors/odometry.h>

#include <math.h>
#include <opencv2/opencv.hpp>

namespace vslam {
    namespace utility {

        //弧度归一化  弧度角保持在 -π ~ π 之间
        double NormalizeAngle(double angle) {
            double a = fmod(angle + M_PI, 2.0 * M_PI);

            if(a < 0.0) {
                a += (2.0 * M_PI);
            }

            return a - M_PI;
        }
        //构造imu0 msg
        sensor_msgs::Imu ImuSimpleToRosMsg(const std::string &frame_id, vslam::sensors::IMU_Simple &val0, vslam::sensors::IMU_Simple &val, const ros::Duration &offset) {
            sensor_msgs::Imu msg;
            msg.header.frame_id = frame_id;
            msg.header.stamp = val.GetSyncTime() + offset;
            auto acc  = val.GetAcceleration();
            auto rpy0 = val0.GetEulerAngle();
            auto rpy  = val.GetEulerAngle();

            msg.linear_acceleration.x = acc.GetAx();
            msg.linear_acceleration.y = acc.GetAy();
            msg.linear_acceleration.z = acc.GetAz();

            // msg.angular_velocity.x = (rpy.GetRoll()  - rpy0.GetRoll())/ 0.01;
            // msg.angular_velocity.y = (rpy.GetPitch() - rpy0.GetPitch())/ 0.01;
            // msg.angular_velocity.z = (rpy.GetYaw()   - rpy0.GetYaw())/ 0.01;
            //对角度做归一化处理
            msg.angular_velocity.x = NormalizeAngle(rpy.GetRoll()  - rpy0.GetRoll()) / 0.01;
            msg.angular_velocity.y = NormalizeAngle(rpy.GetPitch() - rpy0.GetPitch()) / 0.01;
            msg.angular_velocity.z = NormalizeAngle(rpy.GetYaw()   - rpy0.GetYaw()) / 0.01;

            Eigen::Matrix3d R;
            R = Eigen::AngleAxisd(rpy.GetRoll(), ::Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(rpy.GetPitch(), ::Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy.GetYaw(), ::Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q;
            q = R;
            q = q.normalized();
            msg.orientation.x = q.x();
            msg.orientation.y = q.y();
            msg.orientation.z = q.z();
            msg.orientation.w = q.w();

            return msg;
        }

        //构造imu1 msg
        sensor_msgs::Imu ImuOneSimpleToRosMsg(const std::string &frame_id, vslam::sensors::IMU_Simple &val, geometry_msgs::Quaternion q, const ros::Duration &offset) {
            sensor_msgs::Imu msg;
            msg.header.frame_id = frame_id;
            msg.header.stamp = val.GetSyncTime() + offset;
            auto acc  = val.GetAcceleration();
            auto vel = val.GetAngularVelocity();


            msg.linear_acceleration.x = acc.GetAx();
            msg.linear_acceleration.y = acc.GetAy();
            msg.linear_acceleration.z = acc.GetAz();


            msg.angular_velocity.x = vel.GetWx();
            msg.angular_velocity.y = vel.GetWy();
            msg.angular_velocity.z = vel.GetWz();


            msg.orientation = q;
            return msg;
        }
        geometry_msgs::TwistStamped OdometrySimpleToRosMsg(const std::string &frame_id, vslam::sensors::Odometry_Simple &val, const ros::Duration &offset) {
            geometry_msgs::TwistStamped msg;
            msg.header.frame_id = frame_id;
            msg.header.stamp = val.GetSyncTime() + offset;
            msg.twist.linear.x = val.GetLeftVel();
            msg.twist.linear.y = val.GetRightVel();
            return msg;
        }

        //msg.seq  0:-->no crash   1-->left crash  2-->right crash   3-->left and right crash
        std_msgs::Header CrashSensorToRosMsg(const std::string &frame_id, vslam::sensors::Crash_Sensor &val, const ros::Duration &offset) {
            std_msgs::Header msg;
            msg.frame_id = frame_id;
            msg.stamp = val.GetSyncTime() + offset;
            int left = val.GetLeftVel();
            int right = val.GetRightVel();

            if(left == 0 && right == 0) {
                msg.seq = 0;
            } else
                if(left == 1 && right == 0) {
                    msg.seq = 1;
                }

                else
                    if(left == 0 && right == 1) {
                        msg.seq = 2;
                    } else {
                        msg.seq = 3;
                    }

            return msg;
        }

        std::string AsciiToHex(std::string value) {
            std::stringstream ss;
            std::string results;

            for(int i = 0; i < value.size(); ++i) {
                ss << std::hex << (int)value[i];
            }

            results += ss.str();
            return results;
        }

        inline int HexToDecFromLowToHighBits(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4) {
            std::stringstream ss;
            ss << boost::format("%|02X|") % (int)(unsigned char)c4;
            ss << boost::format("%|02X|") % (int)(unsigned char)c3;
            ss << boost::format("%|02X|") % (int)(unsigned char)c2;
            ss << boost::format("%|02X|") % (int)(unsigned char)c1;
            std::string res = ss.str();
            unsigned int x;
            std::stringstream s1;
            s1 << std::hex << res;
            s1 >> x;
            return static_cast<int>(x);
        }

    }
}
#endif  // SINEVA_SLAM_VSLAM_YESENSE_UTILITY_H
