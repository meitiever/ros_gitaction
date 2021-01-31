
#ifndef SINEVA_SLAM_VSLAM_YESENSE_IMU_HPP_
#define SINEVA_SLAM_VSLAM_YESENSE_IMU_HPP_

#include <iostream>
#include <vector>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/noncopyable.hpp>
#include <boost/algorithm/hex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>

#include <glog/logging.h>
#include <sensors/imu-simple.h>
#include <sensors/crash_sensor.h>
#include <ros_imu_wheel/utility.h>
#include <ros_imu_wheel/analysis-data.h>
#include <ros_imu_wheel/buffered-async-serial.h>

using IMU = ::vslam::sensors::IMU_Simple;
using ODOM = ::vslam::sensors::Odometry_Simple;
using CRASH = ::vslam::sensors::Crash_Sensor;

using namespace ::vslam::utility;

namespace vslam {
    namespace drivers {

        class Yesense : boost::noncopyable {
          public:
            Yesense() {
                LOG(INFO) << "must call open later.";
            }

            Yesense(const std::string name, const int speed) : port_(name), baud_width_(speed) {
                serial_.open(port_, baud_width_);
                boost::this_thread::sleep_for(boost::chrono::microseconds{ 10 });
                boost::system::error_code error;
                serial_.flushSerialPort(BufferedAsyncSerial::flush_receive, error);

            }

            bool Open(const std::string name, const int speed) {
                port_ = name;
                baud_width_ = speed;
                serial_.open(port_, baud_width_);

                boost::this_thread::sleep_for(boost::chrono::microseconds{ 10 });
                boost::system::error_code error;
                serial_.flushSerialPort(BufferedAsyncSerial::flush_receive, error);
                return serial_.isOpen();
            }

            bool IsOpen() { return serial_.isOpen(); }

            bool IsEmpty() { return imu_cache_.empty(); }
            //从缓存中读取imu0数据   不含角速度
            std::vector<IMU> PopImu() {
                std::vector<IMU> tmp;
                {
                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                    tmp.clear();

                    for(const auto it : imu_cache_) { tmp.push_back(it); }

                    imu_cache_.clear();
                }
                return tmp;
            }
            //从缓存中读取imu1数据  包含角速度
            std::vector<IMU> PopImuOne() {
                std::vector<IMU> tmp;
                {
                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                    tmp.clear();

                    for(const auto it : imu_one_cache_) { tmp.push_back(it); }

                    imu_one_cache_.clear();
                }
                return tmp;
            }
            //从缓存中读取odom数据
            std::vector<ODOM> PopWheel() {
                std::vector<ODOM> tmp;
                {
                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                    tmp.clear();

                    for(const auto it : odom_cache_) { tmp.push_back(it); }

                    odom_cache_.clear();
                }
                return tmp;
            }
            //从缓存中读取碰撞传感器数据
            std::vector<CRASH> PopCrash() {
                std::vector<CRASH> tmp;
                {
                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                    tmp.clear();

                    for(const auto it : crash_cache_) { tmp.push_back(it); }

                    crash_cache_.clear();
                }
                return tmp;
            }
            //从缓存中读取相机触发时间数据  时间戳
            std::vector<double> PopCameraTime() {
                std::vector<double> tmp;
                {
                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                    tmp.clear();

                    for(const auto it : camera_time_cache_) { tmp.push_back(it); }

                    camera_time_cache_.clear();
                }
                return tmp;
            }

            void Start() {
                ReadAndValidate();
            }

            void AsyncStart() {
                //sleep(10);
                boost::this_thread::sleep_for(boost::chrono::microseconds{ 1000 });
                boost::thread(boost::bind(&Yesense::ReadAndValidate, this)).detach();

            }

            void  CameraStart() {
                serial_.write(camera_start_protocol_data, 11);
                boost::this_thread::sleep_for(boost::chrono::microseconds{ 1000 });
            }

            void CameraStop() {
                ROS_INFO("camera stop");
                serial_.write(camera_stop_protocol_data, 11);
            }

            size_t Size() {
                size_t size;
                {
                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                    size = imu_cache_.size();
                }
                return size;
            }

            void Stop() { stop_ = true; }

          private:
            // cost too much resource
            // std::string time = boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
            void ReadAndValidate() {
                try {
                    int imu_data_count_between_frames = 0;
                    int sample_time_last_four_digits_in_previous_frame = 0;
                    bool first_frame = true, data_available = false;
                    ptime pps_time = microsec_clock::local_time();

                    while(1) {
                        signal(SIGINT, SIG_DFL);

                        if(serial_.isOpen()) {
                            std::string str = serial_.readStringUntil(start_bytes_);

                            //解析imu0数据
                            if(str.length() == imu_bytes_) {
                                str = start_bytes_hex_ + str;

                                for(int i = 0; i < imu_length_; i++) {
                                    std::string out = boost::algorithm::unhex<std::string>({ str[2 * i], str[2 * i + 1] });
                                    imu_protocol_data[i] = (int)out[0];
                                }

                                IMU imu(HexToDecFromLowToHighBits(imu_protocol_data[7], imu_protocol_data[8], imu_protocol_data[9], imu_protocol_data[10]),
                                        HexToDecFromLowToHighBits(imu_protocol_data[11], imu_protocol_data[12], imu_protocol_data[13], imu_protocol_data[14]),
                                        HexToDecFromLowToHighBits(imu_protocol_data[15], imu_protocol_data[16], imu_protocol_data[17], imu_protocol_data[18]),

                                        HexToDecFromLowToHighBits(imu_protocol_data[21], imu_protocol_data[22], imu_protocol_data[23], imu_protocol_data[24]),
                                        HexToDecFromLowToHighBits(imu_protocol_data[25], imu_protocol_data[26], imu_protocol_data[27], imu_protocol_data[28]),
                                        HexToDecFromLowToHighBits(imu_protocol_data[29], imu_protocol_data[30], imu_protocol_data[31], imu_protocol_data[32]),

                                        HexToDecFromLowToHighBits(imu_protocol_data[35], imu_protocol_data[36], imu_protocol_data[37], imu_protocol_data[38]),
                                        HexToDecFromLowToHighBits(imu_protocol_data[41], imu_protocol_data[42], imu_protocol_data[43], imu_protocol_data[44])
                                       );
                                {
                                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                                    imu_cache_.push_back(imu);
                                }
                            }

                            //解析imu1数据
                            if(str.length() == imu_one_bytes_) {
                                str = start_bytes_hex_ + str;

                                for(int i = 0; i < imu_one_length_; i++) {
                                    std::string out = boost::algorithm::unhex<std::string>({ str[2 * i], str[2 * i + 1] });
                                    imu_one_protocol_data[i] = (int)out[0];
                                }

                                IMU imu_one(HexToDecFromLowToHighBits(imu_one_protocol_data[7], imu_one_protocol_data[8], imu_one_protocol_data[9],   imu_one_protocol_data[10]),
                                            HexToDecFromLowToHighBits(imu_one_protocol_data[11], imu_one_protocol_data[12], imu_one_protocol_data[13], imu_one_protocol_data[14]),
                                            HexToDecFromLowToHighBits(imu_one_protocol_data[15], imu_one_protocol_data[16], imu_one_protocol_data[17], imu_one_protocol_data[18]),

                                            HexToDecFromLowToHighBits(imu_one_protocol_data[21], imu_one_protocol_data[22], imu_one_protocol_data[23], imu_one_protocol_data[24]),
                                            HexToDecFromLowToHighBits(imu_one_protocol_data[25], imu_one_protocol_data[26], imu_one_protocol_data[27], imu_one_protocol_data[28]),
                                            HexToDecFromLowToHighBits(imu_one_protocol_data[29], imu_one_protocol_data[30], imu_one_protocol_data[31], imu_one_protocol_data[32]),
                                            //温度
                                            HexToDecFromLowToHighBits(imu_one_protocol_data[35], imu_one_protocol_data[36], imu_one_protocol_data[37], imu_one_protocol_data[38]),

                                            HexToDecFromLowToHighBits(imu_one_protocol_data[41], imu_one_protocol_data[42], imu_one_protocol_data[43], imu_one_protocol_data[44]),
                                            HexToDecFromLowToHighBits(imu_one_protocol_data[47], imu_one_protocol_data[48], imu_one_protocol_data[49], imu_one_protocol_data[50])
                                           );
                                {
                                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                                    imu_one_cache_.push_back(imu_one);
                                }
                            }

                            //解析里程计数据
                            if(str.length() == odom_bytes_) {
                                str = start_bytes_hex_ + str;

                                for(int i = 0; i < odom_length_; i++) {
                                    std::string out = boost::algorithm::unhex<std::string>({ str[2 * i], str[2 * i + 1] });
                                    odom_protocol_data[i] = (int)out[0];
                                }

                                ODOM odom(240, 35, 1); /// 1 for angular speed.
                                odom.SetVel(HexToDecFromLowToHighBits(odom_protocol_data[7], odom_protocol_data[8], odom_protocol_data[9], odom_protocol_data[10]),
                                            HexToDecFromLowToHighBits(odom_protocol_data[11], odom_protocol_data[12], odom_protocol_data[13], odom_protocol_data[14]),
                                            HexToDecFromLowToHighBits(odom_protocol_data[15], odom_protocol_data[16], odom_protocol_data[17], odom_protocol_data[18]),
                                            HexToDecFromLowToHighBits(odom_protocol_data[19], odom_protocol_data[20], odom_protocol_data[21], odom_protocol_data[22]),

                                            HexToDecFromLowToHighBits(odom_protocol_data[25], odom_protocol_data[26], odom_protocol_data[27], odom_protocol_data[28]),
                                            HexToDecFromLowToHighBits(odom_protocol_data[31], odom_protocol_data[32], odom_protocol_data[33], odom_protocol_data[34])
                                           );
                                {
                                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                                    odom_cache_.push_back(odom);
                                }
                            }

                            //解析碰撞数据
                            if(str.length() == crash_bytes_) {
                                str = start_bytes_hex_ + str;

                                for(int i = 0; i < crash_length_; i++) {
                                    std::string out = boost::algorithm::unhex<std::string>({ str[2 * i], str[2 * i + 1] });
                                    crash_protocol_data[i] = (int)out[0];
                                }

                                CRASH crash(HexToDecFromLowToHighBits(crash_protocol_data[7], crash_protocol_data[8], crash_protocol_data[9], crash_protocol_data[10]),
                                            HexToDecFromLowToHighBits(crash_protocol_data[11], crash_protocol_data[12], crash_protocol_data[13], crash_protocol_data[14]),

                                            HexToDecFromLowToHighBits(crash_protocol_data[17], crash_protocol_data[18], crash_protocol_data[19], crash_protocol_data[20]),

                                            HexToDecFromLowToHighBits(crash_protocol_data[23], crash_protocol_data[24], crash_protocol_data[25], crash_protocol_data[26])
                                           );
                                {
                                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                                    crash_cache_.push_back(crash);
                                }

                            }

                            //解析相机时间戳数据
                            if(str.length() == camera_bytes_) {
                                str = start_bytes_hex_ + str;

                                for(int i = 0; i < camera_length_; i++) {
                                    std::string out = boost::algorithm::unhex<std::string>({ str[2 * i], str[2 * i + 1] });
                                    camera_protocol_data[i] = (int)out[0];
                                }


                                double cameraTime =
                                                HexToDecFromLowToHighBits(camera_protocol_data[9], camera_protocol_data[10], camera_protocol_data[11], camera_protocol_data[12]) ;
                                {
                                    boost::lock_guard<boost::mutex> lock(cache_lock_);
                                    camera_time_cache_.push_back(cameraTime);
                                }

                            }

                        }

                        if(stop_) {
                            serial_.close();
                            break;
                        }
                    }
                } catch(boost::system::system_error &e) {
                    std::cout << "Error: " << e.what() << std::endl;
                    serial_.close();
                    return;
                }

                LOG(INFO) << "IMU read frame stoped.";
            }

            boost::mutex cache_lock_;
            const std::string start_bytes_ = "YS";
            const std::string start_bytes_hex_ = "5953";


            const int imu_bytes_ = 90;
            const int imu_one_bytes_ = 102;
            const int odom_bytes_ = 70;
            const int crash_bytes_ = 54;
            const int camera_bytes_ = 26;

            const int imu_length_ = 47;
            const int imu_one_length_ = 53;
            const int odom_length_ = 37;
            const int crash_length_ = 29;
            const int camera_length_ = 15;

            bool stop_ = false;
            BufferedAsyncSerial serial_;
            std::vector<sensors::IMU_Simple> imu_one_cache_;
            std::vector<sensors::IMU_Simple> imu_cache_;
            std::vector<sensors::Odometry_Simple> odom_cache_;
            std::vector<sensors::Crash_Sensor> crash_cache_;
            std::vector<double> camera_time_cache_;
            //  std::vector<sensors::Camera_Time> camera_time_cache_;
            std::string port_;
            int baud_width_;
            unsigned char imu_protocol_data[47] = {
                0x59, 0x53, 0x01, 0x00, 0x28,
                0x10, 0x0C, 0x30, 0xDF, 0x18, 0x00, 0x40, 0xD8, 0xF6, 0xFF, 0xF0, 0x36, 0x97, 0x00,  //18
                0x40, 0x0C, 0xE0, 0xFB, 0xC9, 0xFF, 0x80, 0x2D, 0x71, 0xFF, 0xB0, 0x3C, 0xFF, 0xFF,  //32
                0x51, 0x04, 0x70, 0x8E, 0x01, 0x00,  //38
                0x52, 0x04, 0xb8, 0xE2, 0x68, 0x01,  //44
                0xB6, 0x5E,
            };

            unsigned char imu_one_protocol_data[53] = {
                0x59, 0x53, 0x01, 0x00, 0x28,
                0x10, 0x0C, 0x30, 0xDF, 0x18, 0x00, 0x40, 0xD8, 0xF6, 0xFF, 0xF0, 0x36, 0x97, 0x00,  //18
                0x40, 0x0C, 0xE0, 0xFB, 0xC9, 0xFF, 0x80, 0x2D, 0x71, 0xFF, 0xB0, 0x3C, 0xFF, 0xFF,  //32
                0x53, 0x04, 0x70, 0x8E, 0x01, 0x00,  //38
                0x51, 0x04, 0x70, 0x8E, 0x01, 0x00,  //44
                0x52, 0x04, 0xb8, 0xE2, 0x68, 0x01,  //50
                0xB6, 0x5E,
            };

            unsigned char odom_protocol_data[37] = {
                0x59, 0x53, 0x01, 0x00, 0x1E,  //4
                0x72, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //22
                0x51, 0x04, 0x70, 0x8E, 0x01, 0x00,  //28
                0x52, 0x04, 0xb8, 0xE2, 0x68, 0x01,  //34
                0xB6, 0x5E,
            };
            unsigned  char crash_protocol_data[29] = {
                0x59, 0x53, 0xb4, 0x04, 0x16,  //4
                0x73, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x51,
                0x04, 0x00, 0x00, 0x00, 0x00, //20
                0x52, 0x04, 0xC3, 0xA3, 0x02, 0x00,  //26
                0x5C, 0x69,
            };

            //开启触发数据
            char camera_start_protocol_data[11] = {
                0x59, 0x53, 0x10, 0x04, 0x16,  //4
                0x74, 0x54, 0x01, 0x01,
                0x5C, 0x69,
            };
            //关闭触发数据
            char camera_stop_protocol_data[11] = {
                0x59, 0x53, 0x10, 0x04, 0x16,  //4
                0x74, 0x54, 0x01, 0x00,
                0x5C, 0x69,
            };

            unsigned  char camera_protocol_data[15] = {
                0x59, 0x53, 0x00, 0x00, 0x16,  //4
                0x74, 0x06, 0x52, 0x04, 0x00, 0x00, 0x01,
                0x01,  0x5C, 0x69,
            };

        };
    }  // namespace drivers
}
#endif  // SINEVA_SLAM_VSLAM_YESENSE_IMU_HPP_
