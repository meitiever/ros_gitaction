
#include <sstream>
#include <signal.h>
#include <limits>
#include <unistd.h>
#include <sys/stat.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/lexical_cast.hpp>
#include <map>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/TwistStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros_imu_wheel/yesense.hpp>
#include <D_mipicam.h>
#include <kbhit.h>

const constexpr int BUFFERS_NUM = 100;

#define VCOS_ALIGN_DOWN(p,n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p,n) VCOS_ALIGN_DOWN((ptrdiff_t)(p)+(n)-1,(n))


using namespace std;

namespace ros_imu_wheel {
    class Ros_Sineva_IMU_Node {
      public:
        Ros_Sineva_IMU_Node() : node_("~") {
            node_.param("port", device_name_, std::string("/dev/ttyUSB4"));
            node_.param("imu_topic", imu_topic_, std::string("/imu0/data_raw")); // imu0
            node_.param("imu_one_topic", imu_one_topic_, std::string("/imu1/data_raw")); //imu1
            node_.param("odom_topic", odom_topic_, std::string("/wheel"));
            node_.param("crash_topic", crash_topic_, std::string("/xiaomi/crash_sensor"));
            node_.param("camera_pub_topic", camera_pub_topic_, std::string("/raspicam_node/camera_image"));
            node_.param("camera_compressed_pub_topic", camera_compressed_pub_topic_, std::string("/raspicam_node/camera_compressed_image"));

            node_.param("frame_id", frame_id_, std::string("base_link"));
            node_.param("speed", speed_, 115200);
            speed = speed_;
            node_.param("frequency", fps_, 100);
            fps = fps_;
            node_.param("base_line", base_line, 0.24);
            node_.param("radius", radius, 0.07);
            first_msgs = true;
            first_buffer = true;
            node_.param("dump", dump, false);
            node_.param("output", o_path, std::string("/home/nuc/Documents/output.txt"));


            node_.param("camera_sub_topic", camera_sub_topic_, std::string("image"));
            node_.param("camera_compressed_sub_topic", camera_compressed_sub_topic_, std::string("image/compressed"));
            std::cout  << "Parameter - image Topic name: " << camera_sub_topic_  << std::endl;
            ros::Subscriber s1 = node_.subscribe(camera_sub_topic_, 1, &Ros_Sineva_IMU_Node::ImageCallback, this);
            ros::Subscriber s2 = node_.subscribe(camera_compressed_sub_topic_, 1, &Ros_Sineva_IMU_Node::CompressedImageCallback, this);

            cam_interface.camera_num = -1;//0or1 for CM
            struct format defaultfmt;
            defaultfmt.width = 640;
            defaultfmt.height = 480;
            defaultfmt.framerate = 120;
            cout << "相机初始化--begin" << endl;
            int res = D_init_camera_ex(&camera_instance, cam_interface, &defaultfmt);

            if(res) {
                printf("init camera status = %d\n", res);
            }

            if(dump)
            { ofs.open(o_path, std::ios::out | std::ios::app); }
        }

        ~Ros_Sineva_IMU_Node() {
            kbhit.releaseHit();
            std::cout << "release" << std::endl;

            if(ofs.is_open())
            { ofs.close(); }
        }

        bool Grab_And_Send_Imu() {
            //读取imu0数据
            std::vector<IMU> imu_msg = ports.PopImu();

            std::cout << "imu_msgsize()---->" << imu_msg.size() << std::endl;

            if((imu_msg.size() == 5)   && cou_ < 5) {
                cou_++;
                std::cout << "第" << cou_ << "次等于5" << std::endl;
            }

            if(cou_ < 5) {
                std::cout << "不做处理" << std::endl;
                //数据稳定之前将接收的时间戳和图片都丢弃
                camera_time_list.clear();
                cout << camera_time_list.size() << endl;
                image_list.clear();

            }

            if(!imu_msg.empty() && cou_ >= 5) {
                for(int i = 0; i < imu_msg.size(); i++) {
                    if(first_msgs) {
                        time_offset = ros::Time::now() - imu_msg.at(i).GetSyncTime();
                        cout << std::fixed << "第一次imu时的时间------》" << ros::Time::now().toSec() << "---offset----" << time_offset.toSec() << endl;

                        if(time_offset.toSec() > 0.0) {
                            first_msgs = false;
                            last_measurement = imu_msg.at(i);
                            imu_first_time = ros::Time::now();  //记录下首次ros time
                        }

                        continue;
                    }

                    sensor_msgs::Imu msg = ImuSimpleToRosMsg(frame_id_, last_measurement, imu_msg.at(i), time_offset);
                    //保存imu0的时间戳和四元素  时间戳使用串口接受
                    qt_map.insert(pair<double, geometry_msgs::Quaternion>(imu_msg.at(i).getSyncTd(), msg.orientation));
                    last_measurement = imu_msg.at(i);
                    imu_pub.publish(msg);
                    LOG_EVERY_N(INFO, 1000) << "receiving imu message." << std::endl;

                }
            } else {
                LOG(ERROR) << "Grab imu data failed, port: " << device_name_ << " speed: " << speed;
            }

            //读取imu1数据
            auto imu_one_msg = ports.PopImuOne();
            std::cout << "imu_one_msg()---->" << imu_one_msg.size() << std::endl;

            if(!imu_one_msg.empty() && cou_ >= 5) {
                for(int i = 0; i < imu_one_msg.size(); i++) {
                    //找到时间戳距离最近的imu0的四元素
                    double syns_td = imu_one_msg.at(i).getSyncTd();
                    auto it0 = qt_map.begin();

                    while(it0 != qt_map.end()) {
                        //   ts = it0->first;
                        if(fabs(it0->first - syns_td) <= 5) {
                            // geometry_msgs::Quaternion q0 = it0->second;
                            sensor_msgs::Imu msg_one = ImuOneSimpleToRosMsg(frame_id_, imu_one_msg.at(i),  it0->second, time_offset);
                            imu_one_pub.publish(msg_one);
                            break;
                        } else {
                            //迭代器指向下一个元素位置
                            ++it0;
                        }
                    }

                    LOG_EVERY_N(INFO, 1000) << "receiving imu1 message." << std::endl;

                }
            } else {
                LOG(ERROR) << "Grab imu1 data failed, port: " << device_name_ << " speed: " << speed;
            }

            //读取odom数据
            const auto odom_msg = ports.PopWheel();

            if(!odom_msg.empty() && cou_ >= 5) {
                for(auto item : odom_msg) {
                    geometry_msgs::TwistStamped msg = OdometrySimpleToRosMsg(frame_id_, item, time_offset);
                    odom_pub.publish(msg);
                    LOG_EVERY_N(INFO, 1000) << "receiving odometry message." << std::endl;

                }
            } else {
                LOG(ERROR) << "Grab odom data failed, port: " << device_name_ << " speed: " << speed;
            }

            //读取碰撞数据
            const auto crash_msg = ports.PopCrash();
            LOG_EVERY_N(INFO, 1000) << crash_msg.size() << std::endl;

            if(!crash_msg.empty() && cou_ >= 5) {
                for(auto item : crash_msg) {
                    std_msgs::Header msg = CrashSensorToRosMsg(frame_id_, item, time_offset);
                    crash_pub.publish(msg);
                    LOG_EVERY_N(INFO, 1000) << "receiving crash_sensor message." << std::endl;

                }
            } else {
                LOG(ERROR) << "Grab Crash_sensor data failed, port: " << device_name_ << " speed: " << speed;
            }

            return true;
        }

        bool Spin() {
            ros::Rate loop_rate(fps);
            cout << "Start monitoring serial data" << endl;

            if(ports.Open(device_name_, speed)) {
                imu_pub = node_.advertise<sensor_msgs::Imu>(imu_topic_, 20);
                imu_one_pub = node_.advertise<sensor_msgs::Imu>(imu_one_topic_, 20);
                crash_pub = node_.advertise<std_msgs::Header>(crash_topic_, 10);
                odom_pub = node_.advertise<geometry_msgs::TwistStamped>(odom_topic_, 10);
                camera_pub = node_.advertise<sensor_msgs::Image>(camera_pub_topic_, 10);
                camera_compressed_pub = node_.advertise<sensor_msgs::CompressedImage>(camera_compressed_pub_topic_, 10);
                image_transport::ImageTransport it(node_);


                image_pub = it.advertise(camera_compressed_pub_topic_, 10);
                ports.AsyncStart();

                ports.CameraStart();

                boost::thread(boost::bind(&Ros_Sineva_IMU_Node::GetImageAndPublish, this)).detach();

            } else {
                LOG(FATAL) << "Open port error, port name: " << device_name_ << " speed: " << speed;
                return false;
            }

            while(node_.ok()) {

                if(!Grab_And_Send_Imu()) { ROS_WARN("imu camera sync failed."); }

                ros::spinOnce();
                // 监听键盘按键  q   按q退出循环

                if(kbhit.kbhit() == 113) {
                    is_caputer = false;
                    break;
                }

                loop_rate.sleep();
            }

            ports.CameraStop();

            if(dump)
            { ofs.close(); }

            return true;
        }

        cv::Mat *get_image(CAMERA_INSTANCE camera_instance, int width, int height) {
            IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
            BUFFER *buffer = D_capture(camera_instance, &fmt, 1000);

            if(!buffer)
            { return NULL; }

            // The actual width and height of the IMAGE_ENCODING_RAW_BAYER format and the IMAGE_ENCODING_I420 format are aligned,
            // width 32 bytes aligned, and height 16 byte aligned.
            width = VCOS_ALIGN_UP(width, 32);
            height = VCOS_ALIGN_UP(height, 16);
            cv::Mat *image = new cv::Mat(cv::Size(width, (int)(height * 1.5)), CV_8UC1, buffer->data);
            cv::cvtColor(*image, *image, cv::COLOR_YUV2GRAY_I420);
            D_release_buffer(camera_instance, buffer);
            return image;
        }
        void GetImageAndPublish() {
            while(1) {
                if(is_caputer && (cou_ >= 5)) {
                    BUFFER *buffer = D_capture(camera_instance, &fmt, 1000);

                    if(!buffer) {
                        continue;
                    }

                    cur_camera_time = (buffer->pts) / 1000000.0;

                    //频率正常时计算出camera_time_offset，否则不处理
                    if(first_buffer) {
                        if(fabs(cur_camera_time - last_camera_time) >= 0.098 && fabs(cur_camera_time - last_camera_time) <= 0.102) {
                            cout << "第一帧符合条件的图片---->" << buffer->pts << endl;
                            first_buffer = false;
                            camera_time_offset = ros::Time::now() - ros::Time(cur_camera_time);
                        } else {
                            D_release_buffer(camera_instance, buffer);
                            last_camera_time = cur_camera_time;
                            continue;
                        }
                    }

                    //得出camera_time_offset后，发布图片
                    if(!first_buffer) {
                        cout << fixed << "相机时间---》" << cur_camera_time << "----------" << last_camera_time << endl;
                        width = VCOS_ALIGN_UP(width, 32);
                        height = VCOS_ALIGN_UP(height, 16);
                        cv::Mat *image = new cv::Mat(cv::Size(width, (int)(height * 1.5)), CV_8UC1, buffer->data);
                        cv::cvtColor(*image, *image, cv::COLOR_YUV2GRAY_I420);
                        ros::Time t(cur_camera_time);
                        header.stamp = t + camera_time_offset;
                        header.frame_id = frame_id_;
                        img = cv_bridge::CvImage(header, "mono8", *image).toImageMsg();
                        camera_pub.publish(*img);
                        image_pub.publish(*img);
                        // cv::imwrite("/home/pi/cleanmachine_ws/" + std::to_string(header.stamp.toSec()) + ".png", *image);
                        D_release_buffer(camera_instance, buffer);
                        delete image;
                    }

                    last_camera_time = cur_camera_time;

                }

            }
        }

        void ImageCallback(const sensor_msgs::Image::ConstPtr &msg) {
        }

        void CompressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg) {
        }
      private:
        KbHit kbhit;  //捕获键盘事件
        IMU last_measurement;
        ros::Time imu_first_time;
        ros::NodeHandle node_;
        std::string device_name_, imu_topic_, imu_one_topic_, odom_topic_, crash_topic_, frame_id_, o_path;
        std::string camera_sub_topic_, camera_compressed_sub_topic_, camera_pub_topic_, camera_compressed_pub_topic_;
        int  speed_, fps_;
        uint  speed, fps;
        double base_line, radius;
        ros::Duration time_offset, camera_time_offset;
        ::vslam::drivers::Yesense ports;
        bool first_msgs, dump, first_buffer;
        ros::Publisher imu_pub, imu_one_pub;
        ros::Publisher odom_pub;
        ros::Publisher crash_pub;
        ros::Publisher camera_pub;
        ros::Publisher camera_compressed_pub;
        image_transport::Publisher image_pub;

        std::ofstream ofs;
        boost::mutex cache_lock_;
        std_msgs::Header header;
        sensor_msgs::ImagePtr img ;
        sensor_msgs::CompressedImage compressedImg;
        std::vector<sensor_msgs::Image>  camera_list;
        std::vector<cv::Mat>  image_list;
        std::vector<sensor_msgs::CompressedImage>  compressed_camera_list;
        std::vector<double>  camera_time_list;
        std::map<double, geometry_msgs::Quaternion>  qt_map;
        double camera_count;
        double cou_ = 0; //单次正常读取串口数据次数
        bool is_caputer = true;

        //不同传感器串口数据发送频率
        int imu_fps = 100, odom_fps = 100, crash_fps = 50, camera_fps = 20;

        double cur_camera_time = 0, last_camera_time = 0;

        CAMERA_INSTANCE camera_instance;
        IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
        int width = 640, height = 480;
        struct camera_interface cam_interface;


    };
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_imu_wheel", ros::init_options::NoSigintHandler);
    ros_imu_wheel::Ros_Sineva_IMU_Node node;

    node.Spin();

    return EXIT_SUCCESS;
}
