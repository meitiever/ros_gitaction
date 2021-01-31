
#include <Eigen/Core>
#include <ros_imu_wheel/utility.h>

#include "test-entry-point.hpp"

using namespace vslam::utility;
using namespace std;

namespace {
  TEST(ROS_IMU_WHEEL, TestDataGrubFromStream) {
    unsigned char c1, c2, c3, c4;
    double dec = HexToDecFromLowToHighBits(c1,c2,c3,c4);
    ASSERT_EQ(dec, 0.0) << "dec should zero";
    c1 = 0x10;
    c2 = 0x10;
    c3 = 0x10;
    c4 = 0x10;
    dec = HexToDecFromLowToHighBits(c1,c2,c3,c4);
    ASSERT_EQ(dec, 269488144) << "dec should 269,488,144";
    c1 = 0x10;
    c2 = 0x20;
    c3 = 0x30;
    c4 = 0x40;
    dec = HexToDecFromLowToHighBits(c1,c2,c3,c4); // from Hex 40302010 to Dec
    ASSERT_EQ(dec, 1076895760) << "dec should 1,076,895,760";    
  }

  //
  //  1 4 7
  //  2 5 8
  //  3 6 9    
  // 
  TEST(ROS_IMU_WHEEL, TestEigenMatrix) {
    Eigen::Matrix3d m;
    Eigen::Matrix3d m1;
    m.block(0,0,3,1) << 1,2,3;
    m.block(0,1,3,1) << 4,5,6;
    m.block(0,2,3,1) << 7,8,9;
    m1 << 1,4,7,2,5,8,3,6,9;
    ASSERT_EQ(m1, m) << "matrix should 1 4 7 2 5 8 3 6 9"; 

    //ASSERT_EQ(m.block<1, 3>(0, 0), m1.block<1, 3>(0, 0)) << "matrix should 1 4 7 2 5 8 3 6 9"; 
    std::cout << m.block<1, 3>(0, 0) << std::endl; // 1 4 7 
    std::cout << m.block<1, 3>(1, 0) << std::endl; // 2 5 8
    std::cout << m.block<1, 3>(2, 0) << std::endl; // 3 6 9
  }
}

SINEVA_UNITTEST_ENTRYPOINT