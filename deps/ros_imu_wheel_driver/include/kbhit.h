//
// Created by sineva on 2021/1/12.
//

#ifndef SWEEP_SLAM_KBHIT_H
#define SWEEP_SLAM_KBHIT_H

#include <sys/ioctl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class KbHit {
 public:
  KbHit() {
    tcgetattr(0, &old_attr_);
    new_attr_ = old_attr_;

    new_attr_.c_lflag &= ~ICANON;
    // new_attr_.c_lflag &= ~ECHO;
    new_attr_.c_cc[VMIN]  = 1;
    new_attr_.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_attr_);
  }

  ~KbHit() { tcsetattr(0, TCSANOW, &old_attr_); }

    void releaseHit(){
       std::cout<<"release KbHit"<<std::endl;
       tcsetattr(0, TCSANOW, &old_attr_);
    }
  int kbhit() {
    int bytes;
    ioctl(0, FIONREAD, &bytes);
    if (bytes <= 0) return 0;
    unsigned char ch[3] = {0, 0, 0};
    if (read(0, &ch, 3) > 0) {
      printf("input: %X %X %X\n", ch[0], ch[1], ch[2]);
      return (ch[0] | (ch[1] << 8) | (ch[2] << 16));
    }

    

    return 0;
  }

 private:
  struct termios old_attr_, new_attr_;
};
#endif  //SWEEP_SLAM_KBHIT_H