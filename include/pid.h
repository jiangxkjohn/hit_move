#ifndef _PID_H_
#define _PID_H_

#include "robot_pose.h"

namespace hit{

  class PID
  {
  private:
    float KP, KI, KD, e, e_, e_sum;

  public:
    PID();
    PID(float kp, float ki, float kd);
    ~PID();

    float control(const Pose& goal, const Pose& pose);
    float control(const float theta_goal, const float theta_current);

    float getError();
		void cleanError();

    void setPID(float kp, float ki, float kd);

  };

};
#endif