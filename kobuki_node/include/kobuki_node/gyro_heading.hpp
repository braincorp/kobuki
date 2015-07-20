#ifndef KOBUKI_NODE_GYRO_HEADING_HPP_
#define KOBUKI_NODE_GYRO_HEADING_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <stdio.h>
#include <ecl/geometry/pose2d.hpp>
#include <ros/ros.h>
#include <angles/angles.h>
#include <kobuki_driver/packets/three_axis_gyro.hpp>
#include <kobuki_driver/packets/core_sensors.hpp>

namespace kobuki {

class GyroHeading {
public:
  GyroHeading();
  ~GyroHeading() { fclose(fp_gyro); }
  void init(ros::NodeHandle& nh, const std::string& name);
  void update(ThreeAxisGyro::Data values, float angular_velocity, int new_left_encoder, int new_right_encoder);
  void calibrate(ThreeAxisGyro::Data& values);
  bool is_calibrated() { return calibration_done; }
  void resetOdometry();
  double getHeading() { return angle[2]; }
  double getAngularVelocity() { return angular_velocity[2]; }

private:
  double heading_gyro;
  float angle[3], angular_velocity[3];
  int left_encoder, right_encoder;
  double gyro_scale_factor, gyro_bias_factor;
  int nsamples;
  int prev_frame_id;
  bool calibration_done;
  FILE *fp_gyro;
};

}

#endif
