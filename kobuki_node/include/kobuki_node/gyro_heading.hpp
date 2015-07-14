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
  void update(ThreeAxisGyro::Data values, int new_left_encoder, int new_right_encoder);
  void calibrate(ThreeAxisGyro::Data& values);
  bool is_calibrated() { return calibration_done; }
  void resetOdometry();
  double getHeading() { return angle[2]; }
  double getAngularVelocity() { return angular_velocity[2]; }

private:
  double heading_gyro;
  int offset[3];
  float angle[3], angular_velocity[3];
  int left_encoder, right_encoder;
  int nsamples;
  int prev_frame_id;
  bool calibration_done;
  FILE *fp_gyro;
};

}

#endif
