
#include "../../include/kobuki_node/gyro_heading.hpp"
#include <stdio.h>

namespace kobuki 
{

void GyroHeading::update(ThreeAxisGyro::Data data, float new_angular_velocity, int new_left_encoder, int new_right_encoder)
{ 
    // initialize
    if (left_encoder == -1) {
        left_encoder = new_left_encoder;
        right_encoder = new_right_encoder;
    }

    int delta_left_encoder =  abs(new_left_encoder - left_encoder);
    int delta_right_encoder = abs(new_right_encoder - right_encoder);

    // calibrate gyro here
    const double digit_to_dps = 0.00875; // digit to deg/s ratio, comes from datasheet of 3d gyro[L3G4200D].
    const double scale = (digit_to_dps * M_PI / 180.0);
    unsigned int length = data.followed_data_length/3;
    const double dt = 1.0/100.0;   // gyro at 100 Hz

    if (data.frame_id == prev_frame_id) // same as last frame, so no change
        return;

    //    angular_velocity = angular_velocity * self.gyro_scale_factor
    //    angular_velocity = angular_velocity * self.gyro_scale_error + self.gyro_bias_error
    //    angle = self.angle + angular_velocity*dt # basic integration (change to trapezoidal rule later)
    //    self.angle = np.arctan2(np.sin(angle), np.cos(angle)) # limit the angle to -pi to pi
    //    self.angular_velocity = angular_velocity
    //    return self.angle
    // accumulate the offset values
    double angle_noise_std = 0.00574455;

    if(1) {
        for (unsigned int i=0; i<length; i++) {

            // Sensing axis of 3d gyro is not match with robot. It is rotated 90 degree counterclockwise about z-axis.
            angular_velocity[0] = angles::from_degrees(-digit_to_dps * (short) data.data[i*3+1]);
            angular_velocity[1] = angles::from_degrees( digit_to_dps * (short) data.data[i*3+0]);
            angular_velocity[2] = angles::from_degrees( digit_to_dps * (short) data.data[i*3+2]);

            angular_velocity[2] = (angular_velocity[2] + gyro_bias_factor);  //apply bias factor
            // apply the scale and bias factor (3*std)
            if ((angular_velocity[2] < angle_noise_std) && (angular_velocity[2] > -angle_noise_std))  { // check for zeroing
              //ROS_ERROR_STREAM("angular velocity: " << abs(angular_velocity[2]) << "," << angular_velocity[2]);
              angular_velocity[2] = 0.0;
            }
            angular_velocity[2] = angular_velocity[2] *  gyro_scale_factor; // find the scale factor

            for (unsigned int j=0; j < 3; j++) {
                angle[j] = angle[j] + angular_velocity[j]*dt;  // integrate basic
                angle[j] = angles::normalize_angle(angle[j]);  // limit to -pi to +pi
            }
        }
    }
    else {
        double dt = (1.0/50.0);
        // compute the angle from processed angular velocity
        angular_velocity[2] = new_angular_velocity;
        angle[2] = angle[2] + angular_velocity[2]*dt;  // integrate basic
    }

    left_encoder = new_left_encoder;
    right_encoder = new_right_encoder;
    prev_frame_id =  data.frame_id;
    //fprintf(fp_gyro, "%5.5f,%5.5f,%5.5f,%5.5f\n", heading_gyro, angular_velocity[2], kobuki_heading, kobuki_angular_velocity);
}

void GyroHeading::init(ros::NodeHandle& nh, const std::string& name)
{
  if (!nh.getParam("gyro_scale_factor", gyro_scale_factor)) {
    ROS_WARN_STREAM("Kobuki : no param server setting for gyro_scale_factor, using default [" << gyro_scale_factor << "][" << name << "].");
  } else {
    if ( gyro_scale_factor ) {
      ROS_INFO_STREAM("Kobuki : using imu data for heading [" << name << "].");
    } else {
      ROS_INFO_STREAM("Kobuki : using encoders for heading (see robot_pose_ekf) [" << name << "].");
    }
  }

  if (!nh.getParam("gyro_bias_factor", gyro_bias_factor)) {
    ROS_WARN_STREAM("Kobuki : no param server setting for gyro_bias_factor, using default [" << gyro_bias_factor << "][" << name << "].");
  } else {
    if ( gyro_bias_factor ) {
      ROS_INFO_STREAM("Kobuki : using imu data for heading [" << name << "].");
    } else {
      ROS_INFO_STREAM("Kobuki : using encoders for heading (see robot_pose_ekf) [" << name << "].");
    }
  }
  
  //gyro_scale_factor = 1.02560376307;
  //gyro_bias_factor  = 0.0121441355987;
  ROS_ERROR_STREAM("Kobuki: gyro_scale_factor: " << gyro_scale_factor << ", gyro_bias_factor: " << gyro_bias_factor );
}

GyroHeading::GyroHeading()
{
    prev_frame_id = -1;
    nsamples = 0;
    heading_gyro = 0.0;
    left_encoder = -1;
    right_encoder = -1;

    calibration_done = true;
    resetOdometry();
}

void GyroHeading::resetOdometry()
{
    for (int i=0; i < 3; i++) {
        angle[i] = 0.0;
        angular_velocity[i] = 0.0;
    }
}

void GyroHeading::calibrate(ThreeAxisGyro::Data& data)
{
 // separate calibration step
}

}
