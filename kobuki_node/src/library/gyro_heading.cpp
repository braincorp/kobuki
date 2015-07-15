
#include "../../include/kobuki_node/gyro_heading.hpp"
#include <stdio.h>

namespace kobuki 
{

void GyroHeading::update(ThreeAxisGyro::Data data, int new_left_encoder, int new_right_encoder)
{ 
    // initialize
    if (left_encoder == -1) {
        left_encoder = new_left_encoder;
        right_encoder = new_right_encoder;
    }

    int delta_left_encoder =  abs(new_left_encoder - left_encoder);
    int delta_right_encoder = abs(new_right_encoder - right_encoder);

    // check if significant encoder change has happned
    if ((delta_left_encoder > 5) || (delta_right_encoder > 5)) {

        // calibrate gyro here
        const double digit_to_dps = 0.00875; // digit to deg/s ratio, comes from datasheet of 3d gyro[L3G4200D].
        double scale_dps = gyro_scale_factor * digit_to_dps;
        unsigned int length = data.followed_data_length/3;
        const double dt = 1.0/100.0;   // gyro at 100 Hz

        if (data.frame_id == prev_frame_id) // same as last frame, so no change
            return;

        // accumulate the offset values
        for (unsigned int i=0; i<length; i++) {

            // Sensing axis of 3d gyro is not match with robot. It is rotated 90 degree counterclockwise about z-axis.
            angular_velocity[0] = angles::from_degrees( -scale_dps * ((short)data.data[i*3+1] - offset[1] ));
            angular_velocity[1] = angles::from_degrees(  scale_dps * ((short)data.data[i*3+0] - offset[0] ));
            angular_velocity[2] = angles::from_degrees(  scale_dps * ((short)data.data[i*3+2] - offset[2] ));
            
            for (unsigned int j=0; j < 3; j++) {
                angle[j] = angle[j] + angular_velocity[j]*dt;  // integrate basic
                //integration(i) = integration(i-1) + 1⁄6 ( vali-3 + 2 vali-2 + 2 vali-1 + vali) 
                angle[j] = angles::normalize_angle(angle[j]);  // limit to -pi to +pi
            }
        }

        left_encoder = new_left_encoder;
        right_encoder = new_right_encoder;
        //fprintf(fp_gyro, "%5.5f,%5.5f,%5.5f,%5.5f\n", heading_gyro, angular_velocity[2], kobuki_heading, kobuki_angular_velocity);
    }
    // very slight change in the wheel encoder values, ignore gyro measurement and set the angular velocity to zero
    else {
        for (int i=0; i < 3; i++) {
            angular_velocity[i] = 0.0;
        }
    }
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
  gyro_scale_factor = 1.0 / gyro_scale_factor;
}

GyroHeading::GyroHeading()
{
    prev_frame_id = -1;
    nsamples = 0;
    heading_gyro = 0.0;
    left_encoder = -1;
    right_encoder = -1;

    FILE *fp = fopen("/tmp/gyro_calibration.txt", "r");
    if (fp) {
        fscanf(fp, "%d %d %d", &offset[0], &offset[1], &offset[2]);
        ROS_INFO_STREAM("Kobuki gyro calibration. Offset:[" << offset[0]  << "," << offset[1] << "," << offset[2] << "].");
        calibration_done = true;
        fclose(fp);
    }
    else {
       ROS_INFO_STREAM("Kobuki gyro calibration not present..");
       calibration_done = false;
       for (int i=0; i < 3; i++)
           offset[i] = 0;
    }
    //fp_gyro = fopen("gyro_calculation.csv", "w");
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
    // calibrate gyro here
    unsigned int length = data.followed_data_length/3;

    if (data.frame_id == prev_frame_id) // same as last frame
        return;

    // accumuldate the offset values
    for (unsigned int i=0; i<length; i++) {
        for (unsigned int j=0; j < 3; j++) 
            offset[j] += (short) data.data[i*3+j];
        nsamples += 1;
    } 

    // stop after 10 seconds of calibration
    if (nsamples > (50.0 * 5)) {
        // calculate average offset value
        for (unsigned int j=0; j < 3; j++)
            offset[j] = (int) (1.0 * offset[j] / nsamples);
        calibration_done = true;
        ROS_INFO_STREAM("Kobuki calibration done. Offset:[" << offset[0]  << "," << offset[1] << "," << offset[2] << "].");
        FILE* fp = fopen("/tmp/gyro_calibration.txt", "w");
        fprintf(fp, "%d %d %d", offset[0], offset[1], offset[2]);
        fclose(fp);
    }

    prev_frame_id = data.frame_id;
}

}
