//
// Created by Muyuan Lin 2016-07-07
//

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <vector>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>

#include <slam/se3_spline.h>
#include <typeinfo>

#include <slam/readsensor.h>

typedef SE3Group<double> SE3Type;

struct Pose
{
    // data[0], [1], [2]: the rotation parameters
    // data[3], [4], [5]: the translation parameters
    double *data;
    
    Pose(){
        data = new double[6];
        for (int i=0; i<6; i++)
            data[i] = 0;
    }

    Pose(const Eigen::Matrix<double, 6, 1> &vec){
        data = new double[6];
        for (int i=0; i<6; i++)
            data[i] = vec(i);
        //std::cout <<"This is the pose data value: " <<data[0] <<" " <<this->data[0] <<endl;
    }

    Eigen::Matrix3d rotationMatrix(){
        Eigen::Matrix3d rm;
        // follow the XYZ convention
        /*rm << Eigen::AngleAxisd(data[2], Eigen::Vector3d::UnitZ())
          *   Eigen::AngleAxisd(data[1], Eigen::Vector3d::UnitY())
          *   Eigen::AngleAxisd(data[0], Eigen::Vector3d::UnitX());
        */
        Eigen::Matrix3d yawR,rollR, pitchR;
        const auto yaw = data[2]*M_PI/180;
        const auto pitch = data[1]*M_PI/180;
        const auto roll= data[0]*M_PI/180;

        yawR << cos(yaw), -sin(yaw), 0.0,
                sin(yaw),  cos(yaw), 0.0,
                     0.0,       0.0, 1.0;

        pitchR << cos(pitch),    0.0, sin(pitch),
                         0.0,    1.0,        0.0,
                 -sin(pitch),    0.0, cos(pitch);

        rollR << 1.0,            0.0,         0.0,
                 0.0,       cos(roll), -sin(roll),
                 0.0,       sin(roll),  cos(roll);

        rm = yawR*pitchR*rollR;

        return rm;
    }

    Eigen::Vector3d translationMatrix(){
        Eigen::Vector3d tm;
        tm << data[3], data[4], data[5];
        return tm;
    }

    double* KnotObj(){
        double* knot = new double[7];
        Eigen::Vector3d p = this->translationMatrix();
        Eigen::Quaterniond q(this->rotationMatrix());
        q.normalize();
        
        knot[0] = q.w();
        knot[1] = q.x();
        knot[2] = q.y();
        knot[3] = q.z();

        for (int i=4; i<7; i++)
            knot[i] = p[i-4]; 

        return knot;
    }
    
};
double B1(double u){
    return (pow(u, 3) - 3*pow(u, 2)+3*u+5)/6;
}
double B2(double u){
            return (-2*pow(u, 3) + 3*pow(u, 2)+3*u+1)/6;
        }
double B3(double u){
            return (pow(u, 3))/6;
        }


struct GPSConstraint {
    // constructor for GPSSplineConstraint
    GPSConstraint(gps_data gpsData, double spline_dt, double spline_offset, double sigma_GPS)
        : gpsData_(gpsData), spline_dt_(spline_dt), spline_offset_(spline_offset), sigma_GPS_(sigma_GPS){}
    
    // residual calculation
    bool operator()(Pose p1, Pose p2, Pose p3, Pose p4, double& residuals) const
    {
        UniformSpline<double> spline(spline_dt_, spline_offset_);

        spline.add_knot(p1.KnotObj());
        spline.add_knot(p2.KnotObj());
        spline.add_knot(p3.KnotObj());
        spline.add_knot(p4.KnotObj());
        
        // Distance to each GPS measurement (x, y only)
        Sophus::SE3Group<double> P;
        Eigen::Matrix<double, 4, 4> dP, d2P;
        residuals = 0;

        spline.evaluate(gpsData_.time, P, dP, d2P);
        Eigen::Matrix<double, 3, 1> translation = P.translation();
        //
        //
        
        double u = (gpsData_.time-spline_offset_-0.5)/0.5;
        double x = p1.data[3] + (p2.data[3]-p1.data[3])*B1(u)
            + (p3.data[3]-p2.data[3])*B2(u)
            + (p4.data[3]-p3.data[3])*B3(u);
        double y = p1.data[4] + (p2.data[4]-p1.data[4])*B1(u)
            + (p3.data[4]-p2.data[4])*B2(u)
            + (p4.data[4]-p3.data[4])*B3(u);



        //
/*
        std::cout <<"Pose 0 parameter: " <<p1.data[3] <<" " <<p1.data[4] <<endl;
        std::cout <<"Pose 1  parameter: " <<p2.data[3] <<" "<<p2.data[4] <<endl;

        std::cout <<"Pose 2 parameter: " <<p3.data[3] <<" " <<p3.data[4] <<endl;
        std::cout <<"Pose 3  parameter: " <<p4.data[3] <<" "<<p4.data[4] <<endl;

        std::cout <<"P.translation: " <<translation[0] <<" " <<translation[1] <<" " <<translation[2] <<endl;
        std::cout <<"gps data: " <<gpsData_.time - spline_offset_ <<gpsData_.x <<" " <<gpsData_.y <<endl;
        std::cout <<"x and y: "<<x <<" " <<y <<endl;
*/
        residuals += pow(translation[0]-gpsData_.x, 2);
       residuals += pow(translation[1]-gpsData_.y, 2);

 //       residuals += pow(x-gpsData_.x, 2);
 //       residuals += pow(y-gpsData_.y, 2);       
        residuals /= sigma_GPS_; 
  
        return true;
    }
  
    double spline_offset_;
    double spline_dt_;
    double sigma_GPS_;
    gps_data gpsData_;
};


struct IMUConstraint {
    // constructor for IMUConstraint
    IMUConstraint(imu_data imuData, double spline_dt, double spline_offset, double sigma_IMU)
        : imuData_(imuData), spline_dt_(spline_dt), spline_offset_(spline_offset), sigma_IMU_(sigma_IMU){}
    // residual calculation
    bool operator()(Pose p1, Pose p2, Pose p3, Pose p4, double& residuals) const
    {
        UniformSpline<double> spline(spline_dt_, spline_offset_);
        spline.add_knot(p1.KnotObj());
        spline.add_knot(p2.KnotObj());
        spline.add_knot(p3.KnotObj());
        spline.add_knot(p4.KnotObj());
        
        Sophus::SE3Group<double> P0, delta;
        Eigen::Matrix<double, 4, 4> dP, d2P;
        residuals = 0;

        spline.evaluate(imuData_.time, P0, dP, d2P);

        // Gyro
        Eigen::Matrix<double, 3, 3> rotationMatrix = P0.rotationMatrix();
        Eigen::Matrix<double, 3, 3> rotationMatrixTranspose = rotationMatrix.transpose();
        // angVelTensor is defined as: [   0  -w_z   w_y;
        //                               w_z     0  -w_x;
        //                              -w_y   w_x     0 ]
        Eigen::Matrix<double, 3, 3> angVelTensor= rotationMatrixTranspose * dP.block(0,0,3,3);

        residuals += pow(angVelTensor(2, 1) - imuData_.ang_vel_x, 2); 
        residuals += pow(angVelTensor(0, 2) - imuData_.ang_vel_y, 2); 
        residuals += pow(angVelTensor(1, 0) - imuData_.ang_vel_z, 2); 
        
        //Accel
        Eigen::Matrix<double, 3, 1> accelMatrix = d2P.block(0, 3, 3, 1); 
        Eigen::Matrix<double, 3, 1> gravity(0, 0, -9.781); // gravity constant in Singapore
        Eigen::Matrix<double, 3, 1> accelMatrixG = accelMatrix + gravity; 
        Eigen::Matrix<double, 3, 1> accel = rotationMatrixTranspose * accelMatrixG;
        residuals += pow(accel(0) - imuData_.acc_x, 2); 
        residuals += pow(accel(1) - imuData_.acc_y, 2); 
        residuals += pow(accel(2) - imuData_.acc_z, 2); 


        residuals /= sigma_IMU_; 
  
        return true;
    }
  
    double spline_offset_;
    double spline_dt_;
    double sigma_IMU_;
    imu_data imuData_;
};



// Create the spline for the first time, otherwise the second parameter 
// for spline() should not be 0.0
UniformSpline<double> create_zero_spline(size_t num_knots) {
    UniformSpline<double> spline(1.0, 0.0);
    spline.zero_knots(num_knots);
    return spline;
}

std::vector<double> create_eval_times(UniformSpline<double>& spline, size_t num_eval) {
    std::vector<double> eval_times;
    double t = spline.min_time();
    double delta = (spline.max_time() - spline.min_time()) / num_eval;
    for (size_t i=0; i < num_eval; ++i) {
        eval_times.push_back(t);
        t += delta;
    }

    return eval_times;
}

// Given a received data at time t, find out which spline segment it 
// belong to
int checkInterval(const int num_knots, const double offset, const double dt, const double time)
{
    size_t i;
    for (i=1; i<num_knots-1; ++i){
        double ta = offset + i * dt;
        double tb = offset + (i+1)*dt;
        if ((time >= ta) && (time < tb)){
            break; 
        }
    }
    int i0 = i - 1;

    // for knot i-1, i, i+1, i+2, only data received within (i, i+1) will be
    // considered
    if ((i0<1) || (i0>num_knots-3)){
        return -1;
    }

    return i0;
}
#endif
