//
// Created by Muyuan Lin 2016-07-07
//

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <vector>

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
        rm << 0, -data[2], data[1], data[2], 0, -data[0], -data[1], data[0], 0;
        //std::cout <<"Define Pose rotationMatrix(): " << data[0] <<" "<<data[1] <<" " <<data[2] <<endl;
        //std::cout <<"Define Pose rotationMatrix() Compare: " << rm(2,1) <<" "<<rm(0,2) <<" " <<rm(1,0) <<endl;

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

struct GPSConstraint {
    // constructor for GPSSplineConstraint
    GPSConstraint(gps_data gpsData, double spline_dt, double spline_offset, double sigma_GPS)
        : gpsData_(gpsData), spline_dt_(spline_dt), spline_offset_(spline_offset), sigma_GPS_(sigma_GPS){}
    // residual calculation
    bool operator()(Pose p1, Pose p2, Pose p3, Pose p4, double& residuals) const
    {
        std::cout<< "Pose Parameter:" <<endl;
        std::cout <<p1.data[0] <<" "<<p1.data[1] <<" " <<p1.data[2] <<" "<<p1.data[3] <<p1.data[4]endl;
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
        residuals += pow(translation[0]-gpsData_.x, 2);
        residuals += pow(translation[1]-gpsData_.y, 2);
        
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
        Eigen::Matrix<double, 3, 3> rotationMatrixTranspose = rotationMatrix.inverse();
        Eigen::Matrix<double, 3, 3> angVelMatrix = rotationMatrixTranspose * dP.block(0,0,3,3);

        residuals += pow(angVelMatrix(2, 1) - imuData_.ang_vel_x, 2); 
        residuals += pow(angVelMatrix(0, 2) - imuData_.ang_vel_y, 2); 
        residuals += pow(angVelMatrix(1, 0) - imuData_.ang_vel_z, 2); 
        
        //Accel
        Eigen::Matrix<double, 3, 1> accelMatrix = d2P.block(0, 3, 3, 1); 
        Eigen::Matrix<double, 3, 1> gravity(0, 0, 9.781); // gravity constant in Singapore
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
