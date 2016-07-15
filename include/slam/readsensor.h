//
// Created by Muyuan on 2016-07-10.
//

#ifndef READSENSOR_H
#define READSENSOR_H

#include <rosbag/bag.h>
#include <string>
#include <fstream>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <cmath>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

struct gps_data
{
    double x, y, z; // in meter
    double time; // in second
    gps_data(double xx, double yy, double zz, double time_): x(xx), y(yy), z(zz), time(time_){}
    gps_data(double xx, double yy): x(xx), y(yy){}
    gps_data(double xx, double yy, double time_): x(xx), y(yy), time(time_){}
};

struct imu_data
{
    double ang_vel_x, ang_vel_y, ang_vel_z;
    double acc_x, acc_y, acc_z;
    double time;

    imu_data(double vx, double vy, double vz, double ax, double ay, double az, double time_): ang_vel_x(vx), ang_vel_y(vy), ang_vel_z(vz), acc_x(ax), acc_y(ay), acc_z(az), time(time_){}

    Eigen::Matrix<double, 3, 3> angVelMatrix(){
        Eigen::Matrix<double, 3, 3> rvm;
        rvm << 0,          -ang_vel_z,  ang_vel_y,
               ang_vel_z,  0,           -ang_vel_x,
               -ang_vel_y, ang_vel_x,   0;

        return rvm;
    }

};

// read sensor data from bag file
int readsensor(std::string filename, 
        std::vector<gps_data> &coord){ 
        
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    //topics.push_back(std::string("/mavros/imu/data"));
    //topics.push_back(std::string("/mavros/imu/teperature"));
    topics.push_back(std::string("/mavros/global_position/raw/fix"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::ofstream file("gpsxyz.txt", std::ios_base::out);

    foreach(rosbag::MessageInstance const m, view){
        sensor_msgs::NavSatFix::ConstPtr s = m.instantiate<sensor_msgs::NavSatFix>();
        if (s != NULL){
            double latitude = s->latitude;
            double longitude = s->longitude;
            double altitude = s->altitude;
            //std::cout << "latitude: " << latitude <<std::endl;
            //std::cout << "longitude: " << longitude <<std::endl;
            //std::cout << "altitude: " << altitude <<std::endl;
            geographic_msgs::GeoPoint p = geodesy::toMsg(latitude, longitude, altitude);
            geodesy::UTMPoint pp;
            geodesy::fromMsg(p, pp);
            file <<pp.easting <<"  "<<pp.northing <<"  " <<pp.altitude<<"\n";
            gps_data temp(pp.easting, pp.northing);
            coord.push_back(temp);
        }

    }

    file.close();
    bag.close();
    return 0;
}

class sensor
{
public:
    ros::Subscriber sub_gps;
    ros::Subscriber sub_imu;
    std::vector<gps_data> gps;
    std::vector<imu_data> imu;

    sensor(){
        ros::NodeHandle node;
        sub_gps = node.subscribe("mavros/global_position/raw/fix", 1000, &sensor::gpsDataCallback, this);
        sub_imu = node.subscribe("mavros/imu/data_raw", 1000, &sensor::imuDataCallback, this);    
    }

    void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg){
        if (msg->status.status != 0) return;

        double latitude = msg->latitude;
        double longitude = msg->longitude;
        double altitude = msg->altitude;
        uint32_t sec = msg->header.stamp.sec;
        uint32_t nsec = msg->header.stamp.nsec;
        double time  = sec - sec/100000*100000 + nsec*pow(10, -9);

        geographic_msgs::GeoPoint p = geodesy::toMsg(latitude, longitude, altitude);
        geodesy::UTMPoint pp;
        geodesy::fromMsg(p, pp);

        gps_data temp(pp.easting, pp.northing, pp.altitude, time);
        this->gps.push_back(temp);

    }

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg){
        double vx= msg->angular_velocity.x;
        double vy= msg->angular_velocity.y;
        double vz= msg->angular_velocity.z;
        double ax= msg->linear_acceleration.x;
        double ay= msg->linear_acceleration.y;
        double az= msg->linear_acceleration.z;
        uint32_t sec = msg->header.stamp.sec; 
        uint32_t nsec = msg->header.stamp.nsec; 

        double time  = sec - sec/100000*100000 + nsec*pow(10, -9);

        imu_data temp(vx, vy, vz, ax, ay, az, time);
        this->imu.push_back(temp);

    }
};
// read sensor data online
int getData(ros::Duration duration, std::vector<gps_data>& gps, std::vector<imu_data>& imu)
{
    ros::NodeHandle n;
    sensor sensor_obj;

    ros::Time begin = ros::Time::now();

    while (ros::Time::now() - begin < duration){
        ros::spinOnce();
    }

    gps = sensor_obj.gps;
    imu = sensor_obj.imu;

    return 0;
}
#endif
