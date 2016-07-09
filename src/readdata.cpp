#include <rosbag/bag.h>
#include <string>
#include <fstream>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

int main(){
    rosbag::Bag bag;
    bag.open("loop_20fps.bag", rosbag::bagmode::Read);

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
            std::cout << "latitude: " << latitude <<std::endl;
            std::cout << "longitude: " << longitude <<std::endl;
            std::cout << "altitude: " << altitude <<std::endl;
            geographic_msgs::GeoPoint p = geodesy::toMsg(latitude, longitude, altitude);
            geodesy::UTMPoint pp;
            geodesy::fromMsg(p, pp);
            file <<pp.easting <<"  "<<pp.northing <<"  " <<pp.altitude<<"\n";
        }

    }

    file.close();
    bag.close();
}
