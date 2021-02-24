#ifndef UNIFY_GND_MAP_H
#define UNIFY_GND_MAP_H

#include <ros/ros.h> 
#include <string> 
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h> 
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Geometry> 
/*
#include <grid_map_core> 
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h> 
*/ 

class UnifyGndMap { 
    
    public:

        //Constructor that takes in nodehandle and stl file name
        UnifyGndMap(ros::NodeHandle &nh);  

        //Primary work function in class
        void spin(const std::string &stl_file); 

    private:
        
        //This function loads the STL file into the my_cloud memember variable
        void loadCloudFromSTL(const std::string &stl_file); 
        
        //This function will publish the my_cloud PointCloud map 
        void publish(); 

        void scaleDown(float scale_factor); 

        int filterPointCloud(const char *Loadfile , const char *SaveFile);

        ros::Publisher publisher_;

        ros::NodeHandle nh_; 
         
        pcl::PointCloud<pcl::PointXYZ> my_cloud; 

        pcl::PointCloud<pcl::PointXYZ> my_cloud_scaled; 
 
};
#endif // UNIFY_GND_MAP_H 