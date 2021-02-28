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

//! A library for the unification of ground map data structures 
/*!
    This library includes the main functions required to do the
    take in an input map of an stl data type and convert the data to a 
    point cloud map. In its current state, the scaling/filters are hard coded 
    for a specific stl file. The mechanism to convert a point cloud map to a 
    grid_map data structure is handled in the launch file with the external
    pacakage grid_map_to_pcl. This library relies on Eigen and pcl. 
*/
class UnifyGndMap { 
    
    public:
        /**
         * @breif Constructor for the UnifyGndMap class 
         * @param[in] nh Node Handle 
         * @return Constructed object of type UnifyGndMap
         */
        UnifyGndMap(ros::NodeHandle &nh);  

        /**
         * @breif Primary work function in class, called in node file
         * @param[in] stl_file std::string of the name of the stl file
         */
        void spin(const std::string &stl_file); 

    private:
        
        /**
         * @breif Load in cloud from STL file and store in private member variable
         * @param[in] stl_file std::string of the name of the stl file
         */
        void loadCloudFromSTL(const std::string &stl_file); 

        
        /**
         * @breif Publish the scaled and filtered point cloud map 
         */
        void publish(); 


        /**
         * @breif Scale down the cloud uniformily due to large input map
         * @param[in] scale_factor Float value to set scale factor
         */
        void scaleDown(float scale_factor); 


        /** 
         * @breif Remove outlier points on the x-y plane and save to new pcd
         * @param[in] loadFile name of input pcd file 
         * @param[in] saveFile name of output pcd file 
         * @return -1 if failed to loadFile and 0 otherwise
         */
        int filterPointCloud(const char *loadFile , const char *saveFile);


        /// Publisher for point cloud map message 
        ros::Publisher publisher_;
        
        /// Nodehandle to pub to 
        ros::NodeHandle nh_; 
        
        /// Inital point cloud map 
        pcl::PointCloud<pcl::PointXYZ> my_cloud; 

        ///Scaled point cloud map 
        pcl::PointCloud<pcl::PointXYZ> my_cloud_scaled; 
 
};
#endif // UNIFY_GND_MAP_H 
