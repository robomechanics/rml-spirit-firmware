#include "spirit_utils/unify_gnd_map.h"

UnifyGndMap::UnifyGndMap(ros::NodeHandle &nh) {
    //Set both the node handle and publisher 
    nh_ = nh; 
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl2", 1, true); 


}


void UnifyGndMap::loadCloudFromSTL(const std::string &stl_file) { 
    
    pcl::PolygonMesh testMesh; 
    pcl::io::loadPolygonFileSTL(stl_file, testMesh); 
    //pcl_conversions::fromPCL(testMesh.cloud,my_cloud); 
    pcl::fromPCLPointCloud2(testMesh.cloud,my_cloud);
    
    size_t num_points = my_cloud.size(); 
    
    int width = my_cloud.width; 
    int height = my_cloud.height; 

    std::cout << height << " " << width << std::endl; 

}

void UnifyGndMap::publish() {
    pcl_conversions::toPCL(ros::Time::now(),my_cloud.header.stamp); 
    my_cloud_scaled.header.frame_id = "map"; 
    publisher_.publish(my_cloud_scaled.makeShared());
    ROS_INFO("Published PC map");
}

void UnifyGndMap::spin(const std::string &stl_file) { 
    ros::Rate loop_rate(4); 
    
    //Convert stl to pcl data structure 
    loadCloudFromSTL(stl_file); 

    //Scale down point cloud map
    scaleDown(0.01);


    //Save point cloud map to pcd file 
    pcl::io::savePCDFileASCII("test_pcd.pcd",my_cloud_scaled); 

    //Filter the old map
    filterPointCloud("test_pcd.pcd","new_cloud.pcd");
    

    while(nh_.ok()) { 
        publish(); 
        ros::spinOnce(); 
        loop_rate.sleep();  
    } 

}



int UnifyGndMap::filterPointCloud(const char *loadFile , const char *saveFile) {
    /// Create the cloud object and load it from PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ( loadFile, *Cloud_in) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    /// Filter the cloud, Remove any points which have the coordinate Z=0 and set equal to map
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (Cloud_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0,0.0);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    pcl::io::savePCDFileASCII(saveFile, *cloud_filtered);
    my_cloud_scaled = *cloud_filtered; 
    std::cerr << "\nFiltered " << Cloud_in->points.size () - cloud_filtered->points.size () << " data points\n" << std::endl;
    return 0;
} 



void UnifyGndMap::scaleDown(float scale_factor) { 
    //Homogeneous transformation matrix uniformally scaled 
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(); 
    transform(0,0) = transform(0,0) * scale_factor; 
    transform(1,1) = transform(1,1) * scale_factor; 
    transform(2,2) = transform(2,2) * scale_factor; 

    pcl::transformPointCloud(my_cloud,my_cloud_scaled,transform); 
}

