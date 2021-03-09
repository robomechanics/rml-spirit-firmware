#include <ros/ros.h> 
#include <ros/package.h>
#include "spirit_utils/terrain_quality_processing.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"terrain_quality_processing_node"); 

    //Get stl string from package 
    ros::NodeHandle nh; 

    TerrainQualityProcessing tqp(nh); 
    
    tqp.spin(); 
    
    return 0; 
}