#include <ros/ros.h> 
#include <ros/package.h>
#include "spirit_utils/unify_gnd_map.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"unify_gnd_map_node"); 

    //Get stl string from package 
    const std::string test_stl = ros::package::getPath("spirit_utils") + "/data/terrain.stl";  
    ros::NodeHandle nh; 

    UnifyGndMap unify_gnd_map(nh); 
    
    unify_gnd_map.spin(test_stl); 
    return 0; 
}