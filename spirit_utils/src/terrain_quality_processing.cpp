#include "spirit_utils/terrain_quality_processing.h"
TerrainQualityProcessing::TerrainQualityProcessing(ros::NodeHandle &nh) : nh_(nh) { 
    publisher_ = nh.advertise<grid_map_msgs::GridMap>("grid_map_process", 1, true); 
    subscriber_ = nh.subscribe("/grid_map_filter_demo/filtered_map",1 , &TerrainQualityProcessing::filteredMapCallback, this); 
    outputMap.setFrameId("map");
    outputMap.setGeometry(grid_map::Length(1.2, 2.0), 0.03);
    outputMap.add("cost"); 
    outputMap["cost"].setConstant(0.0);

}

void TerrainQualityProcessing::filteredMapCallback(const grid_map_msgs::GridMap& message) { 
      // Convert message to map.
      grid_map::GridMapRosConverter::fromMessage(message, filteredMap);

      // init outputMap 
}

void TerrainQualityProcessing::processTerrainMap(grid_map::Position center, double radius) {  
    // Init minimum cost, current cost, and outputMap 
    double min_cost = 100000; 
    double current_cost; 



    // Store layers locally for efficency 
    grid_map::Matrix& elevation = filteredMap["elevation"];
    grid_map::Matrix& cost =  outputMap["cost"]; 
    grid_map::Matrix& traversability = filteredMap["traversability"]; 

    outputMap.add("elevation",elevation); 

    grid_map::Index min_index;
    grid_map::Index iter_index; 

    for (grid_map::SpiralIterator iter(outputMap,center,radius); !iter.isPastEnd();++iter) { 

        iter_index = *iter;

        // Light up the current index 
        cost(iter_index(0),iter_index(1)) = 1.0;

        // Get current cost  
        current_cost = getCost(traversability,iter_index); 

        if(current_cost < min_cost) { 
            // Turn off prev_min_cost index  
            cost(min_index(0),min_index(1)) = 0.0; 

            //Update min_index and cost
            min_cost = current_cost;
            min_index = *iter;

            // Light up new min_cost index 
            cost(min_index(0),min_index(1)) = 0.5;   
        }

        publish();
        ros::Duration duration(0.02);
        duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
} 

double TerrainQualityProcessing::getCost(grid_map::Matrix& data,grid_map::Index index) { 
    //spirit_utils::FunctionTimer timer(__FUNCTION__);
    double query =  data(index(0),index(1));
    //timer.report(); 
    return query;  
}

double TerrainQualityProcessing::getCostInterp(grid_map::GridMap gm,std::string &layer, const grid_map::Position pos,grid_map::InterpolationMethods interp) { 
    return gm.atPosition(layer,pos,interp); 
}

void TerrainQualityProcessing::publish() {
  outputMap.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(outputMap, message);
  publisher_.publish(message);
  ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}

void TerrainQualityProcessing::spin() { 
    ros::Rate rate(30.0);
    while (nh_.ok()) {

        // Add data to grid map type layer
        grid_map::Position cent(-0.01,-0.01); 
        double radius = 0.1;

    

        processTerrainMap(cent,radius);
        // Wait for next cycle.
        rate.sleep();  
    }
}