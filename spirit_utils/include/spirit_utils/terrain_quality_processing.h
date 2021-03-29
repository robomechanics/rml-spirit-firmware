#ifndef TERRAIN_QUALITY_PROCESSING_H
#define TERRAIN_QUALITY_PROCESSING_H

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <chrono>
#include <spirit_utils/function_timer.h>


class TerrainQualityProcessing { 
    
    public: 
        
        /**
         * @breif Constructor for the TerrainQualityProcessing class 
         * @param[in] nh Node Handle 
         * @return Constructed object of type TerrainQualityProcessing
         */
        TerrainQualityProcessing(ros::NodeHandle &nh); 
        
        /**
         * @breif Primary work function in class, called in node file  
         */
        void spin(); 

        double getCost(grid_map::Matrix& data, grid_map::Index index);

        double getCostInterp(grid_map::GridMap gm,std::string &layer, const grid_map::Position pos, grid_map::InterpolationMethods interp); 

        
    private:

        /**
         * @breif Publish the output grid_map that returns optimal foothold 
         */
        void publish();

        /**
         * @breif Query input map for traversability and return cost at cell 
         * @param[in] iter Spiral iterator object 
         * @return Cost of cell pointed to by spiral iterator  
         */
          

        void processTerrainMap(grid_map::Position center, double radius); 
        
        void filteredMapCallback(const grid_map_msgs::GridMap& message); 
        
        /// Publisher for grid_map message 
        ros::Publisher publisher_;

        /// Subscriber for filtered_map message
        ros::Subscriber subscriber_; 
        
        /// Nodehandle to pub to and sub from
        ros::NodeHandle nh_; 

        /// GridMap of filtered map to get traversability and elevation  
        grid_map::GridMap filteredMap; 

        /// GridMap for working with in class  
        grid_map::GridMap outputMap; 


}; 
#endif // TERRAIN_QUALITY_PROCESSING_H