#ifndef MAP_SEARCH_LIB_H
#define MAP_SEARCH_LIB_H

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <chrono>
#include <spirit_utils/function_timer.h>


class MapSearchLib { 

    public: 
        /**
         * @breif Constructor for the MapSearchLib class 
         * @param[in] input_map Input Map
         * @param[in] search_layer Search Layer of Interest  
         * @return Constructed object of type MapSearchLib 
         */
        MapSearchLib(grid_map::GridpMap& input_map, std::string search_layer,double search_radius = 0.1); 

        /**
         * @breif Iterate through input map for and return positon of cell with minimum cost 
         * @param[in] center grid_map Position object 
         * @return Position of optimal cell   
         */
        grid_map::Position getOptimalCell(grid_map::Position center); 

    private: 
        /**
         * @breif Query input map and return cost at cell 
         * @param[in] iter Spiral iterator object 
         * @return Cost of cell at the specified index  
         */
        double getCost(grid_map::Index index);

        /// Desired search radius
        double radius_; 

        /// GridMap object to store input map
        grid_map::GridMap map_; 

        /// Store data from a specified layer locally for quicker access
        grid_map::Matrix& data_; 

}; 
#endif // MAP_SEARCH_LIB_H