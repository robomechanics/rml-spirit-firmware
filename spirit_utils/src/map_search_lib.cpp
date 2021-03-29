#include "spirit_utils/map_search_lib.h"

MapSearchLib::MapSearchLib(grid_map::GridMap& input_map, std::string search_layer ,double search_radius) : map_(input_map), radius_(search_radius) {
    data_ = map_[search_layer]; 
}

double MapSearchLib::getCost(grid_map::Index index) { 
    double query =  data_(index(0),index(1));
    //timer.report(); 
    return query;  
}


grid_map::Postion MapSearchLib::getOptimalCell(grid_map::Position center) { 

    grid_map::Index iter_index; 
    grid_map::Index min_index

    grid_map::Position optimal_pos; 

    double min_cost = 100000; 
    double current_cost; 
 
    for (grid_map::SpiralIterator iter(map_,center,radius_); !iter.isPastEnd();++iter) { 
       iter_index  = *iter;

       current_cost = getCost(iter_index); 

       if (current_cost < min_cost) { 
           min_index = iter_index; 
           min_cost = current_cost; 
       }

    } 

    if (map_.getPosition(min_index,optimal_pos)) { 
        return optimal_pos; 
    }
    else { 
        return center; 
    }
}