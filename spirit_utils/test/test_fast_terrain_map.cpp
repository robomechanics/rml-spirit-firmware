#include <ros/ros.h>
#include <gtest/gtest.h>
#include <grid_map_core/grid_map_core.hpp>
#include "spirit_utils/fast_terrain_map.h"
#include "spirit_utils/terrain_quality_processing.h"
#include <iostream> 
#include <grid_map_core/TypeDefs.hpp> 
TEST(FastTerrainMapTest, testConstructor) {
  ros::NodeHandle nh;
  FastTerrainMap fast_terrain_map;
  EXPECT_EQ(1 + 1, 2);
}

TEST(FastTerrainMapTest, testProjection) {
  ros::NodeHandle nh;
  FastTerrainMap fast_terrain_map;

  int x_size = 2;
  int y_size = 2;
  std::vector<double> x_data = {-1, 1};
  std::vector<double> y_data = {-1, 1};
  std::vector<double> z_data_vec = {-1, 1};
  std::vector<std::vector<double> > z_data = {z_data_vec,z_data_vec};

  std::vector<double> dx_data_vec = {0, 0};
  std::vector<double> dz_data_vec = {1, 1};
  std::vector<std::vector<double> > dx_data = {dx_data_vec,dx_data_vec};
  std::vector<std::vector<double> > dy_data = dx_data;
  std::vector<std::vector<double> > dz_data = {dz_data_vec,dz_data_vec};


  fast_terrain_map.loadData(x_size, y_size, x_data, y_data, z_data, dx_data, dy_data, dz_data,
    z_data, dx_data, dy_data, dz_data);

  // Eigen::Vector3d point = {0,0.5,1};
  // Eigen::Vector3d direction = {0.1,0.1,-1};
  Eigen::Vector3d point = {0,0,1};
  Eigen::Vector3d direction = {0,0,-1};

  auto t_start = std::chrono::steady_clock::now();
  //Eigen::Vector3d intersection = fast_terrain_map.projectToMap(point, direction);
  auto t_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
  // std::cout << "projectToMap took " << time_span.count() << " seconds." << std::endl;

  // std::cout << "Result is {" << intersection[0] << ", " << intersection[1] << ", " << intersection[2] << "}" << std::endl;

  EXPECT_EQ(1 + 1, 2);
}

TEST(FastTerrainMapTest, testGridMapQuery) { 
  ros::NodeHandle nh;
  
  //Declare grid_map object and fill 
  grid_map::GridMap gm({"elevation"});
  gm.setFrameId("map");
  gm.setGeometry(grid_map::Length(1.2, 2.0), 0.03);

  for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    gm.getPosition(*it, position);
    gm.at("elevation", *it) = -0.04 + 0.2 * std::sin(5.0 * position.y()) * position.x();
  } 


  //Isolate indivudal layer for easy access
  grid_map::Matrix& elevation = gm["elevation"];
  const grid_map::Position pos(0,0); 
  double radius = 0.1;

  //Setup Parameters for Interpolation ENUM
  grid_map::InterpolationMethods linear = grid_map::InterpolationMethods::INTER_LINEAR; 
  grid_map::InterpolationMethods nearestNeighbor = grid_map::InterpolationMethods::INTER_NEAREST; 
  std::string layer = "elevation"; 

  //Create TerrainQualityProcessing obj 
  grid_map::SpiralIterator iter(gm,pos,radius); 
  grid_map::Index iter_index = *iter; 
  TerrainQualityProcessing tqp(nh); 
  
  //Benchmark direct query
  auto t_start_base = std::chrono::steady_clock::now();
  double cost_base = tqp.getCost(elevation,iter_index);
  auto t_end_base = std::chrono::steady_clock::now();

  std::chrono::duration<double> time_span_base = std::chrono::duration_cast<std::chrono::duration<double>>(t_end_base - t_start_base);
  
  std::cout << "gmapQuery took " << time_span_base.count() << " seconds." << std::endl;
  std::cout << "cost_base =" << cost_base << "\n\n";

  //Benchmark query with linear interpolation
  auto t_start_linear = std::chrono::steady_clock::now();
  double cost_linear = tqp.getCostInterp(gm,layer,pos, linear);
  auto t_end_linear = std::chrono::steady_clock::now();

  std::chrono::duration<double> time_span_linear = std::chrono::duration_cast<std::chrono::duration<double>>(t_end_linear - t_start_linear);
  
  std::cout << "gmapQueryLinearInterp took " << time_span_linear.count() << " seconds." << std::endl;
  std::cout << "cost_linear =" << cost_linear << "\n\n";

  //Benchmark query with nearest neighbor interpolation
  auto t_start_nn = std::chrono::steady_clock::now();
  double cost_nn = tqp.getCostInterp(gm,layer,pos, nearestNeighbor);
  auto t_end_nn = std::chrono::steady_clock::now();

  std::chrono::duration<double> time_span_nn = std::chrono::duration_cast<std::chrono::duration<double>>(t_end_nn - t_start_nn);
  
  std::cout << "gmapQueryNNInterp took " << time_span_nn.count() << " seconds." << std::endl;
  std::cout << "cost_nn =" << cost_nn << "\n\n";

  EXPECT_EQ(1 + 1, 2);

}

TEST(FastTerrainMapTest, testFastMapQuery) {
  ros::NodeHandle nh; 
  grid_map::GridMap gm({"z"});
  gm.setFrameId("map");
  gm.setGeometry(grid_map::Length(1.2, 2.0), 0.03);

  for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    gm.getPosition(*it, position);
    gm.at("z", *it) = -0.04 + 0.2 * std::sin(5.0 * position.y()) * position.x();
  } 
  FastTerrainMap fast_terrain_map;
  fast_terrain_map.loadDataFromGridMap(gm);  

  //Benchmark FastTerrainMap query
  auto t_start = std::chrono::steady_clock::now();

  auto height = fast_terrain_map.getGroundHeight(0.0,0.0); 

  auto t_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);

  std::cout << "fastmapQuery took " << time_span.count() << " seconds." << std::endl;
  std::cout << "height =" << height << "\n\n";

  
  EXPECT_EQ(1 + 1, 2);

} 
