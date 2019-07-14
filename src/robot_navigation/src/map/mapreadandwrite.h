#ifndef MAPREADANDWRITE_H
#define MAPREADANDWRITE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace HL {

bool PgmAndYamlToOccupancy(nav_msgs::OccupancyGrid& grid,const std::string& stem);
void OccupancyToPgmAndYaml(nav_msgs::OccupancyGrid& grid,const std::string& stem);

}


#endif // MAPREADANDWRITE_H
