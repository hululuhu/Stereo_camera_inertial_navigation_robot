#ifndef MAP_PROCESS_H
#define MAP_PROCESS_H

#include "../common/common.h"
#include <assert.h>
#include "eigen3/Eigen/Eigen"
#include "map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <queue>

namespace HL {

constexpr char kOccGrid=100;
constexpr char kFreeGrid=0;
constexpr char kUnknownGrid=-1;

constexpr float kOccProbaility=0.51f;
constexpr float kUnknownProbability=0.5f;
constexpr float kFreeprobaility=0.49f;
constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;



constexpr float kDisiInifinte =100.0f;

#define OPTIMIZE 5

// map coords :low-left

// compute linear index for given map coords
inline uint32_t GetGridIndexOfMap(uint32_t w,uint32_t x,uint32_t y)
{
  uint32_t index= w*y+x;
 // assert(index>=0 && index < w*h);
  return  index;
}

inline float fabs(float f)
{
    return f>=0?f:(-f);
}

/**
 * Returns the map pose for the given world pose.
 */
inline Eigen::Vector3f getMapCoordsPose(const map_t* map, const Eigen::Vector3f& worldPose)
{
    return Eigen::Vector3f(MAP_GXWX(map, worldPose[0]),MAP_GYWY(map, worldPose[1]),
            worldPose[2]);
}

/**
 * Returns the world pose for the given map pose.
 */
inline Eigen::Vector3f getWorldCoordsPose(const map_t* map, const Eigen::Vector3f& mapPose)
{
    return Eigen::Vector3f(MAP_WXGX(map, mapPose[0]),MAP_WYGY(map, mapPose[1]),
            mapPose[2]);
}

/**
 * Compute the map coords for given the cell index
 */

void map_liner(const char *g,char *m ,int32_t w ,int32_t h );
void map_filter(char *out,int32_t w, int32_t h);
void map_filter(char *out, int32_t w, int32_t h,char center,char round);

void map_process(char *m,const nav_msgs::OccupancyGrid& map);


struct CellInfo
{
  int x;
  int y;
  float heading;
  int status;
  float grade;
  float dis_avg;
  float dis_err;
  float neighbour[8];
};

struct SingleScan
{
    float dis_avg;
};

bool CellInfo_cmpdis_err(const CellInfo& c1,const CellInfo & c2);
bool CellInfo_cmpdis_grade(const CellInfo& c1,const CellInfo & c2);


class MapProcess
{
private:
     SingleScan singleScan;
     int valid_cell_count[OPTIMIZE+1];
public:
     std::vector<CellInfo> free_grid_Cell;
     float map_resolution;
     nav_msgs::OccupancyGrid filter_map;
     float optimize[OPTIMIZE];
     int laser_skip;
     int filter_cnt;

private:
     int GetFreeSpcaceIndices(const char *grid,int w,int h);
     void CalNeighbour(const char *grid,int w,int h,float resolution);
     void calHeading(const sensor_msgs::LaserScanConstPtr& scan,int skip=1);
     void Optimiz(const sensor_msgs::LaserScanConstPtr& scan,int skip,int pcnt);
public:
     MapProcess();
     ~MapProcess();
     void GetBinaryAndSample(const nav_msgs::OccupancyGridConstPtr& grid ,int th_occ ,int th_free ,int num );
     void CalScan(const sensor_msgs::LaserScanConstPtr& scan);
     geometry_msgs:: Point32 GetPoint(const CellInfo & cell ,const nav_msgs::OccupancyGrid& map);

};





}



#endif // MAP_PROCESS_H
