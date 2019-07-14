#ifndef NAV_H
#define NAV_H

#include <vector>
#include <list>
#include <stddef.h>

namespace HL
{


constexpr int INF=1000000;
constexpr float PassNodeErr=0.15;


float angle_diff(float a, float b);

struct CarPose
{
    float x;
    float y;
    float h;
};

struct NavPara
{
    CarPose current;
    CarPose desired;
    bool newGoal;
    bool startNav;
    bool emergeStop;
};

struct Edge
{
    int start;
    int end;
};

struct Graph
{
    int vertexNum;
    int edgeNum;
    std::vector<CarPose> vertex;
    std::vector<std::vector<int> > adjmap;
    std::vector<Edge> se;
    bool changed;
};


class nav
{
public:
    Graph g;
    bool firstIn;
    float EndNodeDisErr;
    float EndNodeAngErr;
    unsigned int k;
    unsigned int cmdCnt;

public:
    nav();
    ~nav();
    bool add_vertex(const CarPose & p);
    bool delete_vertex(const int &index);
    bool add_edge(const int& start, const int& end);
    bool delete_edge(const int& start, const int& end);

    bool dijkstra(const int& start, const int& end,std::vector<int>& path);
private:
    void calAdjMap();
};

constexpr int kCost1 =10;  //直移一格消耗
constexpr int kCost2 =14;  //斜移一格消耗

struct Point
{
    int x,y;
    int F,G,H;
    Point *parent;
    Point(int _x, int _y):x(_x),y(_y),F(0),G(0),H(0),parent(NULL)
    {

    }
};

class Astart
{
public:
    void InitAstart(std::vector<std::vector<char>> &_maze);
    std::list<Point *> GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);

private:
   Point *findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);
   std::vector<Point *> getSurroundPoints(const Point *point,bool isIgnoreCorner) const;
   bool isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
   Point *isInList(const std::list<Point *> &list,const Point *point) const; //判断开启/关闭列表中是否包含某点
   Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
   //计算FGH值
   int calcG(Point *temp_start,Point *point);
   int calcH(Point *point,Point *end);
   int calcF(Point *point);
private:
   std::vector<std::vector<char>> maze;
   std::list<Point *> openList;  //开启列表
   std::list<Point *> closeList; //关闭列表
};




}
#endif // NAV_H
