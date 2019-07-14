#include "nav.h"
#include <math.h>
#include "../map/map_process.h"


namespace HL {


static inline  float normalize(float z)
{
  return atan2(sin(z),cos(z));
}

float angle_diff(float a, float b)
{
  float d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

nav::nav()
{
 g.vertexNum=0;
 g.edgeNum=0;
 EndNodeDisErr=0.05;
 EndNodeAngErr=0.05;
 g.changed = false;
 firstIn=true;
 k=0;
 cmdCnt=0;
}

nav::~nav()
{
    g.vertex.clear();
    g.se.clear();
    g.adjmap.clear();
}

bool nav::add_vertex(const CarPose & p)
{
    g.vertex.push_back(p);
    g.vertexNum ++;
    g.changed = true ;
    return true;
}

bool nav::delete_vertex(const int &index)
{
    if( (index<0) || (index>=g.vertexNum))
        return  false;
    g.vertexNum --;
    g.vertex.erase(g.vertex.begin()+index);
    for(int i=0;i<g.edgeNum;i++)
    {
        if(index==g.se[i].start || index == g.se[i].end)
        {
            g.edgeNum --;
            g.se.erase(g.se.begin()+i);
            i--;
            continue;
        }
        if(index <  g.se[i].start)
             g.se[i].start --;
        if(index <  g.se[i].end)
             g.se[i].end --;
    }
    g.changed = true ;
    return true;
}

bool nav::add_edge(const int& start, const int& end)
{
    if(start == end || start<0 ||end<0 || start>=g.vertexNum || end>=g.vertexNum)
        return  false;
    Edge e{start,end};
    g.se.push_back(e);
    g.edgeNum ++ ;
    g.changed = true ;
    return true;
}

bool nav::delete_edge(const int& start, const int& end)
{
    if(start == end || start<0 ||end<0 || start>=g.vertexNum || end>=g.vertexNum)
        return false;
    for(int i=0;i<g.edgeNum;i++)
    {
        if(start==g.se[i].start && end ==g.se[i].end)
        {
            g.edgeNum -- ;
            g.se.erase(g.se.begin()+i);
            g.changed = true ;
            break;
        }
    }
    return true;
}

void nav::calAdjMap()
{
    if(g.changed)
    {
        g.changed =false;
        g.adjmap.resize(g.vertexNum,std::vector<int>(g.vertexNum,-1));
        for(int i=0 ;i<g.edgeNum ;i++)
        {
             Edge se{g.se[i].start,g.se[i].end};
             CarPose p1,p2;
             p1 =g.vertex[se.start];
             p2 =g.vertex[se.end];
             int dis =  floor(sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))*100);
             g.adjmap[se.start][se.end]=g.adjmap[se.end][se.start]=dis;
        }
    }
}

//负边被认作不联通
//dist 出发点到各点的最短路径长度
//path 路径上到达该点的前一个点
bool nav::dijkstra(const int& start, const int& end, std::vector<int> & path)
{
    calAdjMap();
//    if(start == end || start<0 ||end<0 || start>=g.edgeNum || end>=g.edgeNum)
    if(start == end || start<0 ||end<0)
        return false;

    const int &NODE= g.vertexNum;
    std::vector<int> dist;
    dist.assign(NODE,-1);//初始化距离为未知
    path.assign(NODE,-1);//初始化路径为未知
    std::vector<bool> flag(NODE,false);//标志数组，判断是否处理过
    dist[start]=0;//出发点到自身路径长度为0
    while(1)
    {
        int v=-1;//初始化为未知
        for(int i=0; i!=NODE; ++i)
          if(!flag[i]&&dist[i]>=0)//寻找未被处理过且
              if(v<0||dist[i]<dist[v])//距离最小的点
                  v=i;
        if(v==end)
            return true;

        if(v<0) //所有联通的点都被处理过
            return false;
        flag[v]=1;//标记
        for(int i=0; i!=NODE; ++i)
        {
            if(g.adjmap[v][i]>=0)//有联通路径且
            {
                if(dist[i]<0||dist[v]+g.adjmap[v][i]<dist[i])//不满足三角不等式
                {
                    dist[i]=dist[v]+g.adjmap[v][i];//更新
                    path[i]=v;//记录路径
                }
            }
        }
    }
}




void Astart::InitAstart(std::vector<std::vector<char>> &_maze)
{
   maze = _maze ;
   openList.clear();
   closeList.clear();
}

int Astart::calcG(Point *temp_start,Point *point)
{
    int extraG=(abs(point->x-temp_start->x)+abs(point->y-temp_start->y))==1?kCost1:kCost2;
    int parentG=point->parent==NULL?0:point->parent->G; //如果是初始节点，则其父节点是空
    return parentG+extraG;
}

int Astart::calcH(Point *point,Point *end)
{
    //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^
    return sqrt((double)(end->x-point->x)*(double)(end->x-point->x)+(double)(end->y-point->y)*(double)(end->y-point->y))*kCost1;
}

int Astart::calcF(Point *point)
{
    return point->G+point->H;
}

Point *Astart::getLeastFpoint()
{
    if(!openList.empty())
    {
        auto resPoint=openList.front();
        for(auto &point:openList)
            if(point->F<resPoint->F)
                resPoint=point;
        return resPoint;
    }
    return NULL;
}

Point *Astart::findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)
{
    openList.push_back(new Point(startPoint.x,startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
    while(!openList.empty())
    {
        auto curPoint=getLeastFpoint(); //找到F值最小的点
        openList.remove(curPoint); //从开启列表中删除
        closeList.push_back(curPoint); //放到关闭列表
        //1,找到当前周围八个格中可以通过的格子
        auto surroundPoints=getSurroundPoints(curPoint,isIgnoreCorner);
        for(auto &target:surroundPoints)
        {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            if(!isInList(openList,target))
            {
                target->parent=curPoint;

                target->G=calcG(curPoint,target);
                target->H=calcH(target,&endPoint);
                target->F=calcF(target);

                openList.push_back(target);
            }
            //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
            else
            {
                int tempG=calcG(curPoint,target);
                if(tempG<target->G)
                {
                    target->parent=curPoint;

                    target->G=tempG;
                    target->F=calcF(target);
                }
            }
            Point *resPoint=isInList(openList,&endPoint);
            if(resPoint)
                return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
        }
    }
    return NULL;
}

std::list<Point *> Astart::GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)
{
    Point *result=findPath(startPoint,endPoint,isIgnoreCorner);
    std::list<Point *> path;
    //返回路径，如果没找到路径，返回空链表
    while(result)
    {
        path.push_front(result);
        result=result->parent;
    }
    return path;
}

Point *Astart::isInList(const std::list<Point *> &list,const Point *point) const
{
    //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
    for(auto p:list)
        if(p->x==point->x&&p->y==point->y)
            return p;
    return NULL;
}

bool Astart::isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const
{
    if( (target->x<0) || (target->x > (int)maze.size()-1) ||
        (target->y<0) || (target->y > (int)maze[0].size()-1) ||
        (maze[target->x][target->y]==kOccGrid) ||
        (target->x==point->x && target->y==point->y) ||
        isInList(closeList,target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
        return false;
    else
    {
        if(abs(point->x-target->x)+abs(point->y-target->y)==1) //非斜角可以
            return true;
        else
        {
            //斜对角要判断是否绊住
            if(maze[point->x][target->y]==kUnknownGrid&&maze[target->x][point->y]==kUnknownGrid)
                return true;
            else
                return isIgnoreCorner;
        }
    }
}

std::vector<Point *> Astart::getSurroundPoints(const Point *point,bool isIgnoreCorner) const
{
    std::vector<Point *> surroundPoints;

    for(int x=point->x-1;x<=point->x+1;x++)
        for(int y=point->y-1;y<=point->y+1;y++)
            if(isCanreach(point,new Point(x,y),isIgnoreCorner))
                surroundPoints.push_back(new Point(x,y));

    return surroundPoints;
}

}
