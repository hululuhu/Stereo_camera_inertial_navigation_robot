#include "map_process.h"
#include <ros/ros.h>

namespace HL {


void map_process(char *m,const nav_msgs::OccupancyGrid& map)
{
    char p_value;
    unsigned long dat_size = map.info.width*map.info.height;
    char *gm=new char[dat_size];

    for(int i=0; i<dat_size;i++)
    {
      gm[i]= map.data[i];
      p_value = gm[i];

      if(p_value>=(char)(kOccProbaility*kOccGrid))
        p_value=kOccGrid;
      else if(p_value <=(char)(kFreeprobaility*kOccGrid))
        p_value=kFreeGrid;
      else
        p_value=kUnknownGrid;
      m[i]=p_value ;
    }

    map_liner(gm, m ,map.info.width, map.info.height);
    map_filter(m, map.info.width, map.info.height);
    map_filter(m, map.info.width, map.info.height, kOccGrid, kUnknownGrid);
    map_filter(m, map.info.width, map.info.height, kUnknownGrid, kFreeGrid);

    delete gm;
}

void map_liner(const char *g,char *m ,int32_t w ,int32_t h )
{
    char *mtx=new char [w*h];
    char *mty=new char [w*h];
    for(int i=0;i<w*h;i++)
    {
        mtx[i]=m[i];
        mty[i]=m[i];
    }
    for(int y=0;y<h;y++)
    {
        int min=y*w;
        int max=(y+1)*w;
        int x=y*w;
        while(x<max)
        {
            if(m[x++]==kOccGrid)
            {
                int start=x-1;
                while((x<max)&&(m[x]==kOccGrid))
                    x++;
                int end =x;
                int gmin= g[start];
                for(int k=start+1;k<end;k++)
                {
                    if(gmin>g[k])
                        gmin=g[k];
                }
                char sc,ec;
                if(start-1 >=min)
                    sc =m[start-1];
                else
                    sc =kUnknownGrid;
                if(end<max)
                    ec =m[end];
                else
                    ec =kUnknownGrid;
                int left=start;
                int right=end-1;
                for(int k=start;k<end;k++)
                {
                    if(gmin==g[k])
                    {
                        left =k;
                        break;
                    }
                    else
                        mtx[k]=sc;
                }
                for(int k=end-1;k>=start;k--)
                {
                    if(gmin==g[k])
                    {
                        right =k;
                        break;
                    }
                    else
                        mtx[k]=ec;
                }
                while(left+1<right)
                     mtx[left++]=kUnknownGrid;
            }
        }
    }

    for(int x=0;x<w;x++)
    {
        int min=x;
        int max=h*w+x;
        int y=x;
        while(y<max)
        {
            if(m[y]==kOccGrid)
            {
                int start=y;
                while((y<max)&&(m[y]==kOccGrid))
                    y +=w;
                int end =y;

                int gmin= g[start];
                for(int k=start+w;k<end;k+=w)
                {
                    if(gmin>g[k])
                        gmin=g[k];
                }
                char sc,ec;
                if(start-w >=min)
                    sc =m[start-w];
                else
                    sc =kUnknownGrid;
                if(end<max)
                    ec =m[end];
                else
                    ec =kUnknownGrid;
                int left=start;
                int right=end-w;
                for(int k=start;k<end;k+=w)
                {
                    if(gmin==g[k])
                    {
                        left =k;
                        break;
                    }
                    else
                        mty[k]=sc;
                }
                for(int k=end-w;k>=start;k-=w)
                {
                    if(gmin==g[k])
                    {
                        right =k;
                        break;
                    }
                    else
                        mty[k]=ec;
                }
                left +=w;
                while(left<right)
                {
                     mty[left]=kUnknownGrid;
                     left +=w;
                }
            }
            else
                y +=w;
        }
    }

    for(int i=0;i<w*h;i++)
    {
        if(m[i] == kOccGrid)
        {
            int temp = mtx[i] +mty[i] ;
            switch (temp)
            {
                case 200 :
                case 100 :
                case 99 :
                    m[i]=kOccGrid;
                    break;
                case 0:
                    m[i]=kFreeGrid;
                    break;
                case -1:
                    m[i]=kOccGrid;
                    break;
                case -2:
                default :
                    m[i]=kUnknownGrid;
                    break;
            }
        }
    }

    delete mtx;
    delete mty;
}

void map_filter(char *out, int32_t w, int32_t h,char center,char round)
{
    int index;
    for(int x=1;x<w-1;x++)
    {
         index= GetGridIndexOfMap(w,x,0);
         if((out[index]==center)&&(out[index-1]==round)&&
            (out[index+1]==round)&&(out[index+w]==round))
             out[index] =round;
         index= GetGridIndexOfMap(w,x,h-1);
         if((out[index]==center)&&(out[index-1]==round)&&
            (out[index+1]==round)&&(out[index-w]==round))
             out[index] =round;
    }
    for(int y=1;y<h-1;y++)
    {
         index= GetGridIndexOfMap(w,0,y);
         if((out[index]==center)&&(out[index-w]==round)&&
            (out[index+w]==round)&&(out[index+1]==round))
             out[index] =round;
         index= GetGridIndexOfMap(w,w-1,y);
         if((out[index]==center)&&(out[index-w]==round)&&
            (out[index+w]==round)&&(out[index-1]==round))
             out[index] =round;
    }
    for(int y=1;y<h-1;y++)
    {
        for(int x=1;x<w-1;x++)
        {
            index= GetGridIndexOfMap(w,x,y);
            if((out[index]==center)&&(out[index-1]==round)&&
               (out[index+1]==round)&&(out[index+w]==round)&&
               (out[index-w]==round))
                out[index] =round;
        }
    }
}

void map_filter(char *out,int32_t w, int32_t h)
{
  int index;
  for(int x=1;x<w-1;x++)
  {
       index= GetGridIndexOfMap(w,x,0);
       if((out[index]==kFreeGrid)&&(out[index-1]!=kFreeGrid)&&
          (out[index+1]!=kFreeGrid)&&(out[index+w]!=kFreeGrid))
           out[index] =kUnknownGrid;
       index= GetGridIndexOfMap(w,x,h-1);
       if((out[index]==kFreeGrid)&&(out[index-1]!=kFreeGrid)&&
          (out[index+1]!=kFreeGrid)&&(out[index-w]!=kFreeGrid))
           out[index] =kUnknownGrid;
  }
  for(int y=1;y<h-1;y++)
  {
       index= GetGridIndexOfMap(w,0,y);
       if((out[index]==kFreeGrid)&&(out[index-w]!=kFreeGrid)&&
          (out[index+w]!=kFreeGrid)&&(out[index+1]!=kFreeGrid))
           out[index] =kUnknownGrid;
       index= GetGridIndexOfMap(w,w-1,y);
       if((out[index]==kFreeGrid)&&(out[index-w]!=kFreeGrid)&&
          (out[index+w]!=kFreeGrid)&&(out[index-1]!=kFreeGrid))
           out[index] =kUnknownGrid;
  }
  for(int y=1;y<h-1;y++)
  {
      for(int x=1;x<w-1;x++)
      {
          index= GetGridIndexOfMap(w,x,y);
          if((out[index]==kFreeGrid)&&(out[index-1]!=kFreeGrid)&&
             (out[index+1]!=kFreeGrid)&&(out[index+w]!=kFreeGrid)&&
             (out[index-w]!=kFreeGrid))
              out[index] =kUnknownGrid;
      }
  }
}

bool CellInfo_cmpdis_err(const CellInfo& c1,const CellInfo & c2)
 {
    if(c1.status ==c2.status)
       return c1.dis_err <c2.dis_err;
    return c1.status >c2.status ;
 }

bool CellInfo_cmpdis_grade(const CellInfo& c1,const CellInfo & c2)
{
    if(c1.status ==c2.status)
      return c1.grade < c2.grade;
    return c1.status >c2.status ;
}

MapProcess::MapProcess()
{
    for(int i=0;i<=OPTIMIZE;i++)
        valid_cell_count[i]=0;
    free_grid_Cell.resize(0);
}

MapProcess::~MapProcess()
{

}


void MapProcess::GetBinaryAndSample(const nav_msgs::OccupancyGridConstPtr& grid ,int th_occ ,int th_free ,int num )
{
   free_grid_Cell.resize(0);
   map_resolution =0.05;

   for(int i=0;i<=OPTIMIZE;i++)
       valid_cell_count[i]=0;
   valid_cell_count[0]=grid->info.width*grid->info.height ;

   int w=grid->info.width/num;
   int h=grid->info.height/num;
   map_resolution = grid->info.resolution *num;
   char *mapdata = new char[w*h];
   int index=0;
   for(int y=0;y<h;y++)
   {
       for(int x=0; x<w;x++)
       {
          int sum=0;
          for(int j=0;j<num;j++)
          {
              for(int i=0;i<num;i++)
              {
                  index= GetGridIndexOfMap(grid->info.width,(x*num+i),(y*num+j));
                  sum+= grid->data[index];
              }
          }
          if(sum>=th_occ)
              mapdata[y*w+x] = kOccGrid;
          else if(sum <th_free)
              mapdata[y*w+x] = kUnknownGrid;
          else
              mapdata[y*w+x] =kFreeGrid;
       }
   }

   map_filter(mapdata,w,h);
   valid_cell_count[1]= GetFreeSpcaceIndices(mapdata,w,h);
   CalNeighbour(mapdata,w,h ,map_resolution);

   filter_map.info.resolution = map_resolution;
   filter_map.info.width = w;
   filter_map.info.height = h;
   filter_map.info.map_load_time=ros::Time::now();
   filter_map.header.frame_id="map";
   filter_map.header.stamp=ros::Time::now();
   filter_map.data.resize(w*h,-1);

   filter_map.info.origin.position.x =grid->info.origin.position.x;
   filter_map.info.origin.position.y =grid->info.origin.position.y;
   filter_map.info.origin.position.z =0;

   for (size_t k = 0; k < w*h; ++k)
       filter_map.data[k]=mapdata[k];

   delete mapdata;

   ROS_INFO("map info %d X %d map @ %3.2lf m/cell",filter_map.info.width,
            filter_map.info.height,filter_map.info.resolution);
}

int MapProcess::GetFreeSpcaceIndices(const char *grid,int w,int h)
{
    CellInfo cell={0,0,0,0,0,0,0,
                   {kDisiInifinte,kDisiInifinte,kDisiInifinte,kDisiInifinte,
                    kDisiInifinte,kDisiInifinte,kDisiInifinte,kDisiInifinte}};
    for(int j = 0; j <h; j++)
        for(int i = 0; i < w; i++)
        {
            if(grid[GetGridIndexOfMap(w,i,j)]==kFreeGrid)
            {
                cell.x=i;
                cell.y=j;
                free_grid_Cell.push_back(cell);
            }
        }
    return (int)free_grid_Cell.size();
}

void MapProcess::CalNeighbour(const char *grid,int w,int h ,float resolution)
{
  //  int min_count = floor(kMinLaserRange/resolution);
    float res;
    int max_count;
    for(int i=0;i<valid_cell_count[1];i++)
    {
        int k=1;
        int x= free_grid_Cell[i].x;
        int y= free_grid_Cell[i].y;
        float sum=0;
        int count=0;
        res=resolution;
        max_count = floor(kMaxLaserRange/res);
        while(k<=max_count)
        {
            if((y+k)<h)
            {
                 if(grid[GetGridIndexOfMap(w,x,y+k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[0] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break;
        }
        k=1;
        while(k<=max_count)
        {
            if((x+k)<w)
            {
                 if(grid[GetGridIndexOfMap(w,x+k,y)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[2] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
               break;

        }
        k=1;
        while(k<=max_count)
        {
            if((y-k)>=0)
            {
                 if(grid[GetGridIndexOfMap(w,x,y-k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[4] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break;

        }
        k=1;
        while(k<=max_count)
        {
            if((x-k)>=0)
            {
                 if(grid[GetGridIndexOfMap(w,x-k,y)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[6] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
               break;
        }

        res=resolution*1.414213;
        max_count = floor(kMaxLaserRange/res);
        k=1;
        while(k<=max_count)
        {
            if((x+k)<w &&(y+k)<h)
            {
                 if(grid[GetGridIndexOfMap(w,x+k,y+k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[1] = k*res ;
                     sum += k*res;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break ;

        }
        k=1;
        while(k<=max_count)
        {
            if((x+k)<w &&(y-k)>=0)
            {
                 if(grid[GetGridIndexOfMap(w,x+k,y-k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[3] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break;
        }
        k=1;
        while(k<=max_count)
        {
            if((x-k)>=0 &&(y-k)>=0)
            {
                 if(grid[GetGridIndexOfMap(w,x-k,y-k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[5] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break;
        }
        k=1;
        while(k<=max_count)
        {
            if((x-k)>=0 &&(y+k)<h)
            {
                 if(grid[GetGridIndexOfMap(w,x-k,y+k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[7] = k*res ;
                     sum += k*res;
                     count++;
                     break;
                 }
                 k++;
            }
            else
             break;
        }

        if(count!=0)
            free_grid_Cell[i].dis_avg =sum/count;
        else
            free_grid_Cell[i].dis_avg =10000.0;

    //    ROS_INFO("[%4d %4d %4.2f]",x,y,free_grid_Cell[i].dis_avg);
    }

//    for(int i=0;i<5;i++)
//    ROS_INFO("[x,y]=[%d,%d]  [%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f]",
//        free_grid_Cell[i].x,free_grid_Cell[i].y,
//        free_grid_Cell[i].neighbour[0],free_grid_Cell[i].neighbour[1],
//        free_grid_Cell[i].neighbour[2],free_grid_Cell[i].neighbour[3],
//        free_grid_Cell[i].neighbour[4],free_grid_Cell[i].neighbour[5],
//        free_grid_Cell[i].neighbour[6],free_grid_Cell[i].neighbour[7]);
}

void MapProcess::CalScan(const sensor_msgs::LaserScanConstPtr& scan )
{
    size_t size = scan->ranges.size();
    float angle = scan->angle_min;

    float sum=0;
    int count=0;       

    for (size_t i = 0; i < size; ++i)
    {
        float dist = scan->ranges[i];
        if ( (dist >kMinLaserRange) && (dist < kMaxLaserRange))
        {
            sum +=dist ;
            count ++;
        }
        angle += scan->angle_increment;
    }
    singleScan.dis_avg =sum/count ;

    int pcnt =valid_cell_count[1];
    for(int i=0;i<pcnt;i++)
    {
      free_grid_Cell[i].status = 0;
      free_grid_Cell[i].dis_err =fabs(free_grid_Cell[i].dis_avg- singleScan.dis_avg);
    }

    std::sort(free_grid_Cell.begin(),free_grid_Cell.end(),CellInfo_cmpdis_err);
//    for(int i=0;i<free_space_count;i++)
//      ROS_INFO("%f",free_grid_Cell[i].dis_err);

    pcnt *=optimize[0];
    for(int i=0;i< pcnt;i++)
      free_grid_Cell[i].status =1;

    valid_cell_count[2]=pcnt;

//    ROS_INFO("l_dis=%6.2f min_er=%6.2f ma_er=%6.2f",singleScan.dis_avg,
//             free_grid_Cell[0].dis_err,free_grid_Cell[pcnt-1].dis_err);

    calHeading(scan,laser_skip);

    ROS_INFO("optimize-4 = %d/%d/%d/%d", valid_cell_count[4],valid_cell_count[3],
              valid_cell_count[2],valid_cell_count[1]);
}

void  MapProcess::calHeading(const sensor_msgs::LaserScanConstPtr& scan,int skip)
{
    int pcnt =valid_cell_count[2];
    Optimiz(scan,skip,pcnt);

    std::sort(free_grid_Cell.begin(),free_grid_Cell.end(), CellInfo_cmpdis_grade);

    pcnt *= optimize[1];
    for(int i=0;i< pcnt;i++)
    {
      free_grid_Cell[i].status =2;
    //  ROS_INFO("%f",free_grid_Cell[i].grade);
    }
    valid_cell_count[3] =pcnt;

    std::sort(free_grid_Cell.begin(),free_grid_Cell.end(), CellInfo_cmpdis_err);

    pcnt *=optimize[2] ;
    for(int i=0;i< pcnt;i++)
    {
      free_grid_Cell[i].status =3;
    //  ROS_INFO("%f",free_grid_Cell[i].dis_err);
    }

    valid_cell_count[4] =pcnt;
}

void MapProcess::Optimiz(const sensor_msgs::LaserScanConstPtr& scan,int skip ,int pcnt)
{
    float neighbour[8]={0};
    int size = (int)scan->ranges.size();
    float ang= scan->angle_min;
    int cnt= size/skip;

    for(int i=0;i<free_grid_Cell.size();i++)
        free_grid_Cell[i].dis_err = 10000;

    for(int i=0;i<cnt;i++)
    {
        for(int j=0;j<8;j++)
        {
          int index = i*skip+j*size/8 ;
          if(index > size)
            index =index/((index/size)*size);
          float dist = scan->ranges[index];
          if ( (dist >kMinLaserRange) && (dist < kMaxLaserRange))
              neighbour[j]=dist;
          else
              neighbour[j] =kDisiInifinte;
        }
        for(int k=0;k<pcnt;k++)
        {
            float sum=0;
            float avg=0;
            int avg_cnt=0;
            for(int j=0;j<8;j++)
            {
                if(neighbour[j]+1<kDisiInifinte)
                {
                    avg += neighbour[j];
                    avg_cnt ++;
                }
                float err=(free_grid_Cell[k].neighbour[j]-neighbour[j]);
                sum += err*err;
            }
            avg /=avg_cnt;
            avg = fabs(free_grid_Cell[k].dis_avg - avg);
            if(sum < free_grid_Cell[k].grade)
                free_grid_Cell[k].grade =sum ;
            if(avg < free_grid_Cell[k].dis_err)
                free_grid_Cell[k].dis_err =avg;
        }
    }
}


geometry_msgs::Point32 MapProcess::GetPoint(const CellInfo & cell ,const nav_msgs::OccupancyGrid& map)
{
    geometry_msgs:: Point32 p;

    p.x= map.info.origin.position.x +(cell.x+0.5)*map.info.resolution ;
    p.y= map.info.origin.position.y +(cell.y+0.5)*map.info.resolution ;
    p.z=0;
    return p;
}


}

