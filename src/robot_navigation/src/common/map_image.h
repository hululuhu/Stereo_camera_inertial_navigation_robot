#ifndef MAP_IMAGE_H
#define MAP_IMAGE_H

#include <qpixmap.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>


namespace HL {

constexpr int xyLength =20;
constexpr int carLength1 = 15 ;
constexpr int vertex_radius =10 ;
constexpr int vertex_text_offset =5;




struct MapInfo
{
    float resolution;
    int w;
    int h;
    double word_x ;
    double word_y ;
    int map_x ;
    int map_y ;
};

struct CarInfo
{
    float x;
    float y;
    float h;
    int mx;
    int my;  //right-up
};

class MapImage
{
public:
    int filter_cnt;
    int sample_cnt;


public:
    MapImage();
    ~MapImage();
    void GetBinaryImage(const cv::Mat& imgIn,double th,cv::Mat& imgOut,cv::Mat& sample_imgOut );
    void GetQImage(const cv::Mat& image,QImage &img);
    void SurfFeatureMatch(const cv::Mat& map,const cv::Mat &subMap);
    void OrbFeaturematch(const cv::Mat& map,const cv::Mat &subMap);
    void SiftFeaturematch(const cv::Mat& map,const cv::Mat &subMap);

    void ShowBinaryImage(const std::string& winname,const cv::Mat& imgIn,double th);

 //   void

};

}



#endif // MAP_IMAGE_H
