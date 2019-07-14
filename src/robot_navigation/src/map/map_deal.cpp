#include "map_deal.h"

uchar **  map_deal(int *rowl, int *coll, cv::String map_path,int *map_count)
{
	int i = 0, j = 0;
	Mat srcImage = imread(map_path, 0);
	int row = srcImage.rows;
	int col = srcImage.cols;
	//namedWindow("原图");
	//imshow("原图", srcImage);

	Mat two_valueImg = Mat(row, col, CV_8UC1, Scalar(0));
	Mat dilateImage = Mat(row, col, CV_8UC1, Scalar(0));
	Mat erodeImage = Mat(row, col, CV_8UC1, Scalar(0));
	Mat Filter_resultImage = Mat(row, col, CV_8UC1, Scalar(0));

	threshold(srcImage, two_valueImg, 100, 255.0, CV_THRESH_BINARY);
	//namedWindow("二值化");
	//imshow("二值化", two_valueImg);

	//bilateralFilter(two_valueImg, Filter_resultImage, 50, 50, 100/50);
    //namedWindow("滤波");
    //imshow("滤波", Filter_resultImage);

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(35, 35));  
    erode(two_valueImg, erodeImage, element, Point(-1, -1), 1);  
	//namedWindow("膨胀");
	//imshow("膨胀", erodeImage);

	//dilate(erodeImage, dilateImage, element, Point(-1, -1), 1); //���ʹ���
	//namedWindow("腐蚀");
	//imshow("腐蚀图", dilateImage);

	uchar ** ptr = (uchar **)malloc(row * sizeof(uchar *));
	for (i=0; i<row;i++)
		ptr[i]=(uchar *)malloc(col * sizeof(uchar));

    uchar *ptmp=NULL;
	for (i=0;i<row;i++)
        {
		//ptmp = img3.ptr<uchar>(i);
		for (j=0;j<col;j++)
		{
			ptr[i][j]=erodeImage.at<uchar>(i, j);
			//mp[j] = ptr[i][j];
		}
	}
	for (i=0;i<row;i++)
	{
		for (j=0;j<col;j++)
		{
		    if (ptr[i][j]>=1)	
                ptr[i][j]=1;
            else
				ptr[i][j]=0;
			//printf("%d ", ptr[i][j]);
		}
		//printf("\n");
	}

	*rowl=row;
	*coll=col;
	*map_count=row*col;
	return  ptr;
}

void printf_Route(cv::String map_path, pAStarNode * path_stack,int top,int iteration)
{
	Mat srcImage=imread(map_path,0);
	int row=srcImage.rows;
	int col=srcImage.cols;

	Mat two_valueImg=Mat(row, col, CV_8UC1, Scalar(0));
	Mat erodeImage=Mat(row, col, CV_8UC1, Scalar(0));
	
	threshold(srcImage, two_valueImg, 100, 255.0, CV_THRESH_BINARY);

	Mat element=getStructuringElement(MORPH_ELLIPSE, Size(25, 25));  
	erode(two_valueImg, erodeImage, element, Point(-1, -1), 1);   

	while (top>=0)		
	{
		if (top>0)
		{
			erodeImage.at<uchar>(path_stack[top]->s_x, path_stack[top]->s_y)=128;
			//printf("(%d,%d)-->", path_stack[top]->s_x, path_stack[top]->s_y);
		}
		else
		{
			erodeImage.at<uchar>(path_stack[top]->s_x, path_stack[top]->s_y)=128;
			//printf("(%d, %d)\n", path_stack[top]->s_x, path_stack[top]->s_y);
		}
		--top;      
	}
        
    AStarNode *start_node = NULL;
	AStarNode *curr_node = NULL;		
	AStarNode *Middle_point = NULL;			
	AStarNode *last_node = NULL;			
	int flag = 0;
	int count_obs = 0;
	int iteration_count = 0;  

	start_node = path_stack[++top];        
	curr_node = start_node;
	last_node = start_node;          
	path_stack_deal[top]=start_node;

	while (iteration_count<iteration)    
	{
		while (curr_node)
		{
			int xDistance = curr_node->s_x - start_node->s_x;
			int yDistance = curr_node->s_y - start_node->s_y;
			int Distance = (int)round(sqrt(xDistance * xDistance + yDistance * yDistance));   
			//printf("(%d ) ", Distance);
			int tempx, tempy, i;

			for (i=1; i<=Distance; i++)
			{
				tempx = (int)round(start_node->s_x + i * xDistance / Distance);  
				tempy = (int)round(start_node->s_y + i * yDistance / Distance);
				//printf("(%d,%d)-->", tempx, tempy);
				Middle_point = &(map_maze[tempx][tempy]);
				if (judge_curr_node(Middle_point, row, col))  
				{
					count_obs++;
					if (i <= Distance && count_obs > 6)     
					{
						flag = 1;
						count_obs = 0;
						break;
					}
				}
				else if (flag == 1 || curr_node->s_style == STARTNODE)     
				{
					//erodeImage.at<uchar>(tempx, tempy) = 128;
					path_stack_deal[++top] = Middle_point;     
				}
			}
			if (i > Distance && flag == 1)    
			{
				if (!curr_node->s_parent)   
				{
					path_stack_deal[++top] = curr_node;
					break;
				}
				else
				{
					path_stack_deal[++top] = curr_node;    
					start_node = curr_node->s_parent;   
					curr_node = curr_node->s_parent->s_parent;
					path_stack_deal[++top] = start_node;   
				}
				flag = 0;
			}
			else  if (i > Distance)            
			{
				last_node = curr_node;          
				curr_node = curr_node->s_parent; 
			}
			else                                
			{
				curr_node = last_node;
			}
		}

		while ( top>=0 )		
		{
			erodeImage.at<uchar>(path_stack_deal[top]->s_x, path_stack_deal[top]->s_y) = 128;
			--top;
			if (top >= 0)
			{
				if (path_stack_deal[top] == path_stack_deal[top + 1])
				{
					path_stack_deal[top]->s_parent = path_stack_deal[top + 2];
				}
				else
				{
					path_stack_deal[top]->s_parent = path_stack_deal[top + 1];
				}
			}
			else;
		}

		start_node = path_stack_deal[++top];        
		curr_node = start_node;
		last_node = start_node;
		
		flag = 0;
		count_obs = 0;
		Middle_point = NULL;
		path_stack_deal[top] = start_node;

		iteration_count++;
	}

	namedWindow("路径规划图");
	imshow("路径规划图", erodeImage);
}
