#ifndef A_STAR_H
#define A_STAR_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define STARTNODE	3  
#define ENDNODE		4   
#define BARRIER		0   
#define free_grid	1  
#define diagonal_cost  14  
#define transverse_cost  10  

typedef struct AStarNode
{
	int s_x;	
	int s_y;
	int s_g;	
	int	s_h;	
	int s_style;
	struct AStarNode * s_parent;
	int s_is_in_closetable;		
	int s_is_in_opentable;		
}AStarNode, *pAStarNode;

extern  AStarNode  ** map_maze;		
extern  pAStarNode *  open_table;		
extern  pAStarNode *  close_table;		
extern  int    open_node_count;   
extern  int	   close_node_count;  
extern  pAStarNode * path_stack;		
extern  pAStarNode * path_stack_deal;		
extern  int        top;		

void swap(int idx1, int idx2);
void adjust_heap(int nIndex); 
void insert_to_opentable(int x, int y, pAStarNode curr_node, pAStarNode end_node, int w);
void get_neighbors(pAStarNode curr_node, pAStarNode end_node, int row, int col);
int judge_neighbors(pAStarNode curr_node, int row, int col);
int judge_curr_node(pAStarNode curr_node, int row, int col);

#endif 
