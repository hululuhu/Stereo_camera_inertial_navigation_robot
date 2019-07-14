#include "A_star.h"

AStarNode  ** map_maze;		
pAStarNode *  open_table;		
pAStarNode *  close_table;		
int    open_node_count=0;   	
int	   close_node_count=0; 
pAStarNode * path_stack;		
pAStarNode * path_stack_deal;		
int        top = -1;			

void swap(int idx1, int idx2)
{
	pAStarNode tmp = open_table[idx1];
	open_table[idx1] = open_table[idx2];
	open_table[idx2] = tmp;
}

void adjust_heap(int /*i*/nIndex)
{
	int curr = nIndex;
	int child = curr * 2 + 1;	
	int parent = (curr - 1) / 2;	
	if (nIndex < 0 || nIndex >= open_node_count)
	{
		return;
	}
	while (child < open_node_count)
	{
	if (child + 1 < open_node_count && open_table[child]->s_g + open_table[child]->s_h  > open_table[child + 1]->s_g + open_table[child + 1]->s_h)
	{
		++child;

	}
	if (open_table[curr]->s_g + open_table[curr]->s_h <= open_table[child]->s_g + open_table[child]->s_h)
	{
		break;
	}
	else
	{
			swap(child, curr);		
			curr = child;				
			child = curr * 2 + 1;			
		}
	}
	if (curr != nIndex)
	{
		return;
	}
        while (curr != 0)
	{
		if (open_table[curr]->s_g + open_table[curr]->s_h >= open_table[parent]->s_g + open_table[parent]->s_h)
		{
			break;
		}
		else
		{
			swap(curr, parent);
			curr = parent;
			parent = (curr - 1) / 2;
		}
	}
}

void insert_to_opentable(int x, int y, pAStarNode curr_node, pAStarNode end_node, int w)
{
	int i;
	if (map_maze[x][y].s_style != BARRIER)		
	{
		if (!map_maze[x][y].s_is_in_closetable)	
		{
			if (map_maze[x][y].s_is_in_opentable)	
			{
				if (map_maze[x][y].s_g > curr_node->s_g + w)	 
				{
					map_maze[x][y].s_g = curr_node->s_g + w;
					map_maze[x][y].s_parent = curr_node;
					for (i = 0; i < open_node_count; ++i)       
					{
						if (open_table[i]->s_x == map_maze[x][y].s_x && open_table[i]->s_y == map_maze[x][y].s_y)
						{
    						break;
						}
					}
					adjust_heap(i);					
				}
			}
			else									
			{
				map_maze[x][y].s_g = curr_node->s_g + w;                 
				map_maze[x][y].s_h = abs(end_node->s_x - x) + abs(end_node->s_y - y);     
				//map_maze[x][y].s_h = (int)round(sqrt((end_node->s_x - x) *(end_node->s_x - x) + (end_node->s_y - y) * (end_node->s_y - y)));  //ŷʽ����
				map_maze[x][y].s_parent = curr_node;
				map_maze[x][y].s_is_in_opentable = 1;
				open_table[open_node_count++] = &(map_maze[x][y]);
			}
		}
	}
}

  

void get_neighbors(pAStarNode curr_node, pAStarNode end_node,int row,int col)
{
	int x = curr_node->s_x;
	int y = curr_node->s_y;

	if ((x + 1) >= 0 && (x + 1) < row && y >= 0 && y < col)
	{
		insert_to_opentable(x + 1, y, curr_node, end_node, transverse_cost);
	}

	if ((x - 1) >= 0 && (x - 1) < row && y >= 0 && y < col)
	{
		insert_to_opentable(x - 1, y, curr_node, end_node, transverse_cost);
	}

	if (x >= 0 && x < row && (y + 1) >= 0 && (y + 1) < col)
	{
		insert_to_opentable(x, y + 1, curr_node, end_node, transverse_cost);
	}

	if (x >= 0 && x < row && (y - 1) >= 0 && (y - 1) < col)
	{
		insert_to_opentable(x, y - 1, curr_node, end_node, transverse_cost);
	}

	if ((x + 1) >= 0 && (x + 1) < row && (y + 1) >= 0 && (y + 1) < col)
	{
		insert_to_opentable(x + 1, y + 1, curr_node, end_node, diagonal_cost);
	}

	if ((x + 1) >= 0 && (x + 1) < row && (y - 1) >= 0 && (y - 1) < col)
	{
		insert_to_opentable(x + 1, y - 1, curr_node, end_node, diagonal_cost);
	}

	if ((x - 1) >= 0 && (x - 1) < row && (y + 1) >= 0 && (y + 1) < col)
	{
		insert_to_opentable(x - 1, y + 1, curr_node, end_node, diagonal_cost);
	}

	if ((x - 1) >= 0 && (x - 1) < row && (y - 1) >= 0 && (y - 1) < col)
	{
		insert_to_opentable(x - 1, y - 1, curr_node, end_node, diagonal_cost);
	}
}


int judge_neighbors(pAStarNode curr_node, int row, int col)
{
	int x = curr_node->s_x;
	int y = curr_node->s_y;

	if ((x + 1) >= 0 && (x + 1) < row && y >= 0 && y < col)
	{
		if (map_maze[x + 1][y].s_style == BARRIER)
			return 1;
		else;
	}

	if ((x - 1) >= 0 && (x - 1) < row && y >= 0 && y < col)
	{
		if (map_maze[x - 1][y].s_style == BARRIER)
			return 2;
		else;
	}

	if (x >= 0 && x < row && (y + 1) >= 0 && (y + 1) < col)
	{
		if (map_maze[x][y+1].s_style == BARRIER)
			return 3;
		else;
	}

	if (x >= 0 && x < row && (y - 1) >= 0 && (y - 1) < col)
	{
		if (map_maze[x][y-1].s_style == BARRIER)
			return 4;
		else;
	}

	if ((x + 1) >= 0 && (x + 1) < row && (y + 1) >= 0 && (y + 1) < col)
	{
		if (map_maze[x + 1][y + 1].s_style == BARRIER)
			return 5;
		else;
	}

	if ((x + 1) >= 0 && (x + 1) < row && (y - 1) >= 0 && (y - 1) < col)
	{
		if (map_maze[x + 1][y - 1].s_style == BARRIER)
			return 6;
		else;
	}

	if ((x - 1) >= 0 && (x - 1) < row && (y + 1) >= 0 && (y + 1) < col)
	{
    	if (map_maze[x - 1][y + 1].s_style == BARRIER)
			return 7;
		else;
	}

	if ((x - 1) >= 0 && (x - 1) < row && (y - 1) >= 0 && (y - 1) < col)
	{
		if (map_maze[x - 1][y - 1].s_style == BARRIER)
			return 8;
		else;
	}
	return 0;
}

int judge_curr_node(pAStarNode curr_node, int row, int col)
{
	int x = curr_node->s_x;
	int y = curr_node->s_y;

	if ( x >= 0 && x < row && y >= 0 && y < col)
	{
		if (map_maze[x][y].s_style == BARRIER)
			return 1;
		else;
	}
	return 0;
}


