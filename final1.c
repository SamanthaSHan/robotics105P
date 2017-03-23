#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "abdrive.h"
#include "simpletext.h"
#include "simpletools.h"
#include "ping.h"

typedef enum {false, true} bool;
typedef enum {n, e, s, w} direction;

//thresholds and ticks (need to tune accordingly)
int frontThresh = 30;
int sideTurnTresh = 19;
int squareTick = 122;

direction facing = n; 
direction madeTurn;

int frontSensor;
int leftwheel, rightwheel;

//struct to keep which direction is valid in respect to the maze
struct maze 
{
  //0 if there is a no wall, 1 if there is a wall
  int north;
  int east;
  int south;
  int west;
  
  //keep track of how many times the cell is visited
  int visited;
    
};

struct vertex
{
  int i; //row
  int j; //column
  int adj[16]; //adjecent vertices with weights, where 1 for straight and 2 for turn
};

//first dimension for north south, second for left right
struct maze cells[4][4]; 
struct vertex routes[16];

int irLeft = 0, irRight = 0;

//Returns the distance to the left wall
int leftDist (){
  irLeft = 0;
  for(int dacVal = 0; dacVal < 160; dacVal += 8)
        {                                               
          dac_ctr(26, 0, dacVal);                       
          freqout(11, 1, 38000);                        
          irLeft += input(10);
        }
  return irLeft;
}

//Returns the distance to the right wall
int rightDist (){
  irRight = 0;
  for(int dacVal = 0; dacVal < 160; dacVal += 8) // increment dacVal by 16 if we want to increase for higher resolution and have range from 0 to 10.
        {                                               
          dac_ctr(27, 1, dacVal);                       
          freqout(1, 1, 38000);                        
          irRight += input(2);
        }
  return irRight;
}

void trackValidPath(direction facingDirection, int i, int j, int front, int left, int right){
	switch(facingDirection){
		case n: // facing north
			if (front < frontThresh){
				cells[i][j].north = 1;
			}
			if (right < sideTurnTresh){
				cells[i][j].east = 1;
			}
			if (left <sideTurnTresh){
				cells[i][j].west = 1;
			}
			cells[i][j].visited ++;
			break;
		case e: //facing east
			if (front < frontThresh){
				cells[i][j].east = 1;
			}
			if (right < sideTurnTresh){
				cells[i][j].south = 1;
			}
			if (left <sideTurnTresh){
				cells[i][j].north = 1;
			}
			cells[i][j].visited ++;
			break;
		case s: //facing south
			if (front < frontThresh){
				cells[i][j].south = 1;
			}
			if (right < sideTurnTresh){
				cells[i][j].west = 1;
			}
			if (left <sideTurnTresh){
				cells[i][j].east = 1;
			}
			cells[i][j].visited ++;
			break;
		case w: //facing west
			if (front < frontThresh){
				cells[i][j].west = 1;
			}
			if (right < sideTurnTresh){
				cells[i][j].north = 1;
			}
			if (left <sideTurnTresh){
				cells[i][j].east = 1;
			}
			cells[i][j].visited ++;
			break;
	}
}

direction changeDirection(direction previous, direction tookDirection){
	return (previous + tookDirection) % 4;
}

void init(){
	for(int i = 0; i < 4; i++){
		for(int j = 0; j< 4; j++){
			cells[i][j].north = 0;
			cells[i][j].east = 0;
			cells[i][j].south = 0;
			cells[i][j].west = 0;
			cells[i][j].visited = 0;
		}
	}
	for(int a = 0; a < 16; a++){
		routes[a].i = 0;
		routes[a].j = 0;
		for(int b = 0; b <16; b++){
			if(a == b){
				routes[a].adj[b] = 0;
			}
			else{
				routes[a].adj[b] = 50; //may need to have infinity value but is not allowed
			}
		}
	}
}

int main(){
  //initialization
	init();
	low(26); //turn sensor                                      
	low(27);
	int left, right; //left and right variable for left and right sensors
	int i =0, j =0; //cell index
    int previ = i, prevj =j;
	drive_goto(155, 155); // go to the first cell
	
	//read sensore values and update values for first cell
	frontSensor = ping_cm(8);
	left = leftDist();
	right = rightDist();
	trackValidPath(facing, i, j, frontSensor, left, right);
	int a = 13;
  while(a < 16){
    if(frontSensor < frontThresh){
      if(right > sideTurnTresh){
		madeTurn = w; //will make right
        drive_goto(25, -26);
		facing = changeDirection(facing, madeTurn);
        drive_goto(squareTick, squareTick);
		
		//update cell status
		left = leftDist();
		right = rightDist();	
        frontSensor = ping_cm(8);
      }
      else if(left > sideTurnTresh){
        drive_goto(-26, 25);
		madeTurn = e; //will make left
		facing = changeDirection(facing, madeTurn);
        drive_goto(squareTick, squareTick);
		
		//update cell status
		left = leftDist();
		right = rightDist();
        frontSensor = ping_cm(8);
      }
      else{
        drive_goto(-51, 52);
		madeTurn = s; //complete turn back
		facing = changeDirection(facing, madeTurn);
        drive_goto(squareTick, squareTick);
		
		//update cell status
		left = leftDist();
		right = rightDist();
        frontSensor = ping_cm(8);
      }
    }
    else if(right > sideTurnTresh){
      drive_goto(26, -25);
	  madeTurn = e; //right turn
	  facing = changeDirection(facing, madeTurn);
      drive_goto(squareTick, squareTick);
	  
	  //update cell status
	  left = leftDist();
	  right = rightDist();
	  frontSensor = ping_cm(8);

    }
    else{
	  madeTurn = n;
	  facing = changeDirection(facing, madeTurn);
      drive_goto(squareTick, squareTick);
	  
	  //update cell status
	  left = leftDist();
	  right = rightDist();
	  frontSensor = ping_cm(8);
    }
      previ = i;
      prevj = j;
    switch(facing){
    	case n:
    		i++;
    		break;
    	case e:
    		j++;
    		break;
    	case s:
    		i--;
    		break;
    	case w:
    		j--;
    		break;
    }
    trackValidPath(facing, i, j, frontSensor, left, right);
	a++;
      printf("(%d, %d)\n", i, j);
    drive_getTicks(&leftwheel, &rightwheel);
  }
  for(int i = 3; i >= 0; i--){
		for(int j = 0; j < 4; j++){
			printf("(%d,%d)v:%d n:%d ", i, j, cells[i][j].visited, cells[i][j].north);

			/* Vertex and adjency list numbering
			   12 13 14 15
			   8  9  10 11
			   4  5  6  7
			   0  1  2  3
			*/
			//update adjecency list
			if(cells[i][j].visited > 0){
				if(cells[i][j].north == 0){
					routes[(i*4)+j].adj[(i+1)*4+j] = 1;
				}
				if(cells[i][j].east == 0){
					routes[(i*4)+j].adj[(i*4)+1+j] = 1;
				}
				if((cells[i][j].south == 0) && (i >0)){
					routes[(i*4)+j].adj[(i-1)*4+j] = 1;
				}
				if((cells[i][j].west == 0) && (j > 0)){
					routes[(i*4)+j].adj[(i*4)-1+j] = 1;
				}
			}
			
		}
		printf("\n");
	}


  return 0;
}