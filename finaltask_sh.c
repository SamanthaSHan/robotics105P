/**
	Written by: Samantha Seowon Han & Ionut Deaconu
	**Part of the idea to come up with the shortest path is borrowed from Wikipedia's Dijkstra's algorithm
**/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

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
  
  int route[4][4];
};

//struct to keep track while doing dijkstra
struct vertex{
  int parent_i; //corresponds to i (index) of parent
  int parent_j; //corresponds to j (index) of parent
  bool visited; //false if not yet visted, true otherwise
  int distance; //keeps distance from starting point (0,0)
};


//A simple tuple to store indicies i and j
struct position{
  int i;
  int j;
};

//first dimension for north south, second for left right
struct maze cells[4][4]; 

//Array of vertex used for dijkstra for phase 2
struct vertex verticies[4][4];

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
  cells[i][j].visited ++;
  cells[i][j].route[i][j] = 0;
  switch(facingDirection){
    case n: // facing north
      if (front < frontThresh){
        cells[i][j].north = 0;
      }else{
        cells[i][j].route[i+1][j] = 1;
      }
      if (right < sideTurnTresh){
        cells[i][j].east = 0;
      }else{
        cells[i][j].route[i][j+1] = 1;
      }
      if (left <sideTurnTresh){
        cells[i][j].west = 0;
      }else{
        cells[i][j].route[i][j-1] = 1;
      }
      if(i > 0){
        cells[i][j].route[i-1][j] = 1;
      }
      break;
    case e: //facing east
      if (front < frontThresh){
        cells[i][j].east = 0;
      }else{
        cells[i][j].route[i][j+1] = 1;
      }
      if (right < sideTurnTresh){
        cells[i][j].south = 0;
      }else{
        cells[i][j].route[i-1][j] = 1;
      }
      if (left <sideTurnTresh){
        cells[i][j].north = 0;
      }else{
        cells[i][j].route[i+1][j] = 1;
      }
      if(j > 0){
        cells[i][j].route[i][j-1] = 1;
      }
      break;
    case s: //facing south
      if (front < frontThresh){
        cells[i][j].south = 0;
      }else{
        cells[i][j].route[i-1][j] = 1;
      }
      if (right < sideTurnTresh){
        cells[i][j].west = 0;
      }else{
        cells[i][j].route[i][j-1] = 1;
      }
      if (left <sideTurnTresh){
        cells[i][j].east = 0;
      }else{
        cells[i][j].route[i][j+1] = 1;
      }
      if(i < 3){
        cells[i][j].route[i+1][j] = 1;
      }
      break;
    case w: //facing west
      if (front < frontThresh){
        cells[i][j].west = 0;
      }else{
        cells[i][j].route[i][j-1] = 1;
      }
      if (right < sideTurnTresh){
        cells[i][j].north = 0;
      }else{
        cells[i][j].route[i+1][j] = 1;
      }
      if (left <sideTurnTresh){
        cells[i][j].south = 0;
      }else{
        cells[i][j].route[i-1][j] = 1;
      }
      if(j < 3){
        cells[i][j].route[i][j+1] = 1;
      }
      break;
  }
}

direction changeDirection(direction previous, direction tookDirection){
  return (previous + tookDirection) % 4;
}


void init(){
  for(int i = 0; i < 4; i++){
    for(int j = 0; j< 4; j++){
      //initialize our maze
      cells[i][j].north = 1;
      cells[i][j].east = 1;
      cells[i][j].south = 1;
      cells[i][j].west = 1;
      cells[i][j].visited = 0;
      for(int a = 0; a<4; a++){
        for(int b = 0; b<4; b++){
          cells[i][j].route[a][b] = INT_MAX;
        }
      }

      //initialize dijsktra helper array
      verticies[i][j].parent_i = -1; //set parent of each cell to an invalid array index
      verticies[i][j].parent_j = -1;
      verticies[i][j].visited = false;
      verticies[i][j].distance = INT_MAX;
    }
  }
}

//this finishes phase 1
void end_of_phase1(direction facing){
  switch(facing){
    case 3:
      drive_goto(-26, 25);
      drive_goto(squareTick, squareTick);
      drive_goto(52, -51);
      break;
    case 2:
      drive_goto(squareTick, squareTick);
      drive_goto(52, -51);
      break;
  }
}

//print the results of phase 1 if applicable
void print_phase1(){
  for(int i = 3; i >= 0; i--){
    for(int j = 0; j < 4; j++){
      printf("(%d,%d)n:%d e:%d s:%d w:%d \n", i, j, cells[i][j].north, cells[i][j].east, cells[i][j].south, cells[i][j].west);
      for(int a = 3; a >= 0; a--){
        for(int b = 0; b<4; b++){
          printf("(%d,%d)%d ", a,b,cells[i][j].route[a][b]);
        }
        printf("\n");
      }
    }
    printf("\n");
  }
}

//find the next vertex to explore
struct position whereNext (){
  int min = INT_MAX;
  struct position checkNext;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      if((verticies[i][j].visited == false) && verticies[i][j].distance <= min){
        min = verticies[i][j].distance;
        checkNext.i = i;
        checkNext.j = j;
      }
    }
  }
  return checkNext;
}

//find the shortest path from (0,0) using dijkstra's
//I borrowed general ideas from https://en.wikipedia.org/wiki/Dijkstra's_algorithm
void findShortest(){

  //Distance to the (0,0) to (0,0) is 0
  verticies[0][0].distance = 0;

  //This for loop just makes sure that we visit every vertex (we have 16 verticies/cells all the time)
  for(int cnt = 0; cnt < 16; cnt++){
      struct position current = whereNext(); //find index to explore next

      verticies[current.i][current.j].visited = true; //now mark it visited

      //Look at all verticies adjecent to the current one
      for(int a = 0; a < 4 ; a++){
        for(int b = 0; b < 4; b++){
          struct vertex compareVertex = verticies[a][b];
          int route_from_ij_to_ab = cells[current.i][current.j].route[a][b];
          int cumulative_dist = verticies[current.i][current.j].distance;
          if((!compareVertex.visited) && (route_from_ij_to_ab != INT_MAX) && (cumulative_dist != INT_MAX) &&(route_from_ij_to_ab + cumulative_dist < compareVertex.distance)){
            verticies[a][b].parent_i = current.i;
            verticies[a][b].parent_j = current.j;
            verticies[a][b].distance = route_from_ij_to_ab + cumulative_dist;
          }
        }
      }
  }
}

//we have to make sure that the unvisited nodes have correct values
void checkIfVisitedP1(){
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      if(cells[i][j].visited == 0){
        verticies[i][j].parent_i = -2;
        verticies[i][j].parent_j = -2;
        verticies[i][j].distance = INT_MAX;
      }
    }
  }
}

void generatePath(struct position thePath[], int size, int i, int j){
  int index = size;
  struct position toStore;
  toStore.i = i; toStore.j = j;
  
  while((toStore.i > -1)&&(toStore.j > -1)){
    thePath[index] = toStore;
    int a = toStore.i; int b = toStore.j;
    toStore.i = verticies[a][b].parent_i;
    toStore.j = verticies[a][b].parent_j;
    index--;
  }
  toStore.i = 0; toStore.j = 0;
  thePath[0] = toStore;
}


void phase2 (struct position *thePath){
  pause(1000);
  drive_goto(squareTick, squareTick);
  facing = n;
  int currentI = thePath[0].i;
  int currentJ = thePath[0].j;
  for(int current = 1; current < sizeof(thePath); current++){
    if((thePath[current].j == currentJ) && (thePath[current].i - currentI == 1)){
      currentJ = thePath[current].j;
      currentI = thePath[current].i;
      switch(facing){
        case n:
          drive_goto(squareTick, squareTick);
          break;
        case e:
          drive_goto(-25, 26);
          drive_goto(squareTick, squareTick);
          facing = n;
          break;
        case w:
          drive_goto(25, -26);
          drive_goto(squareTick, squareTick);
          facing = n;
          break;
      }
    }else if((thePath[current].i == currentI) && (thePath[current].j - currentJ == 1)){
      currentJ = thePath[current].j;
      currentI = thePath[current].i;
      switch(facing){
        case e:
          drive_goto(squareTick, squareTick);
          break;
        case s:
          drive_goto(-25, 26);
          drive_goto(squareTick, squareTick);
          facing = e;
          break;
        case n:
          drive_goto(26, -25);
          drive_goto(squareTick, squareTick);
          facing = e;
          break;
      }

    }else if((thePath[current].i == currentI) && (currentJ - thePath[current].j == 1)){
      currentJ = thePath[current].j;
      currentI = thePath[current].i;
      switch(facing){
        case w:
          drive_goto(squareTick, squareTick);
          break;
        case n:
          drive_goto(-25, 26);
          drive_goto(squareTick, squareTick);
          facing = w;
          break;
        case s:
          drive_goto(26, -26);
          drive_goto(squareTick, squareTick);
          facing = w;
          break;
      }
    }
  }

}


int main(){
  //initialization
  init();
  low(26), low(27);
  int left, right; //left and right variable for left and right sensors
  int i =0, j =0, previ = i, prevj =j;
  
  //Enter cell (0,0)
  drive_goto(155, 155); 
  //read sensore values and update values for first cell
  frontSensor = ping_cm(8);
  left = leftDist();
  right = rightDist();
  trackValidPath(facing, i, j, frontSensor, left, right);
  //while the first cell has not been visited yet
  while(cells[0][0].visited < 2){
    if(frontSensor < frontThresh){
      if(right > sideTurnTresh){
        madeTurn = e; //will make right
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
        madeTurn = w; //will make left
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
    //update where the robot is facing
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
  }
  //Phase 1 is done at this point
  end_of_phase1(facing);
  
  //uncomment the following to print the result after phase 1
  //print_phase1();
  

  //Find shortest path
  findShortest(); //when this is done, we will have all neccessary info in "verticies" array
  checkIfVisitedP1();
  int thePathSize = verticies[3][3].distance;
  struct position thePath[thePathSize]; 

  //stores the shortest path cell by cell in an array of position (user defined)
  generatePath(thePath, thePathSize, 3, 3);
  
  //perform phase 2
  phase2(thePath);

  return 0;

}
