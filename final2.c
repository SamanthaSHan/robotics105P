/*
Updated: 11:05 AM Thu
READ ME FIRST BEFORE READING THE CODE:
  So far:
  I have implemented Dijkstra's && the code does compile but I haven't done much testing.
  I did follow http://www.geeksforgeeks.org/greedy-algorithms-set-6-dijkstras-shortest-path-algorithm/
  But made my own struct. Otherwise, it will look pretty similar
  
  TODO (in order):
  1. We need to double check if cells[i][j].route[a][b] is updated correctly
  2. If step 1 is good to go, check if Dijkstra works properly
  3. We need to "build out path" accordingly by looking at the result from after running Dijkstra
  4. Properly finish phase 1
*/
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

/*
  REVISED: I just simplified keeping track of what cell was visited, parents and the distance from (0,0) while doing Dijkstra
*/
//struct to keep track while doing dijkstra
struct vertex{
  int parent_i; //corresponds to i (index) of parent
  int parent_j; //corresponds to j (index) of parent
  bool visited; //false if not yet visted, true otherwise
  int distance; //keeps distance from starting point (0,0)
};

/*
  NEW: This is just a struct to store both i and j (used in finding which vertex to look at in each step of Dijkstra)
*/
//A simple tuple to store indicies i and j
struct position{
  int i;
  int j;
};

//first dimension for north south, second for left right
struct maze cells[4][4]; 

/*
  REVISED: So this keeps track of all verticies while doing Dijkstra
*/
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
      cells[i][j].visited ++;
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
      cells[i][j].visited ++;
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
      cells[i][j].visited ++;
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
      cells[i][j].visited ++;
      break;
  }
}

direction changeDirection(direction previous, direction tookDirection){
  return (previous + tookDirection) % 4;
}

/*
  REVISED: I initialized verticies array (let me know if I should chnage the parent values)
*/
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

/*
  NEW: This goes through all the verticies and sees which one to look at next. 
       The next vertex is selected when it is not been visited and if it is the minimum distance from the origin
       If it doesn't satisfy the above, we just flag it with "-2"
*/
//find the next vertex to explore
struct position whereNext (){
  int min = INT_MAX;
  struct position checkNext;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      if(!verticies[i][j].visited && verticies[i][j].distance <= min){
        min = verticies[i][j].distance;
        checkNext.i = i;
        checkNext.j = j;
      }else{
        checkNext.i = -2;
        checkNext.j = -2;
      }
    }
  }
  return checkNext;
}

//find the shortest path from (0,0) using dijkstra's
void findShortest(){

  //Distance to the (0,0) to (0,0) is 0
  verticies[0][0].distance = 0;

  //This for loop just makes sure that we visit every vertex (we have 16 verticies/cells all the time)
  for(int cnt = 0; cnt < 16; cnt++){
      struct position current = whereNext(); //find index to explore next

      //This is used for the case where not all verticies are visited in phase1 and therefore still in the loop to find next vertex
      if((current.i == -2)&& (current.j == -2)){
        return;
      }

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
    printf("%d ", facing);
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
      printf("(%d, %d)\n", i, j);
  }
  //Phase 1 is done at this point

  //Print the results for phase 1
  for(int i = 3; i >= 0; i--){
    for(int j = 0; j < 4; j++){
      printf("(%d,%d)n:%d e:%d s:%d w:%d ", i, j, cells[i][j].north, cells[i][j].east, cells[i][j].south, cells[i][j].west);
    }
    printf("\n");
  }

  //Find shortest path
  findShortest(); //when this is done, we will have all neccessary info in "verticies" array
}