#include <iostream>
#include <cmath>
#include <unistd.h>

using std::cout;

/**********************************************/
//            declarations
/*********************************************/

enum State : uint8_t {
  Free,
  Open,
  Closed
};

enum Cell : uint8_t {
  Wall = 0,
  Hollow = 1
};

//  functions declaration
//pathfinder
int getHeuristic(const int startX, const int startY, const int destX, const int destY);
int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY, 
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize);

//binary heap functions
void sortBinaryHeapEl(unsigned int* openedNodes, const unsigned int length, const unsigned int* nodesF,
		      unsigned int item);
int binaryHeapPull(unsigned int* openedNodes, unsigned int length, const unsigned int* nodesF);
void binaryHeapPush(unsigned int* openedNodes, const unsigned int length, const unsigned int* nodesF, const int id );

/********************************************************/
//                     definitions
/*******************************************************/

//Sort an item from the binaryTree
void sortBinaryHeapEl(unsigned int* openedNodes, const unsigned int length, const unsigned int* nodesF, unsigned int index){
  unsigned int item = openedNodes[index];
  unsigned int u,v,p;
  //while we didn't hit a break or bubbled to the top or the bottom
  do{
    p = index / 2;
    u = 2*index;
    v = u + 1;
    //check parent
    if(p >= 1 && nodesF[openedNodes[p]] > nodesF[item]){
      std::swap(openedNodes[p],openedNodes[index]);
      index = p;
    }  //case two childs
    else if(v <= length && nodesF[openedNodes[v]] < nodesF[item]){
      std::swap(openedNodes[index],openedNodes[v]);
      index = v;
    } //only one child left, also means we reached the bottom 
    else if(u <= length && nodesF[openedNodes[u]] < nodesF[item]){
      std::swap(openedNodes[index],openedNodes[u]);
      index = u;
    }
    else break;
  } while(index != 1 && index <= length);
};

//Pull the first element from the binaryHeap
int binaryHeapPull(unsigned int* openedNodes, unsigned int length, const unsigned int* nodesF){
  int poped = openedNodes[1], tmp;
  unsigned int pos = 1;
  //it is way faster to just put the last element to the first position and then bubble it to his new position.
  if(length > 1){
    openedNodes[1] = openedNodes[length--];
    sortBinaryHeapEl(openedNodes,length,nodesF,1);
  }
  return poped;
};

//Append a new element to the binaryHeap
void binaryHeapPush(unsigned int* openedNodes, const unsigned int length, const unsigned int* nodesF, const int id ){
  unsigned int pos = length + 1, parent;
  openedNodes[pos] = id;

  while(pos != 1){
    //check parent
    parent = pos/2;
    if(parent >= 1 && nodesF[openedNodes[parent]] > nodesF[id]){
      //swap
      openedNodes[pos] = openedNodes[parent]; 
      openedNodes[parent] = id;
      pos = parent;
    } else break;
  }
};

//return the "manhattan" Heuristic value
int getHeuristic(const int startX, const int startY, const int destX, const int destY){
    //move cost : 1
    return 1 * (std::abs(startX - destX) + std::abs(startY - destY));
};

// Pathfinder using the A* algorithm with manhattan heuristic.
// With no diagonal moves and a fixed move cost value it seemed to be the best choice.
// return -1 if no path found,
// eitherway return length of path, if path is longer than the buffersize, we fill it as much as we can
// and return the full length, the caller will have to decide what to do from it
int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY, 
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize){

  //first things first check that we are not goin to waste our time...
  if(nStartX == nTargetX && nStartY == nTargetY) return 0;
  if(pMap[nTargetX + nTargetY * nMapWidth] == Wall) return -1;

  unsigned int  openedCount = 0, size = nMapWidth * nMapHeight,targetIndex = nTargetX + nTargetY * nMapWidth, current, index;
  int pathLength;
  unsigned int* nodesParent = new unsigned int[size];
  unsigned int* nodesG = new unsigned int[size]; //total move cost
  unsigned int* nodesH = new unsigned int[size]; //heuristic
  unsigned int* nodesF = new unsigned int[size]; //final score
  State* nodesState = new State[size]; //current node state : Free, Open or Closed
  unsigned int* openedNodes = new unsigned int[size + 1];

  //initialize nodesState with Free value to prevent unexpected behaviour
  //other arrays will get their values initialized on the flow
  for(int i = 0; i<size; i++) nodesState[i] = Free;

  //initialize with the starting node on the open list
  nodesG[nStartX + nStartY * nMapWidth] = 1;
  nodesState[nStartX + nStartY * nMapWidth] = Open;
  binaryHeapPush(openedNodes, openedCount, nodesF, nStartX + nStartY * nMapWidth);
  openedCount++;

  //begin proper search
  do{
    //look at the lowest Heuristic from the open list
    current = binaryHeapPull(openedNodes,openedCount,nodesF);
    openedCount--;

    //put it in the closed list
    nodesState[current] = Closed;
    int x = current % nMapWidth;
    int y = current / nMapWidth;

    //add adjascent nodes not opened nor closed to the open list
    for (int b = y - 1; b <= y + 1; b++){
      for (int a = x - 1; a <= x + 1; a++){
	if (a >= 0 && b >= 0 && a < nMapWidth && b < nMapHeight){
    	       	    
	  //skip diagonals and current tile
	  if(a != x && b != y) continue;
	  if(a == x && b == y) continue;

	  //check if node is closed or a wall
	  index = a + b * nMapWidth;

	  //should not happen...
	  if(pMap[index] == Wall || nodesState[index] == Closed) continue;
    	       	    
	  //If it isn’t on the open list, add it.
	  if(nodesState[index] != Open){
	    nodesState[index] = Open;
	    
	    //save parent this will also prevent the node reference to be lost
	    nodesParent[index] = current;
	    nodesG[index] = nodesG[current] + 1;
	    nodesH[index] = getHeuristic(a,b,nTargetX,nTargetY);
	    nodesF[index] = nodesG[index] + nodesH[index];
	    //push our opened node to binaryHeap
	    
	    binaryHeapPush(openedNodes, openedCount, nodesF, index);
	    openedCount++;

	  } //if it is on the open list check the new move cost and update if smaller
	  else {
	    int tmpG = nodesG[current] + 1;
	    if(tmpG < nodesG[index]){
	      nodesG[index] = tmpG;
	      nodesF[index] = tmpG + nodesH[index];
	      nodesParent[index] = current;
	      //resort this item position into the binaryHeap
	      int openIndex;
	      for(int j = 1; j <= openedCount;j++){
		if(openedNodes[j] == index){
		  openIndex = j;
		  break;
		}
	      }
	      sortBinaryHeapEl(openedNodes,openedCount, nodesF, openIndex);
	    }
	  }
	}
      } // for (int a = x - 1; a <= x + 1; a++){
    }; //for (int b = y - 1; b <= y + 1; b++){

    //A path have been found if the target is in the open list
    if(nodesState[targetIndex] == Open){
      break;
    }
       	
  } while(openedCount > 0);

  //if target is not opened this means no valid path have been found
  if(nodesState[nTargetX + nTargetY * nMapWidth] != Open){
    pathLength = -1;
  } else {
    
    //fill path with coords
    pathLength = nodesG[nTargetX + nTargetY * nMapWidth];
    current = nTargetX + nTargetY * nMapWidth;
    //skip starting node
    pathLength--;
    int j = pathLength;
    //if pathLength is bigger than our buffersize
    if(pathLength > nOutBufferSize){
      for(; j > nOutBufferSize; j--){
	current = nodesParent[current];
      }
    }
    for(; j >= 1;j--){
      pOutBuffer[j - 1] = current;
      current = nodesParent[current];
    }
  }

  //clean our stuff
  delete[] nodesParent;
  delete[] nodesG;
  delete[] nodesH;
  delete[] nodesF;
  delete[] nodesState;
  delete[] openedNodes;
  
  return pathLength;
};
