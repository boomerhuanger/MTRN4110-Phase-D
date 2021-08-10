// File:          z5206762_MTRN4110_PhaseB.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <Windows.h>
#include <dos.h>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <vector>

using namespace webots;
using namespace std;

#define TIME_STEP 64

void CalcTurns(int x, int row, int turns[45][2]);

int main(int argc, char **argv) { 
  Robot *robot = new Robot();
  //opening output.txt file
  ofstream output;
  output.open("../../Output.txt"); 
  
  // Reading in map
  cout << "[z5206762_MTRN4110_PhaseB] Reading in map from ../../Map.txt..." << endl;
  output << "[z5206762_MTRN4110_PhaseB] Reading in map from ../../Map.txt..." << endl;
  
  fstream Map;
  Map.open("../../Map.txt", ios::in);
  
  string map;
  string saveMap[11] = {};
  int row = 0;
  
  if (Map.is_open()) {
      while(getline(Map, map)) {
        cout << "[z5206762_MTRN4110_PhaseB] " <<  map << "\n";
        output << "[z5206762_MTRN4110_PhaseB] " <<  map << "\n";
        saveMap[row] = map;
        row++;
      }
      Map.close();
  }
  cout << "[z5206762_MTRN4110_PhaseB] Map read in!" << endl << "[z5206762_MTRN4110_PhaseB] Finding shortest paths..." << endl;
  output << "[z5206762_MTRN4110_PhaseB] Map read in!" << endl << "[z5206762_MTRN4110_PhaseB] Finding shortest paths..." << endl;
  
  // Arrays to hold wall data values
  int horizontalWalls[6][9] = {0}; //references horizontal wall at top of cell i,j
  int verticalWalls[5][10] = {0}; //references vertical wall at left of cell i,j

  int hCount = 0;
  int vCount = 1;
  int i = 0;
  int j = 0;
  int num = 2;
  
  while (hCount < 11) {
    num = 2;
    j = 0;
    while (num < 35) {
      if (saveMap[hCount].at(num) == '-') {
        horizontalWalls[i][j] = 1;
      } else {
        horizontalWalls[i][j] = 0;
      }
      num = num + 4;
      j++;
    }
    hCount = hCount + 2;
    i++;
  } 
  
  i = 0;
  while (vCount < 10) {
    num = 0;
    j = 0;
    while (num < 37) {
      if (saveMap[vCount].at(num) == '|') {
        verticalWalls[i][j] = 1;
      } else {
        verticalWalls[i][j] = 0;
      }
      num = num + 4;
      j++;
    }
    vCount = vCount + 2;
    i++;
  }
  
  // Locate epuck and destination
  char startPos[3][1] = {};
  int start[1][2] = {};
  int endPos[1][2] = {0};
  i = 1;
  j = 2;
  int rowNum = 0;
  int colNum = 0;
  while (i < 10) {
    j = 2;
    colNum = 0;
    while (j < 35) {
      if (saveMap[i].at(j) == 'x') {
        endPos[0][0] = rowNum;
        endPos[0][1] = colNum;
      } else if (saveMap[i].at(j) == 'v') {
        startPos[0][0] = '0' + rowNum;
        startPos[1][0] = '0' + colNum;
        startPos[2][0] = 'S';
        start[0][0] = rowNum;
        start[0][1] = colNum;
      } else if (saveMap[i].at(j) == '^') {
        startPos[0][0] = '0' + rowNum;
        startPos[1][0] = '0' + colNum;
        startPos[0][2] = 'N';
        start[0][0] = rowNum;
        start[0][1] = colNum;
      } else if (saveMap[i].at(j) == '<') {
        startPos[0][0] = '0' + rowNum;
        startPos[1][0] = '0' + colNum;
        startPos[0][2] = 'W';
        start[0][0] = rowNum;
        start[0][1] = colNum;
      } else if (saveMap[i].at(j) == '>') {
        startPos[0][0] = '0' + rowNum;
        startPos[1][0] = '0' + colNum;
        startPos[0][2] = 'E';
        start[0][0] = rowNum;
        start[0][1] = colNum;
      }
      j = j + 4;
      colNum++;
    }
    i = i + 2;
    rowNum++;
  }
  
  // Create PathPlan file
  ofstream pathPlan;
  pathPlan.open("../../PathPlan.txt"); 
  //pathPlan << "Hello" << endl;
  
  // Array for floodfill algo
  i = 0;
  int grid[5][9] = {}; 
  while (i < 5) {
    j = 0;
    while (j < 9) {
      grid[i][j] = 45;
      j++;
    }
    i++;
  }
  
  grid[endPos[0][0]][endPos[0][1]] = 0;
  int cVal = 0;
  bool mVal = 1;
  
  while (mVal != 0) {
    mVal = 0;
    for (i = 0; i < 5; i++) {
      for (j = 0; j < 9; j++) {
        if (grid[i][j] == cVal) {       
          if (horizontalWalls[i][j] == 0) { //north wall
            if (grid[i-1][j] == 45) {
              grid[i-1][j] = cVal + 1;
              mVal = 1;
            }
          }
          if (horizontalWalls[i+1][j] == 0) { //south wall
            if (grid[i+1][j] == 45) {
              grid[i+1][j] = cVal + 1;
              mVal = 1;
            }
          }
          if (verticalWalls[i][j] == 0) { //west wall
            if (grid[i][j-1] == 45) {
              grid[i][j-1] = cVal + 1;
              mVal = 1;
            }
          }
          if (verticalWalls[i][j+1] == 0) { //east wall
            if (grid[i][j+1] == 45) {
              grid[i][j+1] = cVal + 1;
              mVal = 1;
            }
          }
        }
      }
    }
    cVal++;
  }
  
  int initVal = grid[start[0][0]][start[0][1]]; // max length of path
  
  i = 0;
  //vector<vector<int>> path;
  // setting all values of the array initially to a number > rows x cols to distinguish if the path is complete
  int paths[100][initVal + 1];
  while (i < 100) {
    j = 0;
    while (j < (initVal + 1)) {
      paths[i][j] = 50;
      j++;
    }
    i++;
  }
  
  // defining initial heading by a number to count turns for each path
  int heading = 0;
  if (startPos[2][0] =='S') {
    heading = 1;
  } else if (startPos[2][0] =='N') {
    heading = 2;
  } else if (startPos[2][0] =='E') {
    heading = 3;
  } else if (startPos[2][0] =='W') {
    heading = 4;
  }
  int cH = heading; // current heading
  int h = heading; // previous heading
  int turns[100][2] = {0}; // 1st col is no. turns, 2nd col current heading
  i = 0;
  while (i < 100) {
    turns[i][1] = heading; // set current heading to initial heading
    i++;
  }
  
  i = 9*start[0][0] + start[0][1]; 
  // classifying map by
  // [ 0  1  2 ... 7   8]
  // [ 9 10 11 ... 16 17]
  // [ .  .  . ...  .  .]
  // [ .  .  . ...  .  .]
  // [36 37 38 ... 43 44]
  // so every cell has a unique value characterised by cell = 9*row + column
  paths[0][0] = i; // array to store paths
  cVal = initVal - 1; // current step value
  int move = 1; // current move/element in path
  num = 0; // number of paths avaialable
  int e = 0; // path
  mVal = 0; // bool to check for branch in path
  while (cVal > -1) { //MVAL = 0???
    mVal = 0;
    h = heading;
    cH = heading;
    if ((horizontalWalls[((i - (i % 9))/9) + 1][i % 9] == 0) && (grid[((i - (i % 9))/9) + 1][i % 9] == cVal)) {
      if (mVal == 1) {
        num++;
        for (int k = 0; k < move; k++) {
          paths[num][k] = paths[e][k];
          if (k > 1) {
            if (paths[num][k] - paths[num][k-1] == 9) {
              cH = 1;
            } else if (paths[num][k] - paths[num][k-1] == -9) {
              cH = 2;
            } else if (paths[num][k] - paths[num][k-1] == 1) {
              cH = 3;
            } else if (paths[num][k] - paths[num][k-1] == -1) {
              cH = 4;
            }
            if (cH == h) {
            } else if (((cH == 1) && (h == 2)) || ((cH == 2) && (h == 1)) || ((cH == 3) && (h == 4)) || ((cH == 4) && (h == 3))) {
              turns[num][0] = turns[num][0] + 2;
              turns[num][1] = cH;
              h = cH;
            } else {
              turns[num][0] = turns[num][0] + 1;
              turns[num][1] = cH;
              h = cH;
            }
          }
        }
        paths[num][move] = i + 9;
        //CalcTurns(1, num, turns);
      } else {
        paths[e][move] = i + 9;
        mVal = 1;
        CalcTurns(1, e, turns);
      }
    }
    if ((horizontalWalls[((i - (i % 9))/9)][i % 9] == 0) && (grid[((i - (i % 9))/9) - 1][i % 9] == cVal)) {
      if (mVal == 1) {
        num++;
        for (int k = 0; k < move; k++) {
         paths[num][k] = paths[e][k];
         if (k > 1) {
            if (paths[num][k] - paths[num][k-1] == 9) {
              cH = 1;
            } else if (paths[num][k] - paths[num][k-1] == -9) {
              cH = 2;
            } else if (paths[num][k] - paths[num][k-1] == 1) {
              cH = 3;
            } else if (paths[num][k] - paths[num][k-1] == -1) {
              cH = 4;
            }
            if (cH == h) {
            } else if (((cH == 1) && (h == 2)) || ((cH == 2) && (h == 1)) || ((cH == 3) && (h == 4)) || ((cH == 4) && (h == 3))) {
              turns[num][0] = turns[num][0] + 2;
              turns[num][1] = cH;
              h = cH;
            } else {
              turns[num][0] = turns[num][0] + 1;
              turns[num][1] = cH;
              h = cH;
            }
          }
        }
        turns[num][0] = turns[e][0];
        turns[num][1] = turns[e][1];
        paths[num][move] = i - 9;
        //CalcTurns(2, num, turns);
      } else {
        paths[e][move] = i - 9;
        mVal = 1;
        CalcTurns(2, e, turns);
      }
    }
    if ((verticalWalls[((i - (i % 9))/9)][(i % 9) + 1] == 0) && (grid[(i - (i % 9))/9][(i % 9) + 1] == cVal)) {
      if (mVal == 1) {
        num++;
        for (int k = 0; k < move; k++) {
         paths[num][k] = paths[e][k];
         if (k > 1) {
            if (paths[num][k] - paths[num][k-1] == 9) {
              cH = 1;
            } else if (paths[num][k] - paths[num][k-1] == -9) {
              cH = 2;
            } else if (paths[num][k] - paths[num][k-1] == 1) {
              cH = 3;
            } else if (paths[num][k] - paths[num][k-1] == -1) {
              cH = 4;
            }
            if (cH == h) {
            } else if (((cH == 1) && (h == 2)) || ((cH == 2) && (h == 1)) || ((cH == 3) && (h == 4)) || ((cH == 4) && (h == 3))) {
              turns[num][0] = turns[num][0] + 2;
              turns[num][1] = cH;
              h = cH;
            } else {
              turns[num][0] = turns[num][0] + 1;
              turns[num][1] = cH;
              h = cH;
            }
          }
        }
        turns[num][0] = turns[e][0];
        turns[num][1] = turns[e][1];
        paths[num][move] = i + 1;
        //CalcTurns(3, num, turns);
      } else {
        paths[e][move] = i + 1;
        mVal = 1;
        CalcTurns(3, e, turns);
      }
    }
    if ((verticalWalls[(i - (i % 9))/9][i % 9] == 0) && (grid[(i - (i % 9))/9][(i % 9) - 1] == cVal)) {
      if (mVal == 1) {
        num++;
        for (int k = 0; k < move; k++) {
         paths[num][k] = paths[e][k];
         if (k > 1) {
            if (paths[num][k] - paths[num][k-1] == 9) {
              cH = 1;
            } else if (paths[num][k] - paths[num][k-1] == -9) {
              cH = 2;
            } else if (paths[num][k] - paths[num][k-1] == 1) {
              cH = 3;
            } else if (paths[num][k] - paths[num][k-1] == -1) {
              cH = 4;
            }
            if (cH == h) {
            } else if (((cH == 1) && (h == 2)) || ((cH == 2) && (h == 1)) || ((cH == 3) && (h == 4)) || ((cH == 4) && (h == 3))) {
              turns[num][0] = turns[num][0] + 2;
              turns[num][1] = cH;
              h = cH;
            } else {
              turns[num][0] = turns[num][0] + 1;
              turns[num][1] = cH;
              h = cH;
            }
          }
        }
        turns[num][0] = turns[e][0];
        turns[num][1] = turns[e][1];
        paths[num][move] = i - 1;
        //CalcTurns(4, num, turns);
      } else {
        paths[e][move] = i - 1;
        mVal = 1;
        CalcTurns(4, e, turns);
      }
    }
    if (e == num) {
      move++;
      cVal--;
      e = 0;
      while (paths[e][move-1] == 50) {
        e++;
      }
      i = paths[e][move-1];
    } else {
      e++;
      if ((e == num) && (paths[e][move] != 50)) {
        move++;
        cVal--;
        e = 0;
      } else {
        while (e != num) {
          if ((paths[e][move] == 50) && (paths[e][move-1] != 50)) {
            break;
          } else {
            e++;
          }
        }
      }
      if ((e == num) && (paths[e][move] != 50)) {
        move++;
        cVal--;
        e = 0;
      }
      i = paths[e][move-1];
    }
  }

  // Printing shortest paths
  i = 0;
  j = 0;
  int k = 0;
  mVal = 0;
  rowNum = 0;
  colNum = 0;
  while (i < (num + 1)) {
    rowNum = 0;
    if (paths[i][initVal] != 50) {
      cout << "[z5206762_MTRN4110_PhaseB] Path - " << (i + 1) << ":" << endl;
      output << "[z5206762_MTRN4110_PhaseB] Path - " << (i + 1) << ":" << endl;
      while (rowNum < 5) {
        k = 0;
        cout << "[z5206762_MTRN4110_PhaseB]  ";
        output << "[z5206762_MTRN4110_PhaseB]  ";
        while (k < 9) {
          if (horizontalWalls[rowNum][k] == 1) {
            cout << "--- ";
            output << "--- ";
          } else {
            cout << "    ";
            output << "    ";
          }
          k++;
        }
        cout << endl << "[z5206762_MTRN4110_PhaseB] |";
        output << endl << "[z5206762_MTRN4110_PhaseB] |";
        colNum = 0;
        while (colNum < 9) {
          j = 0;
          while (j < (initVal + 1)) {
            mVal = 0;
            if (paths[i][j] == (9*rowNum + colNum)) {       
              if (j == 0) {
                if (heading == 1) {
                  cout << " v ";
                  output << " v ";
                  mVal = 1;
                  break;
                } else if (heading == 2) {
                  cout << " ^ ";
                  output << " ^ ";
                  mVal = 1;
                  break;
                } else if (heading == 3) {
                  cout << " > ";
                  output << " > ";
                  mVal = 1;
                  break;
                } else {
                  cout << " < ";
                  output << " < ";
                  mVal = 1;
                  break;
                }
              } else {
                if (grid[rowNum][colNum] > 9) {
                  cout << " " << grid[rowNum][colNum];
                  output << " " << grid[rowNum][colNum];
                  mVal = 1;
                  break;
                } else {
                  cout << " " << grid[rowNum][colNum] << " ";
                  output << " " << grid[rowNum][colNum] << " ";
                  mVal = 1;
                  break;
                }
              }
            }
            j++;
          }
          if (mVal == 0) {
            cout << "   ";
            output << "   ";
          }
          if (verticalWalls[rowNum][colNum+1] == 1) {
            cout << "|";
            output << "|";
          } else {
            cout << " ";
            output << " ";
          }
          colNum++;
        }
        cout << endl;
        output << endl;
        rowNum++;
      }
      cout << "[z5206762_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << endl;
      output << "[z5206762_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << endl;
    }
    i++;
  }
  
  cout << "[z5206762_MTRN4110_PhaseB] " << (num + 1) << " shortest paths found!" << endl << "[z5206762_MTRN4110_PhaseB] Finding shortest path with least turns..." << endl;
  output << "[z5206762_MTRN4110_PhaseB] " << (num + 1) << " shortest paths found!" << endl << "[z5206762_MTRN4110_PhaseB] Finding shortest path with least turns..." << endl;
  // Finding shortest path
  i = 0;
  cVal = 0;
  int size = turns[0][0];
  while (i < num) {
    if (turns[i+1][0] < size) {
      size = turns[i+1][0];
      cVal = i + 1;
    }
    i++;
  }
  
  i = cVal;
  j = 0;
  k = 0;
  mVal = 0;
  rowNum = 0;
  colNum = 0;
  while (rowNum < 5) {
    k = 0;
    cout << "[z5206762_MTRN4110_PhaseB]  ";
    output << "[z5206762_MTRN4110_PhaseB]  ";
    while (k < 9) {
      if (horizontalWalls[rowNum][k] == 1) {
        cout << "--- ";
        output << "--- ";
      } else {
        cout << "    ";
        output << "    ";
      }
      k++;
    }
    cout << endl << "[z5206762_MTRN4110_PhaseB] |";
    output << endl << "[z5206762_MTRN4110_PhaseB] |";
    colNum = 0;
    while (colNum < 9) {
      j = 0;
      while (j < (initVal + 1)) {
        mVal = 0;
        if (paths[i][j] == (9*rowNum + colNum)) {       
          if (j == 0) {
            if (heading == 1) {
              cout << " v ";
              output << " v ";
              mVal = 1;
              break;
            } else if (heading == 2) {
              cout << " ^ ";
              output << " ^ ";
              mVal = 1;
              break;
            } else if (heading == 3) {
              cout << " > ";
              output << " > ";
              mVal = 1;
              break;
            } else {
              cout << " < ";
              output << " < ";
              mVal = 1;
              break;
            }
          } else {
            if (grid[rowNum][colNum] > 9) {
              cout << " " << grid[rowNum][colNum];
              output << " " << grid[rowNum][colNum];
              mVal = 1;
              break;
            } else {
              cout << " " << grid[rowNum][colNum] << " ";
              output << " " << grid[rowNum][colNum] << " ";
              mVal = 1;
              break;
            }
          }
        }
        j++;
      }
      if (mVal == 0) {
        cout << "   ";
        output << "   ";
      }
      if (verticalWalls[rowNum][colNum+1] == 1) {
        cout << "|";
        output << "|";
      } else {
        cout << " ";
        output << " ";
      }
      colNum++;
    }
    cout << endl;
    output << endl;
    rowNum++;
  }
  cout << "[z5206762_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << endl;
  output << "[z5206762_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << endl;
  cout << "[z5206762_MTRN4110_PhaseB] Shortest path with least turns found!" << endl;
  output << "[z5206762_MTRN4110_PhaseB] Shortest path with least turns found!" << endl;
  cout << "[z5206762_MTRN4110_PhaseB] Path Plan (" << (turns[cVal][0] + initVal) << " steps): " << startPos[0][0] << startPos[1][0] << startPos[2][0];
  output << "[z5206762_MTRN4110_PhaseB] Path Plan (" << (turns[cVal][0] + initVal) << " steps): " << startPos[0][0] << startPos[1][0] << startPos[2][0];
  
  char motion[turns[cVal][0] + initVal] = {};
  i = 0;
  j = 0;
  h = heading;
  cH = heading;
  while (i < initVal) {
    if (paths[cVal][i+1] - paths[cVal][i] == 9) {
      cH = 1;
    } else if (paths[cVal][i+1] - paths[cVal][i] == -9) {
      cH = 2;
    } else if (paths[cVal][i+1] - paths[cVal][i] == 1) {
      cH = 3;
    } else if (paths[cVal][i+1] - paths[cVal][i] == -1) {
      cH = 4;
    }
    if (cH == h) {
      cout << "F";
      output << "F";
      motion[j] = 'F';
      j++;
    } else if (((cH == 1) && (h == 2)) || ((cH == 2) && (h == 1)) || ((cH == 3) && (h == 4)) || ((cH == 4) && (h == 3))) {
      cout << "LL";
      output << "LL";
      motion[j] = 'L';
      j++;
      motion[j] = 'L';
      j++;
      h = cH;
    } else if (((cH == 3) && (h == 1)) || ((cH == 2) && (h == 3)) || ((cH == 4) && (h == 2)) || ((cH == 1) && (h == 4))) {
      cout << "LF";
      output << "LF";
      motion[j] = 'L';
      j++;
      motion[j] = 'F';
      j++;
      h = cH;
    } else {
      cout << "RF";
      output << "RF";
      motion[j] = 'R';
      j++;
      motion[j] = 'F';
      j++;
      h = cH;
    }
    i++;
  }
  
  cout << endl << "[z5206762_MTRN4110_PhaseB] Writing path plan to ../../PathPlan.txt..." << endl;
  output << endl << "[z5206762_MTRN4110_PhaseB] Writing path plan to ../../PathPlan.txt..." << endl;
  
  pathPlan << startPos[0][0] << startPos[1][0] << startPos[2][0];
  i = 0;
  while (i < (turns[cVal][0] + initVal)) {
    pathPlan << motion[i];
    i++;
  } 
  pathPlan.close();
  
  cout << "[z5206762_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!";
  output << "[z5206762_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!";
  
  output.close();
  
  return 0;
}

void CalcTurns(int x, int row, int turns[][2]) {
  int i = x;
  if (i == turns[row][1]) {
    return;
  } else if (((x == 1) && (turns[row][1] == 2)) || ((x == 2) && (turns[row][1] == 1)) || ((x == 3) && (turns[row][1] == 4)) || ((x == 4) && (turns[row][1] == 3))) {
    turns[row][0] = turns[row][0] + 2;
    turns[row][1] = x;
  } else {
    turns[row][0] = turns[row][0] + 1;
    turns[row][1] = x;
  }  
}