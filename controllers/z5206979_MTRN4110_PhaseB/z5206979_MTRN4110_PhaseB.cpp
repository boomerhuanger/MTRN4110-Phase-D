/*
 * File:            z5206979_MTRN4110_PhaseB.cpp
 * Date:            30/06/2021
 * Description:     Controller of E-puck for Phase B - Path Planning
 * Author:          Noah Correa
 * Modifications: 
 * Platform:        Windows
 * Notes:           The map is sequence is NOT hard-coded into the program
 */


// Includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstring>
#include <queue>
#include <algorithm>


// Namespaces
using namespace std;

// Constants
#define NROWS 5
#define NCOLS 9

const string PREFIX = "[z5206979_MTRN4110_PhaseB] ";
const string MAP_FILE_NAME  =  "../../Map.txt"; 
const string PATH_PLAN_FILE_NAME  =  "../../PathPlan.txt";
const string OUTPUT_FILE_NAME = "../../Output.txt";

const string EMPTY = "   ";
const string HWALL = "---";
const string VWALL = "|";
const string TARGET = " x ";
const string RBT_SOUTH = " v ";
const string RBT_NORTH = " ^ ";
const string RBT_EAST = " > ";
const string RBT_WEST = " < ";


// Headings
const string HeadingString[] = {"N", "E", "S", "W", "None"};
enum Heading {
    N,
    E,
    S,
    W,
    None
};


// Cell struct
struct Cell {
    int row;
    int col;
    int getIndex() {return this->row * NCOLS + this->col;};
    bool isValid() {return this->row >= 0 && this->row < NROWS && this->col >= 0 && this->col < NCOLS;};
    bool operator==(const Cell& c) {return this->row == c.row && this->col == c.col;};
    bool operator!=(const Cell& c) {return !(*this==c);};
    string to_string() const {return "(" + std::to_string(row) + "," + std::to_string(col) + ")";};
    friend ostream& operator<<(ostream& os, const Cell& c) {os << c.to_string(); return os;};
    Heading directionTo(const Cell& c) {
        Heading h = None;
        if (this->row == c.row) h = (this->col > c.col) ? W : E;
        else if (this->col == c.col) h = (this->row > c.row) ? N : S;
        return h;
    }
    string moveRequired(const Cell& c, Heading h) {
        string move;
        Heading newDir = this->directionTo(c);
        if (h == this->directionTo(c)) move = "F";
        else if ((h==N&&newDir==S)||(h==E&&newDir==W)||(h==S&&newDir==N)||(h==W&&newDir==E)) move = "LLF";
        else {
            if (h == N) move = (newDir == E) ? "RF" : "LF";
            else if (h == E) move = (newDir == S) ? "RF" : "LF";
            else if (h == S) move = (newDir == W) ? "RF" : "LF";
            else if (h == W) move = (newDir == N) ? "RF" : "LF";
        }
        return move;
    }
};


// Robot struct
struct Robot {
    Cell pos;
    Heading heading;
};



// Helper Functions Protoypes
void outputText(string s, fstream &outfile);
template <int R, int C> void parseLine(char mode, string line, int r, int (&walls)[R][C], Robot &robot, Cell &targetPos);
Heading getRobotHeading(string cell);
void floodFill(int (&values)[NROWS][NCOLS], Cell target, int (&hWalls)[NROWS+1][NCOLS], int (&vWalls)[NROWS][NCOLS+1]);
void findShortestPaths(vector<vector<Cell>> &paths, int (&values)[NROWS][NCOLS], Robot robot, Cell target, int (&hWalls)[NROWS+1][NCOLS], int (&vWalls)[NROWS][NCOLS+1]);
int numTurns(vector<Cell> path, Heading startHeading);
string generateMotionPlan(vector<Cell> path, Heading startHeading);
void outputPath(vector<Cell> path, string stringMap[], fstream &outfile);
int *neighbourWalls(int i, int j, int (&hWalls)[NROWS+1][NCOLS], int (&vWalls)[NROWS][NCOLS+1]);
int *neighbourValues(int i, int j, int (&values)[NROWS][NCOLS]);




// Main function
int main(int argc, char **argv) {

    // Open files
    fstream mapfile;
    mapfile.open(MAP_FILE_NAME, ios::in);
    fstream pathfile;
    pathfile.open(PATH_PLAN_FILE_NAME, ios::out);
    fstream outputfile;
    outputfile.open(OUTPUT_FILE_NAME, ios::out);

    // Define walls and cell values arrays
    int vWalls[NROWS][NCOLS+1];
    int hWalls[NROWS+1][NCOLS];

    // Initialise robot and target structs
    Robot robot = {-1, -1, None};
    Cell targetPos = {-1, -1};

    // Read map
    outputText("Reading in map from " + MAP_FILE_NAME + "...\n", outputfile);
    
    string stringMap[2*NROWS+1];
    string line;
    int i = 0;
    while (getline(mapfile, line)) {
        outputText(line + "\n", outputfile);

        // Parse i string
        if (i % 2 == 0) {
            //cout << "Horizontal\n";
            parseLine('H', line, i, hWalls, robot, targetPos);
        } else {
            parseLine('V', line, i, vWalls, robot, targetPos);
            //cout << "Vertical\n";
        }
        stringMap[i] = line;
        i++;
        
    }
    outputText("Map read in!\n", outputfile);

    // Find all shortest paths
    outputText("Finding shortest paths...\n", outputfile);
    int cellValues[NROWS][NCOLS];
    floodFill(cellValues, targetPos, hWalls, vWalls);
    vector<vector<Cell>> paths;
    findShortestPaths(paths, cellValues, robot, targetPos, hWalls, vWalls);
    for (int i = 0; i < (int)paths.size(); i++) {
        outputText("Path - " + std::to_string(i+1) + ":\n", outputfile);
        outputPath(paths.at(i), stringMap, outputfile);
    }
    outputText(std::to_string(paths.size()) + " shortest paths found!\n", outputfile);


    // Find the shortest path with least turns
    outputText("Finding shortest path with least turns...\n", outputfile);
    int minTurns = INT32_MAX;
    int minPath = -1;
    for (int i = 0; i < (int)paths.size(); i++) {
        int min = numTurns(paths.at(i), robot.heading);
        if (min < minTurns) {
            minTurns = min;
            minPath = i;
        }
    }
    vector<Cell> selectedPath = paths.at(minPath);
    outputPath(selectedPath, stringMap, outputfile);
    outputText("Shortest path with least turns found!\n", outputfile);


    // Generate motion sequence of path
    string planString = generateMotionPlan(selectedPath, robot.heading);
    int numSteps = planString.size() - 3;
    outputText("Path Plan (" + std::to_string(numSteps) + " steps): " + planString + "\n", outputfile);
    
    
    // Write path to file
    outputText("Writing path plan to " + PATH_PLAN_FILE_NAME + "...\n", outputfile);
    pathfile << planString;
    outputText("Path plan written to " + PATH_PLAN_FILE_NAME + "!", outputfile);

    
    // Close files
    mapfile.close();
    pathfile.close();
    outputfile.close();

    return 0;
}




/*
 * Helper Functions
 */


// Outputs text to console and file
void outputText(string s, fstream &outfile) {
    cout << PREFIX << s;
    outfile << PREFIX << s;
}


// Parses a line, populating walls arrays and robot and target structs
template <int R, int C> void parseLine(char mode, string line, int r, int (&walls)[R][C], Robot &robotPos, Cell &targetPos) {
    int row = r / 2;
    if (mode == 'H') {
        // Horizontal walls
        bool cell = false;
        int i = 0;
        int col = 0;

        while (i < (int)line.length()) {
            if (cell) {
                // check 3 chars
                string wall = HWALL;
                walls[row][col] = (wall.compare(line.substr(i, 3)) == 0) ? 1 : 0;
                cell = false;
                i = i + 3;
                col++;
            } else {
                // check 1 char
                cell = true;
                i++;
            }
        }
            
    } else if (mode == 'V') {
        // Vertical walls
        bool cell = false;
        int i = 0;
        int col = 0;

        while (i < (int)line.length()) {
            if (cell) {
                // Check 3 chars for robot or target
                string c = line.substr(i, 3);
                if (c.compare(TARGET) == 0) {
                    // Found target cell
                    targetPos = {row, col-1};
                } else if (c.compare(EMPTY) != 0) {
                    // Found robot cell
                    Heading heading = getRobotHeading(c);
                    robotPos = {row, col-1, heading};
                }
                cell = false;
                i = i + 3;
            } else {
                // Check 1 char for vertical wall
                string wall = VWALL;
                walls[row][col] = (wall.compare(line.substr(i, 1)) == 0) ? 1 : 0;
                cell = true;
                i++;
                col++;
            }
        }
    }
}


// Converts ^,>,V,< to heading enum
Heading getRobotHeading(string cell) {
    if (cell.compare(RBT_NORTH) == 0) {
        return N;
    } else if (cell.compare(RBT_EAST) == 0) {
        return E;
    } else if (cell.compare(RBT_SOUTH) == 0) {
        return S;
    } else if (cell.compare(RBT_WEST) == 0) {
        return W;
    } else {
        return None;
    }
}


// Populates array with values using the Flood Fill algorithm
void floodFill(int (&values)[NROWS][NCOLS], Cell target, int (&hWalls)[NROWS+1][NCOLS], int (&vWalls)[NROWS][NCOLS+1]) {

    // Initialise cell values
    for (int i = 0; i < NROWS; i++) {
        for (int j = 0; j < NCOLS; j++) {
            values[i][j] = NROWS*NCOLS;
        }
    }
    // Initialise variables
    values[target.row][target.col] = 0;
    int currValue = 0;
    bool changed = true;

    // Loop until values are not updated
    while (changed) {
        changed = false;

        // Loop through all cells
        for (int i = 0; i < NROWS; i++) {
            for (int j = 0; j < NCOLS; j++) {

                // Check if current cell has same value as the current value
                if (values[i][j] == currValue) {

                    // Walls/Direction Values {N, E, S, W}
                    int *walls =neighbourWalls(i, j, hWalls, vWalls);
                    int dirValues[4] = {values[i-1][j], values[i][j+1], values[i+1][j], values[i][j-1]};

                    // Check each direction
                    for (int d = 0; d < 4; d++) {
                        if (!walls[d] && dirValues[d] == NROWS*NCOLS) {
                            values[i-1][j] = (d == 0) ? currValue+1 : values[i-1][j];
                            values[i][j+1] = (d == 1) ? currValue+1 : values[i][j+1];
                            values[i+1][j] = (d == 2) ? currValue+1 : values[i+1][j];
                            values[i][j-1] = (d == 3) ? currValue+1 : values[i][j-1];
                            
                            changed = true;
                        }
                    }
                    delete walls;
                }
            }
        }
        currValue++;
    }
}



// Populates paths vector with all shortest paths by moves
void findShortestPaths(vector<vector<Cell>> &paths, int (&values)[NROWS][NCOLS], Robot robot, Cell target, int (&hWalls)[NROWS+1][NCOLS], int (&vWalls)[NROWS][NCOLS+1]) {

    vector<Cell> path;
    vector<Cell> queue;
    vector<vector<Cell>> cellPaths;
    queue.push_back(robot.pos);
    bool first = true;
    while (!queue.empty()) {
        // Pop front of queue
        Cell curr = queue.front();
        queue.erase(queue.begin());

        if (!first) {
            path = cellPaths.front();
            cellPaths.erase(cellPaths.begin());
        }

        // Find path to target from current starting cell
        while (curr!=target) {

            path.push_back(curr);

            // Check for reachable neighbours with value 1 less than current value
            vector<Cell> currPrev;
            int i = curr.row;
            int j = curr.col;
            int currValue = values[i][j];
            int *walls = neighbourWalls(i, j, hWalls, vWalls);
            int *neighbours = neighbourValues(i, j, values);
            Cell cellNeighbours[4] = {Cell({i-1,j}), Cell({i,j+1}), Cell({i+1,j}), Cell({i,j-1})};
            
            // Check all directions
            for (int n = 0; n < 4; n++) {
                if (!walls[n] && cellNeighbours[n].isValid() && neighbours[n] == currValue - 1) {
                    currPrev.push_back(cellNeighbours[n]);
                }
            }
            delete walls;
            delete neighbours;
            
            // Target cell reached
            if (currPrev.size() == 0) {
                break;
            // Choices
            } else {
                curr = currPrev.at(0);
                if ((int)currPrev.size() > 1) {
                    for (int i = 1; i < (int)currPrev.size(); i++) {
                        queue.push_back(currPrev.at(i));
                        cellPaths.push_back(path);
                    }
                }
            }

        }

        path.push_back(curr);
        first = false;
        paths.push_back(path);

    }

}


// Calculates number of turns required along a given path and starting heading
int numTurns(vector<Cell> path, Heading startHeading) {
    int num = 0;
    Heading currDir = startHeading;
    for (int i = 0; i < (int)path.size()-1; i++) {
        Cell curr = path.at(i);
        Cell next = path.at(i+1);
        Heading nextDir = curr.directionTo(next);
        if (nextDir!=currDir) num++;
        currDir = nextDir;
        // cout << curr << "->" << next << " = " << HeadingString[currDir] << "->" << HeadingString[nextDir] << "\n";
    }
    return num;
}


// Generates a motion plan from given path and starting heading
string generateMotionPlan(vector<Cell> path, Heading startHeading) {
    string plan;
    Heading currDir = startHeading;
    plan.append(std::to_string(path.at(0).row) + std::to_string(path.at(0).col) + HeadingString[startHeading]);
    for (int i = 0; i < (int)path.size() - 1; i++) {
        Cell curr = path.at(i);
        Cell next = path.at(i+1);
        plan.append(curr.moveRequired(next, currDir));
        currDir = curr.directionTo(next);
    }

    return plan;
}


// Outputs map of given path
void outputPath(vector<Cell> path, string stringMap[], fstream &outfile) {

    string mapCopy [2*NROWS+1];
    for (int i = 0; i < 2*NROWS+1; i++) mapCopy[i] = stringMap[i];

    for (int i = (int)path.size()-2; i >= 0; i--) {
        Cell curr = path.at((int)path.size()-1-i);
        string num;
        num = (i > 9) ? " " + std::to_string(i) : " " + std::to_string(i) + " ";

        mapCopy[2*curr.row+1].replace(4*curr.col+1, 3, num);
    }
    for (int i = 0; i < 2*NROWS+1; i++) {
        outputText(mapCopy[i] + "\n", outfile);
    }
}


// Finds the values of the cells neighbouring cell(i,j)
int *neighbourValues(int i, int j, int (&values)[NROWS][NCOLS]) {
    int *nVals = new int[4];
    nVals[0] = (i == 0) ? NROWS*NCOLS : values[i-1][j];
    nVals[1] = (j == NCOLS-1) ? NROWS*NCOLS : values[i][j+1];
    nVals[2] = (i == NROWS-1) ? NROWS*NCOLS : values[i+1][j];
    nVals[3] = (j == 0) ? NROWS*NCOLS : values[i][j-1];
    return nVals;
}


// Finds the walls of the cells neighbouring cell(i,j)
int *neighbourWalls(int i, int j, int (&hWalls)[NROWS+1][NCOLS], int (&vWalls)[NROWS][NCOLS+1]) {
    int *walls = new int[4]();
    walls[0] = hWalls[i][j];
    walls[1] = vWalls[i][j+1];
    walls[2] = hWalls[i+1][j];
    walls[3] = vWalls[i][j];

    return walls;
}

