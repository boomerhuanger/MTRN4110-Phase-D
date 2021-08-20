// File:          z5205986_MTRN4110_PhaseB.cpp
// Date:
// Description:   Controller of e-puck for Phase B - Path Planning
// Author:        William Huang z5205986 
// Modifications:
// Platform: Windows

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <limits.h>
#include <algorithm>

constexpr int TOTAL_CELLS{45};
constexpr char VERTICAL_WALL{'|'};
constexpr char HORIZONTAL_WALL{'-'};
constexpr int ROW_SIZE{9};
constexpr char START[4] = {'^', '>', 'v', '<'};
constexpr char FINISH{'x'};
constexpr int MAZE_1ST_ROW_LEFT_LIMIT{0};
constexpr int MAZE_1ST_ROW_RIGHT_LIMIT{8};
constexpr int MAZE_2ND_ROW_LEFT_LIMIT{9};
constexpr int MAZE_2ND_ROW_RIGHT_LIMIT{17};
constexpr int MAZE_3RD_ROW_LEFT_LIMIT{18};
constexpr int MAZE_3RD_ROW_RIGHT_LIMIT{26};
constexpr int MAZE_4TH_ROW_LEFT_LIMIT{27};
constexpr int MAZE_4TH_ROW_RIGHT_LIMIT{35};
constexpr int MAZE_5TH_ROW_LEFT_LIMIT{36};
constexpr int MAZE_5TH_ROW_RIGHT_LIMIT{44};
constexpr int MAZE_1ST_ROW{1};
constexpr int MAZE_2ND_ROW{3};
constexpr int MAZE_3RD_ROW{5};
constexpr int MAZE_4TH_ROW{7};
constexpr int MAZE_5th_ROW{9};
constexpr int MAZE_1ST_COLUMN[2] = {0, 2};
constexpr int MAZE_2ND_COLUMN[2] = {1, 6};
constexpr int MAZE_3RD_COLUMN[2] = {2, 10};
constexpr int MAZE_4TH_COLUMN[2] = {3, 14};
constexpr int MAZE_5TH_COLUMN[2] = {4, 18};
constexpr int MAZE_6TH_COLUMN[2] = {5, 22};
constexpr int MAZE_7TH_COLUMN[2] = {6, 26};
constexpr int MAZE_8TH_COLUMN[2] = {7, 30};
constexpr int MAZE_9TH_COLUMN[2] = {8, 34};
constexpr char NORTH{'N'};
constexpr char EAST{'E'};
constexpr char SOUTH{'S'};
constexpr char WEST{'W'};
constexpr char LEFT{'L'};
constexpr char RIGHT{'R'};
constexpr char FORWARD{'F'};

// All the webots classes are defined in the "webots" namespace
using namespace webots;

void addEdge(std::vector<int> graph[], int source, int destination) {
    graph[source].push_back(destination);
    graph[destination].push_back(source);
}

//finds the adjacent parent nodes for every vertex in the map
void findParentVertices(std::vector<int> parentVertices[], int startVertex, int finishVertex, std::vector<int> graph[]) {
    //used to store the adjacent parent vertices for each vertex in the map
    //used to store the distance from the start vertex to each vertex in the map and initialise all distances to be INFINITY_VALUE 
    //(actually highest value possible in C++ but effectively INFINITY_VALUE)
    const auto INFINITY_VALUE = std::numeric_limits<int>::max();
    int distanceFromStart[TOTAL_CELLS];
    std::fill_n(distanceFromStart, TOTAL_CELLS, INFINITY_VALUE);
 
    //queue which stores the vertices to visit 
    std::queue<int> verticesToVisit;
 
    //Push the starting vertex into the queue so the vertices adjacent to the starting vertex will be visited 
    verticesToVisit.push(startVertex);

    //distance of starting vertex from itself is 0
    distanceFromStart[startVertex] = 0;

    //Loop until there are no more vertices to visit
    while (!verticesToVisit.empty()) {
        //obtain the vertex at the front of the queue and remove it from the queue
        auto currentlyVisitedVertex = verticesToVisit.front();
        verticesToVisit.pop();
        
        for (auto adjacentToCurrentlyVisitedVertex : graph[currentlyVisitedVertex]) {
            //distance between the starting vertex and vertex adjacent to the vertex currently being visited is less than the distance 
            //from the start vertex to the node currently being visited 
            if (distanceFromStart[currentlyVisitedVertex] + 1 < distanceFromStart[adjacentToCurrentlyVisitedVertex]) {
                
                //push vertex adjacent to current visited vertex onto the queue so that it can be visited
                verticesToVisit.push(adjacentToCurrentlyVisitedVertex);
                
                ///Update distance as a shorter path has been found
                distanceFromStart[adjacentToCurrentlyVisitedVertex] = distanceFromStart[currentlyVisitedVertex] + 1;
              
                //clear all parent vertices as a shorter path has been found
                parentVertices[adjacentToCurrentlyVisitedVertex].clear();
                
                //the currently visited node is a parent to the nodes which are adjacent to and visited after it
                parentVertices[adjacentToCurrentlyVisitedVertex].push_back(currentlyVisitedVertex);
                
            } else if (distanceFromStart[adjacentToCurrentlyVisitedVertex] == distanceFromStart[currentlyVisitedVertex] + 1) {
                //there is more than one shortest way from the start to the vertex adjacent to the currently visited vertex so add
                //currently visited vertex to parent nodes
                parentVertices[adjacentToCurrentlyVisitedVertex].push_back(currentlyVisitedVertex);
                
            }
        }
    }
}

//Recursive backwards to find shortest path from finishVertex to startVertex
//once you have reached -INFINITY_VALUE as a parent node (as set above), the shortest path has been found 
void calculateShortestPaths(std::vector<int> parentVertices[], std::vector<int> &shortestPath, std::vector<std::vector<int>> &allShortestPaths, int startVertex, int currentVertex) {

    // Base Case- when you have reached the start vertex, a shortest path has been found, which is then pushed to all the possible shortest paths
    if (currentVertex == startVertex) {
        allShortestPaths.push_back(shortestPath);
        return;
    }

    //Recurse through all the parent vertices to find the shortest path
    for (auto parentVertex : parentVertices[currentVertex]) {
        //push the current parentVertex onto the shortest path
        shortestPath.push_back(parentVertex);

        // Recursive call for its parent
        calculateShortestPaths(parentVertices, shortestPath, allShortestPaths, startVertex, parentVertex);
        
        //clear the shortest path after finding a shortest path to find another shortest path
        shortestPath.pop_back();  
    }
}
 
//find all shortest paths from the start to finish vertex
std::vector<std::vector<int>> findAllShortestPaths(int startVertex, int finishVertex, std::vector<int> graph[]) {

    //find parent vertices adjacent to every vertex in the map
    std::vector<int> parentVertices[TOTAL_CELLS];
    findParentVertices(parentVertices, startVertex, finishVertex, graph);

    //used to store all shortest paths
    std::vector<std::vector<int>> allShortestPaths;
    
    //used to store a shortest path and add the final vertex to it
    std::vector<int> shortestPath;
    shortestPath.push_back(finishVertex);

    //calculating all the shortest paths to this final vertex
    calculateShortestPaths(parentVertices, shortestPath, allShortestPaths, startVertex, finishVertex);
    
    return allShortestPaths;
}

void runPhaseB(Robot *robot, int start, int finish, bool phaseB, bool wallFollowingMode, std::string pathPlanPath, char &wallFollowing, bool &firstLoop) {
    const auto epuckRobotName = "e-puck";
    
    if (robot->getName() == epuckRobotName) {
        const std::string PATH_PLAN_FILE_NAME = pathPlanPath; 
        const auto MAP_FILE_NAME = "../../Map.txt";
        const std::string studentInfo = "[z5205986_MTRN4110_PhaseB] ";  
        const auto OUTPUT_FILE_NAME = "../../Output.txt";
        const auto READING_IN_MAP = studentInfo + "Reading in map from " + MAP_FILE_NAME + "...";
        const auto MAP_READ_IN = studentInfo + "Map read in!";
        const auto FINDING_SHORTEST_PATHS = studentInfo + "Finding shortest paths..."; 
        const auto SHORTEST_PATHS = " shortest paths found!";
        const auto SHORTEST_PATH_LEAST_TURNS = "Finding shortest path with least turns...";
        const auto SHORTEST_PATH_WITH_LEAST_TURNS_FOUND = "Shortest path with least turns found!";
        const auto WRITING_PATH_PLAN = "Writing path plan to " + PATH_PLAN_FILE_NAME + "...";
        const auto PATH_PLAN_WRITTEN = "Path plan written to " + PATH_PLAN_FILE_NAME + "!";
    
        //open the outputFile for writing everything that goes into the console into the output file as well
        std::ofstream outputFile{OUTPUT_FILE_NAME, std::ios::trunc};
        
        if (phaseB == true) {
            std::cout << READING_IN_MAP << std::endl;
        }
        
        outputFile << READING_IN_MAP << std::endl;
        
        //read in the map into vector of strings motionPath and output map to console and output file
        std::ifstream file {MAP_FILE_NAME, std::ios::in};
        std::string motionPathLine;
        std::vector<std::string> motionPath;
        if(file.is_open()) {
            while (std::getline(file, motionPathLine)) {
                //std::cout << motionPathLine.size() << std::endl;
                motionPath.push_back(motionPathLine);
                if (phaseB == true) {
                    std::cout << studentInfo << motionPathLine << std::endl;
                }
               
                outputFile << studentInfo << motionPathLine << std::endl;
            }
        }
    
        //need to make a graph based on the given map- we shall use adjacency lists
        std::vector<int> graph[TOTAL_CELLS];
        
        //used to store the locations of the vertexes 
        auto source = 0;
        auto destination = 0;
    
        //working out horizontally adjacent nodes
        for(auto row = 1; row <= ROW_SIZE; row += 2) {
            auto line = motionPath[row];
            for (std::string::size_type column = 0; column < line.size(); column += 4) {
                //not the side walls
                if (column != 0 && column != 36) {
                    if (line[column] != VERTICAL_WALL) {
                        //when there is no vertical wall, two adjacent vertices are connected horizontally
                        source = column / 4 - 1 + ROW_SIZE*((row - 1)/2);
                        destination = source + 1;
                        addEdge(graph, source, destination);
                    } 
                } 
            }
        }
    
        //working out vertically adjacent nodes
        for(auto row = 2; row <= 8; row += 2) {
            auto line = motionPath[row];
            for (std::string::size_type column = 2; column < line.size(); column += 4) {
                if (line[column] != HORIZONTAL_WALL) {
                    //when there is no horizontally wall, two adjacent vertices are connected vertically
                    source = (column + 2)/ 4 - 1 + ROW_SIZE*((row - 2)/2);
                    destination = source + ROW_SIZE;
                    addEdge(graph, source, destination);
                }     
            }
        } 
        
        //store the location of the starting and ending vertex
        auto startVertex = start;
        auto finishVertex = finish;
        
    
        //which direction the robot is initially facing 
        auto initiallyFacing = 'v';
        if (wallFollowingMode == true) {
            if (wallFollowing == NORTH_DIRECTION) {
                initiallyFacing = START[0];
            } else if (wallFollowing == EAST_DIRECTION) {
                initiallyFacing = START[1];
            } else if (wallFollowing == SOUTH_DIRECTION) {
                initiallyFacing = START[2];
            } else if (wallFollowing == WEST_DIRECTION) {
                initiallyFacing = START[3];
            }
        }
        
        if (phaseB == true) {
            //working out where the start and finish vertices are
            for (auto row = 1; row <= ROW_SIZE; row += 2) {
                auto line = motionPath[row];
                for (std::string::size_type column = 2; column < line.size(); column += 4) {
                    if (line[column] == START[0] || line[column] == START[1] || line[column] == START[2] || line[column] == START[3]) {
                        startVertex = (column + 2)/ 4 - 1 + ROW_SIZE*((row - 1)/2);
                        
                        //update direction robot is initally facing
                        initiallyFacing = line[column];
                    } else if (line[column] == FINISH) {
                        finishVertex = (column + 2)/ 4 - 1 + ROW_SIZE*((row - 1)/2);
                    }
                }
            }
        }
       
        //stores the final path plan of the shortest path with the least turns
        std::vector<char> pathPlan;
    
        //add the starting location and initial facing of the robot
        auto startingVertexXCoordinate = startVertex / ROW_SIZE;
        auto startingVertexYCoordinate = startVertex - startingVertexXCoordinate * ROW_SIZE;
    
        //push initial starting coordinates of the robot into path plan
        pathPlan.push_back(startingVertexXCoordinate + '0');
        pathPlan.push_back(startingVertexYCoordinate + '0');
    
        std::cout << MAP_READ_IN << std::endl;
        outputFile << MAP_READ_IN << std::endl;
        
        std::cout << FINDING_SHORTEST_PATHS << std::endl;
        outputFile << FINDING_SHORTEST_PATHS << std::endl;
        
        //find all shortest paths using a modification of BFS
        //BFS original source: https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/
        std::vector<std::vector<int>> allShortestPaths = findAllShortestPaths(startVertex, finishVertex, graph);
        
        //store all possible shortest path maps into vector of vector of strings 
        std::vector<std::vector<std::string>> shortestPathMaps;
    
        //determine which row and column index to print the vertex at
        auto rowIndex = 0;
        auto columnIndex = 0;
        auto lengthOfTotalPath = allShortestPaths[0].size() - 1;
        auto pathNumber = 0;
        long long unsigned int pathLengthAtLeastTen = 10;
    
        for (auto path : allShortestPaths) {
            //track how much of the shortest path has been stored
            pathNumber += 1;
            long long unsigned int lengthOfCurrentPath = 0;
            //make a copy of the entire map in preparation for adding the shortest path to it
            auto completedMotionPath = motionPath;
            if (phaseB == true) {
                std::cout << studentInfo << "Path - " << pathNumber << ":" << std::endl;
            }
            
            outputFile << studentInfo << "Path - " << pathNumber << ":" << std::endl;
    
            for (auto vertex : path) {
                //std::cout << "Current vertex is " << vertex << std::endl;
                if (vertex >= MAZE_1ST_ROW_LEFT_LIMIT && vertex <= MAZE_1ST_ROW_RIGHT_LIMIT) {
                    //vertex in 1st row of the maze
                    rowIndex = MAZE_1ST_ROW;
                } else if (vertex >= MAZE_2ND_ROW_LEFT_LIMIT && vertex <= MAZE_2ND_ROW_RIGHT_LIMIT) {
                    //vertex in 2nd row of the maze
                    rowIndex = MAZE_2ND_ROW;
                } else if (vertex >= MAZE_3RD_ROW_LEFT_LIMIT && vertex <= MAZE_3RD_ROW_RIGHT_LIMIT) {
                    //vertex in 3rd row of the maze
                    rowIndex = MAZE_3RD_ROW;
                } else if (vertex >= MAZE_4TH_ROW_LEFT_LIMIT && vertex <= MAZE_4TH_ROW_RIGHT_LIMIT) {
                    //vertex in 4th row of the maze
                    rowIndex = MAZE_4TH_ROW; 
                } else if (vertex >= MAZE_5TH_ROW_LEFT_LIMIT && vertex <= MAZE_5TH_ROW_RIGHT_LIMIT) {
                    //vertex in 5th row of the maze
                    rowIndex = MAZE_5th_ROW;
                }
    
                //calculate what column the vertex is at (left-most column has index 0 and the right-most index has index 8)
                auto column = vertex % ROW_SIZE;
                
                if (column == MAZE_1ST_COLUMN[0]) {
                    columnIndex = MAZE_1ST_COLUMN[1];
                } else if (column == MAZE_2ND_COLUMN[0]) {
                    columnIndex = MAZE_2ND_COLUMN[1];
                } else if (column == MAZE_3RD_COLUMN[0]) {
                    columnIndex = MAZE_3RD_COLUMN[1];
                } else if (column == MAZE_4TH_COLUMN[0]) {
                    columnIndex = MAZE_4TH_COLUMN[1];
                } else if (column == MAZE_5TH_COLUMN[0]) {
                    columnIndex = MAZE_5TH_COLUMN[1];
                } else if (column == MAZE_6TH_COLUMN[0]) {
                    columnIndex = MAZE_6TH_COLUMN[1];
                } else if (column == MAZE_7TH_COLUMN[0]) {
                    columnIndex = MAZE_7TH_COLUMN[1];
                } else if (column == MAZE_8TH_COLUMN[0]) {
                    columnIndex = MAZE_8TH_COLUMN[1];
                } else if (column == MAZE_9TH_COLUMN[0]) {
                    columnIndex = MAZE_9TH_COLUMN[1];
                }
    
                //store the length of the current vertex from the the finish vertex as a char
                auto lengthOfCurrentPathAsChar= lengthOfCurrentPath + '0';
                
                //if the length of the shortest path is less than 10, you need to store the length as 2 chars instead of 1
                if (lengthOfCurrentPath >= pathLengthAtLeastTen && lengthOfCurrentPath != lengthOfTotalPath) {
                    completedMotionPath[rowIndex][columnIndex] = lengthOfCurrentPath / 10 + '0';
                    completedMotionPath[rowIndex][columnIndex + 1] = lengthOfCurrentPath % 10 + '0';
                } else if (lengthOfCurrentPath == lengthOfTotalPath) {
                    //starting vertex does not need to need to be updated
                    continue;
                } else {
                    completedMotionPath[rowIndex][columnIndex] = lengthOfCurrentPathAsChar;
                }
                lengthOfCurrentPath += 1;
            }
            
            shortestPathMaps.push_back(completedMotionPath);
    
            //print each entire shortest path
            for (auto line : completedMotionPath) {
                if (phaseB == true) {
                    std::cout << studentInfo;
                }
                
                outputFile << studentInfo;
                for (auto character : line) {
                    if (phaseB == true) {
                        std::cout << character;
                    }
                    
                    outputFile << character;
                }
                if (phaseB == true) {
                    std::cout << std::endl;
                }
                
                outputFile << std::endl;
            }
        }
        
        if (phaseB == true) {
            std::cout << studentInfo << pathNumber << SHORTEST_PATHS << std::endl;
            std::cout << studentInfo << SHORTEST_PATH_LEAST_TURNS << std::endl;
        }
        
        outputFile << studentInfo << pathNumber << SHORTEST_PATHS << std::endl;
    
        outputFile << studentInfo << SHORTEST_PATH_LEAST_TURNS << std::endl;
    
        //calculate the shortest path with the least turns
    
        //must reverse the paths so they start from the beginning first 
        for (auto &path : allShortestPaths) {
            std::reverse(path.begin(), path.end());
        }
    
        pathNumber = 0;
        auto shortestPathNumber = 0;
        const auto INFINITY_VALUE = std::numeric_limits<int>::max();
        auto minimumTurns = INFINITY_VALUE;
        auto turnsInPath = 0;
        auto difference = 0;
        auto initialDirection = NORTH;
        auto currentDirection = NORTH;
        std::vector<int> shortestPathWithLeastTurns;
        
        for (auto path : allShortestPaths) {
            difference = path[1] - path[0];
            turnsInPath = 0;
    
            //determine which direction the robot is initially heading towards and update
            if (difference == 1) {
                initialDirection = EAST;
                currentDirection = initialDirection;
            } else if (difference == ROW_SIZE) {
                initialDirection = SOUTH;
                currentDirection = initialDirection;
            } else if (difference == -1) {
                initialDirection = WEST;
                currentDirection = initialDirection;
            } else if (difference == -ROW_SIZE) {
                initialDirection = NORTH;
                currentDirection = initialDirection;
            }
            
            //robot must turn from the direction it is initially facing to the direction it initially plans on heading
            if (initiallyFacing == START[0]) {
                //if initially facing north
                if (initialDirection == EAST) {
                    turnsInPath += 1;
                } else if (initialDirection == SOUTH) {
                    turnsInPath += 2;
                } else if (initialDirection == WEST) {
                    turnsInPath += 1;
                }
            } else if (initiallyFacing == START[1]) {
                //if initially facing east
                if (initialDirection == SOUTH) {
                    turnsInPath += 1;
                } else if (initialDirection == WEST) {
                    turnsInPath += 2;
                } else if (initialDirection == NORTH) {
                    turnsInPath += 1;
                }
            } else if (initiallyFacing == START[2]) {
                //if initially facing south
                if (initialDirection == WEST) {
                    turnsInPath += 1;
                } else if (initialDirection == NORTH) {
                    turnsInPath += 2;
                } else if (initialDirection == EAST) {
                    turnsInPath += 1;
                }
            } else if (initiallyFacing == START[3]) {
                //if initially facing west
                if (initialDirection == NORTH) {
                    turnsInPath += 1;
                } else if (initialDirection == EAST) {
                    turnsInPath += 2;
                } else if (initialDirection == SOUTH) {
                    turnsInPath += 1;
                }
            }
    
            for (std::vector<int>::size_type i = 1; i < path.size(); i += 1) {
                if ((path[i + 1] - path[i]) != difference) {
                    //if the 2 vertices are not in the same line, there must have been a turn
                    turnsInPath += 1;
                }
                //update this difference
                difference = path[i+1] - path[i];
            }
    
            //update the shortest path with least turns
            if (turnsInPath < minimumTurns) {
                minimumTurns = turnsInPath;
                shortestPathNumber = pathNumber;
                shortestPathWithLeastTurns = path;
    
                //update if existing plan with least amount of turns has already been found
                if (pathPlan.size() == 3) {
                    pathPlan.pop_back();
                } 
                
                //if initially facing north
                if (initiallyFacing == START[0]) {
                    //if initially facing north
                    pathPlan.push_back(NORTH);
                } else if (initiallyFacing == START[1]) {
                    //if initially facing east
                    pathPlan.push_back(EAST);
                } else if (initiallyFacing == START[2]) {
                    //if initially facing south
                    pathPlan.push_back(SOUTH);
                } else if (initiallyFacing == START[3]) {
                    //if initially facing west
                    pathPlan.push_back(WEST);
                }
            }
            pathNumber += 1;
        }
    
        //print the shortest path with least turns to console and output file
        
        auto shortestPathMap = shortestPathMaps[shortestPathNumber];
        for (auto line : shortestPathMap) {
            if (phaseB == true) {
                std::cout << studentInfo;
            }
            
            outputFile << studentInfo;
            for (auto character : line) {
                if (phaseB == true) {
                    std::cout << character; 
                }
                outputFile << character;
            }
            if (phaseB == true) {
                std::cout << std::endl;
            }
            outputFile << std::endl;
        }
 
        std::cout << studentInfo << SHORTEST_PATH_WITH_LEAST_TURNS_FOUND << std::endl;
        outputFile << studentInfo << SHORTEST_PATH_WITH_LEAST_TURNS_FOUND << std::endl;
    
        //initial direction robot is facing has been determined already above
        initialDirection = pathPlan[2];
        if (wallFollowingMode == true) {
            initialDirection = wallFollowing;
        }
        
        currentDirection = initialDirection;
        
        for (auto character : pathPlan) {
            outputFile << character;
        }
        
        for (std::vector<int>::size_type i = 0; i < shortestPathWithLeastTurns.size() - 1; i += 1) {
            difference = shortestPathWithLeastTurns[i + 1] - shortestPathWithLeastTurns[i];
            if (difference == 1) {
                //the next vertex is the right of the current one
                if (currentDirection == EAST) {
                    pathPlan.push_back(FORWARD);
                } else if (currentDirection == NORTH) {
                    //pathPlan.push_back(FORWARD);
                    pathPlan.push_back(RIGHT);
                    pathPlan.push_back(FORWARD);
                } else if (currentDirection == SOUTH) {
                    //pathPlan.push_back(FORWARD);
                    pathPlan.push_back(LEFT); 
                    pathPlan.push_back(FORWARD);   
                } else if (i == 0 && currentDirection == WEST) {
                    //must turn 180 degrees in the very beginning if facing the opposite direction 
                    pathPlan.push_back(LEFT);
                    pathPlan.push_back(LEFT);
                }
                //update current direction robot is heading
                currentDirection = EAST;
            } else if (difference == -1) {
                //the next vertex is to the left of the current one
                if (currentDirection == WEST) {
                    pathPlan.push_back(FORWARD);
                } else if (currentDirection == NORTH) {
                    //pathPlan.push_back(FORWARD);
                    pathPlan.push_back(LEFT);
                    pathPlan.push_back(FORWARD);
                } else if (currentDirection == SOUTH) {
                    //pathPlan.push_back(FORWARD);
                    pathPlan.push_back(RIGHT);
                    pathPlan.push_back(FORWARD);
                } else if (i == 0 && currentDirection == EAST) {
                    //must turn 180 degrees in the very beginning if facing the opposite direction 
                    pathPlan.push_back(LEFT);
                    pathPlan.push_back(LEFT);
                }
                //update current direction robot is heading
                currentDirection = WEST;
            } else if (difference == ROW_SIZE) {
                //the next vertex is to the bottom of the current one
                if (currentDirection == WEST) {
                    //pathPlan.push_back(FORWARD);
                    pathPlan.push_back(LEFT);
                    pathPlan.push_back(FORWARD);
                } else if (currentDirection == EAST) {
                    //pathPlan.push_back(FORWARD);
                    pathPlan.push_back(RIGHT);
                    pathPlan.push_back(FORWARD);
                } else if (currentDirection == SOUTH) {
                    pathPlan.push_back(FORWARD);
                } else if (i == 0 && currentDirection == NORTH) {
                    //must turn 180 degrees in the very beginning if facing the opposite direction 
                    pathPlan.push_back(LEFT);
                    pathPlan.push_back(LEFT);
                }
                //update current direction robot is heading
                currentDirection = SOUTH;
            } else if (difference == -ROW_SIZE) {
                //the next vertex is to the top of the current one
                if (currentDirection == WEST) {
                    //pathPlan.push_back(FORWARD);
                    pathPlan.push_back(RIGHT);
                    pathPlan.push_back(FORWARD);
                } else if (currentDirection == EAST) {
                    //pathPlan.push_back(FORWARD);
                    pathPlan.push_back(LEFT);
                    pathPlan.push_back(FORWARD);
                } else if (currentDirection == NORTH) {
                    pathPlan.push_back(FORWARD);
                } else if (i == 0 && currentDirection == SOUTH) {
                    //must turn 180 degrees in the very beginning if facing the opposite direction 
                    pathPlan.push_back(LEFT);
                    pathPlan.push_back(LEFT);
                }
                //update current direction robot is heading
                currentDirection = NORTH;
            }    
            if (i == 0) {
                //beginning case after turning the robot to face the correct direction it needs to head, move forward
                if (pathPlan[pathPlan.size()-1] != FORWARD) {
                    //only push forward if you were not facing the correct direction in the beginning
                    pathPlan.push_back(FORWARD);
                }
            }
        }
    
        //print the shortest path with least turns to console and output file
        if (phaseB == true) {
            std::cout << studentInfo << "Path Plan (" << pathPlan.size() - 3 << " steps): ";
        }
        
        outputFile << studentInfo << "Path Plan (" << pathPlan.size() - 3 << " steps): ";
        
        auto initialHeading = pathPlan[2];
        for (auto character : pathPlan) {
            std::cout << character;
            outputFile << character;
        }
        if (wallFollowingMode == true) {
            if (firstLoop == false) {
                pathPlan[2] = wallFollowing;
                firstLoop = true;
            } else {
                pathPlan[2] = initialHeading;
            }
            for (auto i = 0; i < (int)pathPlan.size()-3; i++) {
                initialHeading = getHeading(initialHeading, pathPlan[i + 3]);   
            }
            wallFollowing = initialHeading;
        }
        
        for (auto character : pathPlan) {
            //std::cout << character;
            outputFile << character;
        }
    
        std::cout << std::endl;
        outputFile << std::endl;
        
        std::cout << studentInfo << WRITING_PATH_PLAN << std::endl;
        outputFile << studentInfo << WRITING_PATH_PLAN << std::endl;
        auto mode = std::ios::trunc;
        auto m = pathPlan;
        if (wallFollowingMode == true) {
            mode = std::ios::app;  
        } 
        
        std::ofstream pathPlanForWriting{PATH_PLAN_FILE_NAME, mode};
        //store the shortest path with least turns into PathPlan.txt
        for (auto path : pathPlan) {
            pathPlanForWriting << path; 
        }
        if (wallFollowingMode == true) {
            pathPlanForWriting << std::endl;
        }
        
        std::cout << studentInfo << PATH_PLAN_WRITTEN << std::endl;
        outputFile << studentInfo << PATH_PLAN_WRITTEN;   
    }
}
