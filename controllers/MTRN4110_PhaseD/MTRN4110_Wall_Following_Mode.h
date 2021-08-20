#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <array>
#include <algorithm>
#include <typeinfo>
#include <windows.h>

#define DISTANCE_SENSOR_NUMBER 6
#define OBSTACLE_THRESHOLD 800
#define MAX_MOTOR_SPEED 6.28

const auto STARTING_MESSAGE = "Running Wall Following Mode...";
const auto leftMotorDevice = "left wheel motor";
const auto rightMotorDevice = "right wheel motor";



void initialMove(char &initiallyFacing, char &initialDirection, char &heading, std::string &wallFollowingPathPlan, bool fix) {
    if (initiallyFacing == START[0]) {
            //if initially facing north
        if (initialDirection == EAST_DIRECTION) {
            if (fix == true) {
                wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            } else {
                wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            }
            heading = getHeading(heading, RIGHT_DIRECTION);
        } else if (initialDirection == SOUTH_DIRECTION) {
            wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            heading = getHeading(heading, RIGHT_DIRECTION);
            wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            heading = getHeading(heading, RIGHT_DIRECTION);
        } else if (initialDirection == WEST_DIRECTION) {
            if (fix == true) {
                wallFollowingPathPlan.push_back(LEFT_DIRECTION);
            } else {
                wallFollowingPathPlan.push_back(LEFT_DIRECTION);
                heading = getHeading(heading, LEFT_DIRECTION);
            }
        } else {
           // wallFollowingPathPlan.push_back(RIGHT);
            //wallFollowingPathPlan.push_back(RIGHT);
        }
    } else if (initiallyFacing == START[1]) {
        //if initially facing east
        if (initialDirection == SOUTH_DIRECTION) {
            if (fix == true) {
                wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            } else {
                wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                heading = getHeading(heading, RIGHT_DIRECTION);
            }
        } else if (initialDirection == WEST_DIRECTION) {
            wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            heading = getHeading(heading, RIGHT_DIRECTION);
            wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            heading = getHeading(heading, RIGHT_DIRECTION);
        } else if (initialDirection == NORTH_DIRECTION) {
            if (fix == true) {
                wallFollowingPathPlan.push_back(LEFT_DIRECTION);
            } else {
                wallFollowingPathPlan.push_back(LEFT_DIRECTION);
                heading = getHeading(heading, LEFT_DIRECTION);
            }
        } else {
            //wallFollowingPathPlan.push_back(RIGHT);
            //wallFollowingPathPlan.push_back(RIGHT);
            
        }
    } else if (initiallyFacing == START[2]) {
        //if initially facing south
        if (initialDirection == WEST_DIRECTION) {
            if (fix == true) {
                wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            } else {
                wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                heading = getHeading(heading, RIGHT_DIRECTION);
            }
        } else if (initialDirection == NORTH_DIRECTION) {
            wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            heading = getHeading(heading, RIGHT_DIRECTION);
            wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            heading = getHeading(heading, RIGHT_DIRECTION);
        } else if (initialDirection == EAST_DIRECTION) {
            if (fix == true) {
                wallFollowingPathPlan.push_back(LEFT_DIRECTION);
            } else {
                wallFollowingPathPlan.push_back(LEFT_DIRECTION);
                heading = getHeading(heading, LEFT_DIRECTION);
            }
        } else {
            //wallFollowingPathPlan.push_back(RIGHT);
            //wallFollowingPathPlan.push_back(RIGHT);
            
        }
    } else if (initiallyFacing == START[3]) {
        //if initially facing west
        if (initialDirection == NORTH_DIRECTION) {
            if (fix == true) {
                wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            } else {
                wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                heading = getHeading(heading, RIGHT);
            }
        } else if (initialDirection == EAST_DIRECTION) {
            wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            heading = getHeading(heading, RIGHT_DIRECTION);
            wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
            heading = getHeading(heading, RIGHT_DIRECTION);
        } else if (initialDirection == SOUTH_DIRECTION) {
            if (fix == true) {
                wallFollowingPathPlan.push_back(LEFT_DIRECTION);
            } else {
                wallFollowingPathPlan.push_back(LEFT_DIRECTION);
                heading = getHeading(heading, LEFT_DIRECTION);
            }
        } else {
            //wallFollowingPathPlan.push_back(RIGHT);
            //wallFollowingPathPlan.push_back(RIGHT);
            
        }
    }
        
}

void runWallFollowingMode(Robot *robot, int timeStep) {

    const auto epuckRobotName = "e-puck";
    if (robot->getName() == epuckRobotName) {
        const auto MAP_FILE_NAME = "../../Map.txt"; 
        const auto PATH_PLAN_FILE_NAME = "../../Plan.txt";
        std::cout << STARTING_MESSAGE  << std::endl;

        std::string mpline;
        ifstream file(MOTION_PLAN_PATH);
        if (file.is_open()) {
            getline(file, mpline);
            file.close();
        } else {
            std::cout << UNABLE_TO_OPEN_FILE_MESSAGE << std::endl;
        }
        
        //read in the map into vector of strings motionPath and output map to console and output file
        std::ifstream file1 {MAP_FILE_NAME, std::ios::in};
        std::string motionPathLine;
        std::vector<std::string> motionPath;
        if(file1.is_open()) {
            while (std::getline(file1, motionPathLine)) {
                motionPath.push_back(motionPathLine);
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
        
        //if robot is not already on the edges,go to the edges
        int edgeVertices[24] = {0,1,2,3,4,5,6,7,8,17,26,35,44,43,42,41,40,39,38,37,36,27,18,9};
        std::vector<std::vector<int>> shortestDistances;
        auto row = mpline[0] - '0';
        auto col = mpline[1] - '0';
        auto heading = mpline[2];

        auto vertexNumber = row * 9 + col;
        std::string wallFollowingPathPlan;
        
        //move to robot to edges
        for (auto i = 0; i < 24; i++) {
            auto allShortestPaths = findAllShortestPaths(vertexNumber, edgeVertices[i], graph);
            std::vector<int> shortestPath = allShortestPaths[0];
            for (auto path : allShortestPaths) {
                if (path.size() < shortestPath.size()) {
                    shortestPath = path;
                }
            }
            shortestDistances.push_back(shortestPath);
        }

         //must reverse the paths so they start from the beginning first 
        for (auto &path : shortestDistances) {
            std::reverse(path.begin(), path.end());
        }
        
        std::sort(shortestDistances.begin(), shortestDistances.end(), [](const vector<int> & a, const vector<int> & b){ return a.size() < b.size(); });
       
        //move to near the wall if not already against the wall
        
        //fix the initial heading

        wallFollowingPathPlan.push_back(row + '0'); 
        wallFollowingPathPlan.push_back(col + '0'); 
        wallFollowingPathPlan.push_back(heading); 

        auto shortestPathToWallSize = (int) shortestDistances[0].size();

        auto initiallyFacing = heading;
        auto initialDirection = heading;
        if (heading == NORTH_DIRECTION) {
            initiallyFacing = START[0];
        } else if (heading == EAST_DIRECTION) {
            initiallyFacing = START[1];
        } else if (heading == SOUTH_DIRECTION) {
            initiallyFacing = START[2];
        } else if (heading == WEST_DIRECTION) {
            initiallyFacing = START[3];
        }
        
        auto i = 0;
        if (shortestPathToWallSize != 1) {
            while (i < shortestPathToWallSize - 1) {
                auto difference = shortestDistances[0][i + 1] - shortestDistances[0][i];
                if (i == 0) {
                    //before robot moves, must correct heading so it head towards the wall
                    initialMove(initiallyFacing, initialDirection, heading, wallFollowingPathPlan, false);
                }
                if (difference == 1) {
                    //the next vertex is the right of the current one
                    if (heading== EAST) {
                        
                    } else if (heading == NORTH) {
                        //pathPlan.push_back(FORWARD);
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                    } else if (heading == SOUTH) {
                        //pathPlan.push_back(FORWARD);
                        wallFollowingPathPlan.push_back(LEFT); 
                        heading = getHeading(heading, LEFT_DIRECTION);   
                    } else {
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                    }
                    //update current direction robot is heading
                    //heading = EAST;
                    wallFollowingPathPlan.push_back(FORWARD);
                } else if (difference == -1) {
                    //the next vertex is to the left of the current one
                    if (heading == WEST) {
                        
                    } else if (heading == NORTH) {
                        //pathPlan.push_back(FORWARD);
                        wallFollowingPathPlan.push_back(LEFT);
                        heading = getHeading(heading, LEFT_DIRECTION);
                    } else if (heading == SOUTH) {
                        //pathPlan.push_back(FORWARD);
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                    } else {
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                    }
                    //update current direction robot is heading
                    //heading = WEST;
                    wallFollowingPathPlan.push_back(FORWARD);
                } else if (difference == ROW_SIZE) {
                    //the next vertex is to the bottom of the current one
                    if (heading == WEST) {
                        //pathPlan.push_back(FORWARD);
                        wallFollowingPathPlan.push_back(LEFT);
                        heading = getHeading(heading, LEFT_DIRECTION);
                    } else if (heading == EAST) {
                        //pathPlan.push_back(FORWARD);
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                    } else if (heading == SOUTH) {
                        
                    } else {
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                    }
                    //update current direction robot is heading
                    //heading = SOUTH;
                    wallFollowingPathPlan.push_back(FORWARD);
                } else if (difference == -ROW_SIZE) {
                    //the next vertex is to the top of the current one
                    if (heading == WEST) {
                        //pathPlan.push_back(FORWARD);
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                    } else if (heading == EAST) {
                        //pathPlan.push_back(FORWARD);
                        wallFollowingPathPlan.push_back(LEFT);
                        heading = getHeading(heading, LEFT_DIRECTION);
                    } else if (heading == NORTH) {
                        
                    } else {
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                        wallFollowingPathPlan.push_back(RIGHT);
                        heading = getHeading(heading, RIGHT_DIRECTION);
                    }
                    //update current direction robot is heading
                    //heading = NORTH;
                    wallFollowingPathPlan.push_back(FORWARD);
                }
                i += 1;
            }
        } 
        
        if (heading == NORTH_DIRECTION) {
            initiallyFacing  = START[0];
        } else if (heading == EAST_DIRECTION) {
            initiallyFacing  = START[1];
        } else if (heading == SOUTH_DIRECTION) {
            initiallyFacing  = START[2];
        } else if (heading == WEST_DIRECTION) {
            initiallyFacing = START[3];
        }
        
        //NOW ROBOT IS AT CORRECT INITIAL SPOT
        
        auto startVertex = shortestDistances[0][shortestPathToWallSize - 1];
        
        auto start = std::distance(edgeVertices, std::find(edgeVertices, edgeVertices + 24, startVertex));
        auto startVertexCopy = start;

        std::ofstream pathPlanForWriting{PATH_PLAN_FILE_NAME}; 
        auto startedLoop = false;
        auto firstLoop = true;
        i = 0;
        auto finishVertex = 0;
        std::string shortestPathToEdge;
        for (i = 0; i <= 23; i++) {
            startVertex = edgeVertices[i];
            if (i != 23) {
                finishVertex = edgeVertices[i + 1];
            } else {
                finishVertex = edgeVertices[0];   
            }
            if (startedLoop == false) {
                firstLoop = false;
            }
            startedLoop = true;
            runPhaseB(robot, startVertex,finishVertex, false, true, PATH_PLAN_FILE_NAME, heading, firstLoop);
        }
        
        std::vector<std::string> allPaths;
        std::ifstream allPathsFile{PATH_PLAN_FILE_NAME, std::ios::in};
        std::string shortestPath;
        if(!file.is_open()) {
            while (std::getline(allPathsFile, shortestPath)) {
                allPaths.push_back(shortestPath);
            }
        } else {
            std::cout << "Error opening file";
        }
        
        auto firstLine = allPaths[startVertexCopy];
        firstLine.erase(firstLine.begin() + 3, firstLine.end());

        auto correctDirection = allPaths[startVertexCopy][2];
        auto currentDirection = initiallyFacing;
        auto correctDirectionCopy = correctDirection;
        
        if (correctDirectionCopy == NORTH_DIRECTION) {
            correctDirectionCopy = START[0];
        } else if (correctDirectionCopy == EAST_DIRECTION) {
            correctDirectionCopy = START[1];
        } else if (correctDirectionCopy == SOUTH_DIRECTION) {
            correctDirectionCopy= START[2];
        } else if (correctDirectionCopy == WEST_DIRECTION) {
            correctDirectionCopy = START[3];
        }
        
        if (currentDirection != correctDirectionCopy) {
             auto i = 0;
             while (i <= 3) {
                 if (START[i] == currentDirection) {
                     break;
                 }
                 i += 1;
             }
             auto j = 0;
             while (i <= 3) {
                 if (START[j] == correctDirectionCopy) {
                     break;
                 }
                 j += 1;
             }
             auto difference = i - j;
             switch(difference) {
                 case -3:
                     wallFollowingPathPlan.push_back(LEFT_DIRECTION);
                     break;
                 case -2:
                     wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                     wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                     break;
                 case -1:
                     wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                     break;
                 case 0:
                     break;
                 case 1:
                     wallFollowingPathPlan.push_back(LEFT_DIRECTION);
                     break;
                 case 2:
                     wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                     wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                     break;
                 case 3:
                     wallFollowingPathPlan.push_back(RIGHT_DIRECTION);
                     break;    
             }
        }
        wallFollowingPathPlan.erase(wallFollowingPathPlan.begin(), wallFollowingPathPlan.begin() + 3);

        std::string newWallPath = wallFollowingPathPlan;
        auto j = startVertexCopy;
        while (j <= 23) {
            auto path = allPaths[j];
            path.erase(path.begin(), path.begin() + 3);
            for (auto c : path) {
                newWallPath.push_back(c);
            }
            j += 1;
        }
        if (startVertexCopy != 0) {
            auto i = 0;
            allPaths[0] = "00NRF";
            while (i < startVertexCopy) {
                auto path = allPaths[i];
                path.erase(path.begin(), path.begin() + 3);
                for (auto c : path) {
                    newWallPath.push_back(c);
                }
                i += 1;
            }
        }
        runPhaseA(robot, true, newWallPath);
    }
}
           
            