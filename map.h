#ifndef RAILWAYCBS_MAP_H
#define RAILWAYCBS_MAP_H

#include "vector"
#include "dataStruct.h"
#include "tinyxml2.h"
#include <sstream>
#include <fstream>
#include <iostream>
class Map {
private:
    std::vector<Vertex> vertices;
    std::vector<std::vector<Node>> valid_moves;
    int size;

public:

    Map(){}
    ~Map(){}
    bool getMap(const char* file_path);
    std::vector<Node> getValidMoves(int index,int id);
    subVertex getSubVertex(int id,int index);
    int getMapSize(){return size;}
    double getX(int id);
    double getY(int id);
    int getValidMoveSize(int index,int id);

};


#endif //RAILWAYCBS_MAP_H
