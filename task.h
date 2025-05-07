#ifndef RAILWAYCBS_TASK_H
#define RAILWAYCBS_TASK_H

#include "dataStruct.h"
#include "tinyxml2.h"
#include "iostream"
#include "map.h"
class Task {
private:
    std::vector<Agent> agents;
public:
    Task(){agents.clear();}
    bool getTask(const char* FileName);
    int getAgentSize(){return agents.size();}

    Agent getAgent(int id){
        if(id>=0&&id<int(agents.size()))
            return agents[id];
        else
            return Agent();
    }

    void getXY(Map &map);

};


#endif //RAILWAYCBS_TASK_H
