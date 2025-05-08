#include <iostream>
#include "map.h"
#include "task.h"
#include "lowLevel.h"
#include "heuristic.h"
#include "highLevel.h"

int main() {
    Map map;
    Task task;
    map.getMap("../MapData/Task4.xml");
    task.getTask("../AgentData/Task4_10Agent.xml");
    task.getXY(map);

    CBS cbs;
    Solution res=cbs.findSolution(map,task);

    res.initCost=0;
    for(size_t i=0;i<res.paths.size();i++){
          res.initCost+=res.paths[i].cost;
    }

    std::cout<<res.initCost;

    return 0;
}
