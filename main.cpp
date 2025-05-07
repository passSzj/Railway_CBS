#include <iostream>
#include "map.h"
#include "task.h"
#include "lowLevel.h"
#include "heuristic.h"
#include "highLevel.h"

int main() {
    Map map;
    Task task;
    map.getMap("../MapData/Task1.xml");
    task.getTask("../AgentData/Task1_8Agent.xml");
    task.getXY(map);




    CBS cbs;
    Solution res=cbs.findSolution(map,task);
    std::cout<<res.paths[0].agentID<<std::endl;
    int sum=0;
    for(int i=0;i<res.paths.size();i++){
        sum+=res.paths[i].cost;
    }

    std::cout<<"ok "+sum<<std::endl;
//    heuristic h_values;
//    solutionPath path;
//    lowLevel planner;
//    h_values.init(map.getMapSize(),task.getAgentSize());
//
//    for(int i=0;i<int(task.getAgentSize());i++){
//        Agent agent =task.getAgent(i);
//        h_values.count(map,agent);
//    }
//    std::vector<Constraint*> empty_constraints;
//    for (int i=0; i<int(task.getAgentSize());i++){
//        Agent agent = task.getAgent(i);
//        path = planner.find_path(agent,map,h_values,empty_constraints);
//        if(path.cost<0){
//            return false;
//        }
//    }




    return 0;
}
