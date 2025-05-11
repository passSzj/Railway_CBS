#include "highLevel.h"


Solution CBS::findSolution(Map &map, Task &task) {
    this->map=&map;
    this->task=&task;
    h_values.init(map.getMapSize(),task.getAgentSize());

    for(int i=0;i<int(task.getAgentSize());i++){
        Agent agent =task.getAgent(i);
        h_values.count(map,agent);
    }


    initRoot(map,task);
    this->startTime= std::chrono::high_resolution_clock::now();
    CTNode node;
    deadlockmap.clear();
    while(!tree.empty()){

        node=tree.front();
        tree.erase(tree.begin());
        bool valid = conflictCheck(node);
        if(valid){
            return setRes(node);
        }


        //存在冲突
        Conflict conflict =node.getFirstConflict();

        bool pathFound;

        if(conflict.conflictedAgentsID.second!= -1){
            for(int i=0;i<2;i++){
                CTNode newCTNode;
                newCTNode.parent=&node;
                std::pair<std::pair<int,solutionNode>,std::pair<int,solutionNode>> timeAndV=node.checkConflict(&conflict,&map);

                if (timeAndV.first.first != conflict.timeStep) {
                    handleNotEqualTimeStep(&newCTNode, conflict, i, timeAndV, &node);
                } else {
                    if (conflict.time1 == conflict.time2) {
                        handleEqualTimes(&newCTNode, conflict, i, &node);
                    } else {
                        handleUnequalTimes(&newCTNode, conflict, i, &node);
                    }
                }

                newCTNode.setSolution(node.getSolution());



                if(timeAndV.first.first==conflict.timeStep){
                    if(i==0){
                        pathFound = UpdateSolutionByLowLevel(newCTNode,conflict.conflictedAgentsID.first);
                    }else if(i==1){
                        pathFound = UpdateSolutionByLowLevel(newCTNode,conflict.conflictedAgentsID.second);
                    }
                }
                else if(timeAndV.first.first>=timeAndV.second.first){

                    if(i==0){
                        pathFound = UpdateSolutionByLowLevel(newCTNode,conflict.conflictedAgentsID.first);
                    }else if(i==1){
                        pathFound = UpdateSolutionByLowLevel(newCTNode,conflict.conflictedAgentsID.second);
                    }
                }
                else if(timeAndV.first.first<timeAndV.second.first){
                    pathFound = UpdateSolutionByLowLevel(newCTNode,conflict.conflictedAgentsID.second);
                }


                if(timeAndV.first.first!=conflict.timeStep&&!checkNewSolution(newCTNode,conflict.conflictedAgentsID.first,conflict.conflictedAgentsID.second)){
                    if(timeAndV.first.first>=timeAndV.second.first){
                        for(int u=0;u<=timeAndV.first.first-timeAndV.second.first;u++){
                            newCTNode.addConstraints(new Constraint(conflict.conflictedAgentsID.second,timeAndV.second.second,timeAndV.second.first+u));
                        }
                        pathFound = UpdateSolutionByLowLevel(newCTNode,conflict.conflictedAgentsID.second);
                        if(pathFound){
                            newCTNode.cost= getSIC(newCTNode.getSolution());
                            deadlockmap[newCTNode.cost]++;
                            if(deadlockmap[newCTNode.cost]>20){

                                continue;
                            }
                            if(conflictCheck(newCTNode)){
                                return setRes(newCTNode);
                            }
                            if(newCTNode.cost<INT_MAX){
                                newCTNode.conflicts.clear();
                                insertCTNodeByCost(newCTNode);
                            }
                        }
                        break;
                    }else if(timeAndV.first.first<timeAndV.second.first){
                        for(int u=0;u<timeAndV.second.first-timeAndV.first.first;u++){
                            if(i==0){
                                newCTNode.addConstraints(new Constraint(conflict.conflictedAgentsID.first,timeAndV.first.second,timeAndV.first.first+u));
                            }else if(i==1){
                                newCTNode.addConstraints(new Constraint(conflict.conflictedAgentsID.second,timeAndV.first.second,timeAndV.first.first+u));
                            }

                        }
                        if(i==0){
                            pathFound = UpdateSolutionByLowLevel(newCTNode,conflict.conflictedAgentsID.first);
                        }else{
                            pathFound = UpdateSolutionByLowLevel(newCTNode,conflict.conflictedAgentsID.second);
                        }

                        if(pathFound){
                            newCTNode.cost= getSIC(newCTNode.getSolution());
                            deadlockmap[newCTNode.cost]++;

                            if(deadlockmap[newCTNode.cost]>20){
                                continue;
                            }
                            if(newCTNode.cost<INT_MAX){
                                if(conflictCheck(newCTNode)){
                                    return setRes(newCTNode);
                                }
                                newCTNode.conflicts.clear();
                                insertCTNodeByCost(newCTNode);
                            }
                        }
                        break;
                    }

                }

                if(pathFound){
                    newCTNode.cost= getSIC(newCTNode.getSolution());
                    deadlockmap[newCTNode.cost]++;

                    if(deadlockmap[newCTNode.cost]>20){
                        continue;
                    }
                    if(newCTNode.cost<INT_MAX){
                        if(conflictCheck(newCTNode)){
                            return setRes(newCTNode);
                        }
                        newCTNode.conflicts.clear();
                        insertCTNodeByCost(newCTNode);

                    }
                }
            }
        }

    }

    return Solution({},-1);

}


bool CBS::initRoot(Map &map, Task &task) {
    CTNode root;
    solutionPath path;

    for (int i=0; i<int(task.getAgentSize());i++){
        Agent agent = task.getAgent(i);
        std::vector<Constraint*> empty_constraints;
        empty_constraints.clear();
        path = planner.find_path(agent,map,h_values,empty_constraints);
        if(path.cost<0){
            return false;
        }
        root.paths.push_back(path);
        root.cost+=path.cost;
        root.parent= nullptr;

    }
    tree.push_back(root);
    //冲突检测
    return true;
}

void CBS::handleNotEqualTimeStep(CTNode* newCTNode, const Conflict& conflict, int i,
                                 const std::pair<std::pair<int,solutionNode>,std::pair<int,solutionNode>>& timeAndV,
                                          CTNode* p) {
    if (timeAndV.first.first - 1 < conflict.timeStep && timeAndV.second.first < conflict.timeStep) {
        if(i==0){
            newCTNode->addConstraints(p->getConstraint(),
                                      new Constraint(conflict.conflictedAgentsID.first, timeAndV.first.second, timeAndV.first.first));
        }else if(i==1){
            newCTNode->addConstraints(p->getConstraint(),
                                      new Constraint(conflict.conflictedAgentsID.second, timeAndV.first.second, timeAndV.first.first));
        }


    } else if (timeAndV.first.first >= timeAndV.second.first) {

        if(i==0){
            newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.conflictedAgentsID.first, timeAndV.first.second, timeAndV.first.first));
        }else if(i==1){
            newCTNode->addConstraints(p->getConstraint(),
                                      new Constraint(conflict.conflictedAgentsID.second, timeAndV.first.second, timeAndV.first.first));
        }



    } else if (timeAndV.first.first < timeAndV.second.first) {
        newCTNode->addConstraints(p->getConstraint(),
                                  new Constraint(conflict.conflictedAgentsID.second, timeAndV.second.second, timeAndV.second.first));
    }
}


void CBS::handleEqualTimes(CTNode* newCTNode, const Conflict& conflict, int i, CTNode* p) {
    if (conflict.node1.id == -1) {
        if(i==0){
            newCTNode->addConstraints(p->getConstraint(),
                                      new Constraint(conflict.conflictedAgentsID.first,p->getSolution()[i].nodes[conflict.time1],conflict.time1));

        }else if(i==1){
            newCTNode->addConstraints(p->getConstraint(),
                                      new Constraint(conflict.conflictedAgentsID.second,p->getSolution()[i].nodes[conflict.time1],conflict.time2));

        }
    } else {
        if(i==0){
            newCTNode->addConstraints(p->getConstraint(),
                                      new Constraint(conflict.conflictedAgentsID.first, conflict.node1, conflict.time1));
        }else if(i==1){
            newCTNode->addConstraints(p->getConstraint(),
                                      new Constraint(conflict.conflictedAgentsID.second, conflict.node1, conflict.time2));
        }
    }
}

void CBS::handleUnequalTimes(CTNode *newCTNode, const Conflict &conflict, int i, CTNode *p) {

    int Time0 = conflict.time1;
    int Time1 = (i == 0 && conflict.time1 < conflict.time2)
                ? p->getNextTime(conflict.time2, p->getSolution()[1].nodes)
                : conflict.time2;

    int startTime = (conflict.time1 < conflict.time2) ? Time0 : Time1;
    int endTime = (conflict.time1 < conflict.time2) ? Time1 : Time0;
    if (i == 1) {
        startTime = (conflict.time1 <= conflict.time2) ? Time1 : Time0;
        endTime = (conflict.time2<= conflict.time1) ? Time0 : Time1;
    }

    for (int s = 0; s <= endTime - startTime; s++) {
        if (s == 0) {
            if(i==0){
                newCTNode->addConstraints(p->getConstraint(),
                                          new Constraint(conflict.conflictedAgentsID.first, conflict.node1, conflict.time1 + s));
            }else if(i==1){
                newCTNode->addConstraints(p->getConstraint(),
                                          new Constraint(conflict.conflictedAgentsID.second, conflict.node1, conflict.time2+ s));
            }

        } else {
            if(i==0){
                newCTNode->addConstraints(new Constraint(conflict.conflictedAgentsID.first, conflict.node1, conflict.time1 + s));
            }else if(i==1){
                newCTNode->addConstraints(new Constraint(conflict.conflictedAgentsID.second, conflict.node1, conflict.time2+ s));
            }

        }
    }
}

bool CBS::UpdateSolutionByLowLevel(CTNode &node, int agentIndex){
    std::vector<Constraint*> cons= node.getConstraint();
    Path path=planner.find_path(task->getAgent(agentIndex),*map,h_values,cons);
    node.paths[agentIndex]=path;
    return true;
}

bool CBS::checkNewSolution(CTNode& node,int agent0,int agent1){
    bool valid=true;
    int lastTimeStep=0;
    solutionPath p1= node.paths[agent0];
    solutionPath p2=node.paths[agent1];

    std::vector<solutionPath> solution={p1,p2};
    for(size_t i=0;i<solution.size();i++){
        int currentNodeSize =solution[i].nodes.size();
        if(lastTimeStep<currentNodeSize){
            lastTimeStep=currentNodeSize;
        }
    }


    for(int i=0;i<lastTimeStep;i++){

        for(size_t j=0;j<solution.size();j++){

            if(solution[j].nodes.size()==0) continue;

            int a=std::min((int)solution[j].nodes.size()-1,i);

            for(size_t k=0;k<solution.size();k++){
                if(j==k) continue;
                if(solution[k].nodes.size()==0) continue;
                int b=std::min((int)solution[k].nodes.size()-1,i);


                if(solution[j].nodes[a]== solution[k].nodes[b]){

                    valid= false;
                    return valid;
                }

                if(a!=(int)solution[j].nodes.size()-1&&b!=(int)solution[k].nodes.size()-1){
                    if((solution[j].nodes[a+1]==solution[k].nodes[b])&&(solution[j].nodes[a]==solution[k].nodes[b+1])){
                        valid= false;
                        return valid;
                    }
                }
            }
        }
    }

    return false;
}


int CBS::getSIC(std::vector<solutionPath> &solution)
{
    int cost = 0;
    for (size_t i = 0; i < solution.size(); i++)
    {
        cost += solution[i].cost;
    }

    return cost;
}

// 按 cost 升序插入 newCTNode
void CBS::insertCTNodeByCost(CTNode& newCTNode) {
    // 使用 std::lower_bound 找到插入位置
    auto it = std::lower_bound(
            tree.begin(),
            tree.end(),
            newCTNode,
            [](const CTNode& a, const CTNode& b) {
                return a.cost < b.cost; // 按 cost 升序比较
            }
    );
    // 在找到的位置插入 newCTNode
    tree.insert(it, newCTNode);
}



bool CBS::conflictCheck(CTNode &ctnode) {
    bool valid_solution=true;

    std::vector<solutionPath> solution=ctnode.getSolution();
    if(solution.size()==0){return false;}
    int lastTimeStep=0;
    for(size_t i=0;i<solution.size();i++){
        int currentNodeSize =solution[i].nodes.size();
        if(lastTimeStep<currentNodeSize){
            lastTimeStep=currentNodeSize;
        }
    }

    for(int i=0;i<lastTimeStep;i++){    //时间

        for(size_t j=0;j<solution.size();j++){   //路径1
            if(solution[j].nodes.size()==0) continue;
            std::vector<solutionNode> pathJ=solution[j].nodes;
            size_t a=std::min((int)solution[j].nodes.size()-1,i);

            for(size_t k=0;k<solution.size();k++) {   //路径2
                if(j==k) continue;
                if(solution[k].nodes.size()==0) continue;
                std::vector<solutionNode> pathK=solution[k].nodes;
                size_t b=std::min((int)solution[k].nodes.size()-1,i);



                if(solution[j].nodes[a].id== solution[k].nodes[b].id){
                    if(solution[k].nodes[b].g==solution[k].nodes.back().g ||solution[j].nodes[a].g==solution[j].nodes.back().g ){  //到达终点认为不产生冲突
                        continue;
                    }
                    int agent1Time=ctnode.findFirstOccurrenceTime(solution[j].nodes,solution[j].nodes[a]);
                    int agent2Time=ctnode.findFirstOccurrenceTime(solution[k].nodes,solution[k].nodes[b]);

                    //道岔
                    if(pathJ[agent1Time].index==pathK[agent2Time].index){
                        ctnode.addConflict(new Conflict(solution[j].agentID,solution[k].agentID,pathJ[agent1Time],pathK[agent2Time],agent1Time,agent2Time,i));
                    }

                    //顶点
                    if(pathJ[agent1Time].index!=pathK[agent2Time].index){
                        ctnode.addConflict(new Conflict(solution[j].agentID,solution[k].agentID,pathJ[agent1Time],pathK[agent2Time],agent1Time,agent2Time,i));
                    }

                    valid_solution= false;
                    return valid_solution;
                }

                //边冲突
                if(a!=pathJ.size()-1&&b!=pathK.size()-1){
                    if(pathJ[a+1].id==pathK[b].id&&pathJ[a].id==pathK[b+1].id){
                        int agent1Time=ctnode.findFirstOccurrenceTime(solution[j].nodes,solution[j].nodes[a]);
                        int agent2Time=ctnode.findFirstOccurrenceTime(solution[k].nodes,solution[j].nodes[a]);

                        int node1Time=ctnode.findFirstOccurrenceTime(solution[j].nodes,solution[j].nodes[a+1]);
                        int node2Time=ctnode.findFirstOccurrenceTime(solution[k].nodes,solution[k].nodes[b+1]);
                        ctnode.addConflict(new Conflict(solution[j].agentID,solution[k].agentID,pathJ[node1Time],pathK[node2Time],agent1Time,agent2Time,i));
                    }
                }


                //交叉
                if(a!=solution[j].nodes.size()-1&&b!=solution[k].nodes.size()-1){

                    if(doIntersect(solution[j].nodes[a],solution[j].nodes[a+1],solution[k].nodes[b],solution[k].nodes[b+1])){

                        int agent1Time=ctnode.findFirstOccurrenceTime(solution[j].nodes,solution[j].nodes[a]);
                        int agent2Time=ctnode.findFirstOccurrenceTime(solution[k].nodes,solution[k].nodes[b]);
                        ctnode.addConflict(new Conflict(solution[j].agentID,solution[k].agentID,
                                                        solutionNode(),solutionNode(),agent1Time,agent2Time,i));
                        valid_solution= false;
                        return valid_solution;
                    }

                }




            }

        }

    }


    return valid_solution;
}

Solution CBS::setRes(CTNode &ctNode) {
    std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
    std::vector<solutionPath> paths=ctNode.getSolution();
    double cost=0;
    for(size_t i=0;i<paths.size();i++){
        cost+=paths[i].cost;
    }
    std::chrono::duration<double> duration =end_time - startTime;
    solution.time=duration;
    solution.Cost=cost;
    solution.paths=paths;
    solution.found= true;
    return solution;
}


CTNode CBS::retrieveAndPopCTNodeWithLowestCost(){
    if (tree.empty()) {
        return CTNode{}; // 返回默认构造的空节点
    }
    // 查找最小 cost 节点的迭代器
    auto minIt = std::min_element(
            tree.begin(),
            tree.end(),
            [](const CTNode& a, const CTNode& b) {
                return a.cost < b.cost;
            }
    );
    // 保存最小节点的副本
    CTNode minNode = *minIt;
    // 从树中移除该节点
    tree.erase(minIt);
    return minNode;
}



double CBS::crossProduct(solutionNode P1,solutionNode P2,solutionNode P3,solutionNode P4){

    return (map->getX(P2.id)-map->getX(P1.id))*(map->getY(P4.id)-map->getY(P3.id))-(map->getY(P2.id)-map->getY(P1.id))*(map->getX(P4.id)-map->getX(P3.id));
}

bool CBS::doIntersect(solutionNode A,solutionNode B,solutionNode C,solutionNode D){

    double cross1 = crossProduct(A, B, A, C);
    double cross2 = crossProduct(A, B, A, D);
    double cross3 = crossProduct(C, D, C, A);
    double cross4 = crossProduct(C, D, C, B);

    return (cross1 * cross2 < 0) && (cross3 * cross4 < 0 );
}