#ifndef RAILWAYCBS_HIGHLEVEL_H
#define RAILWAYCBS_HIGHLEVEL_H


#include "map.h"
#include "heuristic.h"
#include "lowLevel.h"
#include "task.h"

class CTNode{
public:
    CTNode(){cost=0;constraint.clear();conflicts.clear();paths.clear();}
    std::vector<Constraint*> constraint;
    std::vector<Conflict*> conflicts;
    std::vector<solutionPath> paths;
    CTNode *parent;
    double cost;
    std::vector<solutionPath>& getSolution(){return this->paths;}
    void addConflict(Conflict* newConflict){conflicts.push_back(newConflict);}
    double findFirstOccurrenceTime(const std::vector<solutionNode>& detailedPath, const solutionNode& targetNode) {
        for (const solutionNode& node : detailedPath) {
            if (node.id == targetNode.id) {
                return node.g; // 返回首次匹配的 g 值
            }
        }
        return -1; // 未找到

    }
    const Conflict& getFirstConflict() const{return *conflicts[0];}






    std::pair<std::pair<int,solutionNode>,std::pair<int,solutionNode>> checkConflict(Conflict *conflict,Map *map){
        int j=conflict->conflictedAgentsID.first;
        int k=conflict->conflictedAgentsID.second;
        int time1=conflict->timeStep;
        int time2=conflict->timeStep;


       if(conflict->node1.id==-1&&conflict->node2.id==-1){
            return std::make_pair(std::make_pair(time1,paths[j].nodes[time1]), std::make_pair(time2,paths[k].nodes[time2]));
       }

        if((map->getY(paths[j].nodes[(conflict->time1 == 0) ? 0 : (conflict->time1 - 1)].id)==map->getY(paths[k].nodes[(conflict->time2 == 0) ? 0 : (conflict->time2 - 1)].id))&&
            (map->getY(paths[j].nodes[conflict->time1+1].id)==map->getY(paths[k].nodes[conflict->time2+1].id))){
            time1 = getConstraintTime(conflict->timeStep,&this->paths[j],map);
            time2 = getConstraintTime(conflict->timeStep,&this->paths[k],map);

            if(time1<conflict->timeStep&&time2<conflict->timeStep){
                return make_pair(std::make_pair(time1+1,paths[j].nodes[time1+1]), std::make_pair(time2,paths[k].nodes[time2]));
            }

            if(time1>time2){
                if(time1>conflict->timeStep){
                    for(size_t index=0;index<this->paths[k].nodes.size();index++){
                        if(this->paths[j].nodes[time1]==this->paths[k].nodes[index]){
                            time2=index;
                            time1= getNextTime(time1,this->paths[j].nodes);
                            break;
                        }
                    }
                }
                else
                    time1++;
            }else{
                if(time2>conflict->timeStep){
                    time2++;
                    for(size_t index=0;index<this->paths[j].nodes.size();index++){
                        if(this->paths[k].nodes[time2-1]==this->paths[j].nodes[index]){
                            time1=index;
                            break;
                        }
                    }
                }
                else
                    time2++;
            }

        }

        return std::make_pair(std::make_pair(time1,paths[j].nodes[time1]), std::make_pair(time2,paths[k].nodes[time2]));

    }

    int getFirstTime(int time,std::vector<solutionNode> nodes){
        for(size_t i=0;i<nodes.size();i++){
            if(nodes[time].id==nodes[i].id){
                return i;
            }
        }
        return -1;
    }

    int getConstraintTime(int conflictTime,solutionPath* s1,Map* map){
        int time=conflictTime;
        int i=conflictTime;
        size_t i2=conflictTime;
        while(i>=0){
            if(map->getValidMoveSize(s1->nodes[i].index,s1->nodes[i].id)<=1){
                i--;
            }else
                break;
        }

        if(i<0){
            while(i2<s1->nodes.size()) {
                if(map->getValidMoveSize(s1->nodes[i2].index,s1->nodes[i2].id)){
                    i2++;
                }
                else
                    break;
            }
        }



        if(i>=0){
            time=getFirstTime(i,s1->nodes);
           // time=i;
        }else if(i2!=s1->nodes.size())
            time=i2;
        return time;
    }


    int getNextTime(int time, std::vector<solutionNode> nodes){
        for(size_t i=time;i<nodes.size();i++){
            if(nodes[i].id!=nodes[time].id){
                return i;
            }
        }
        return -1;
    }




    void addConstraints(const std::vector<Constraint*> oldConstraintList,Constraint* newConstraint){
        constraint.clear();
        for(size_t i=0;i<oldConstraintList.size();i++){
            constraint.push_back(new Constraint(oldConstraintList[i]->agent,oldConstraintList[i]->node,oldConstraintList[i]->time));
        }
        constraint.push_back(newConstraint);
    }

    std::vector<Constraint*> getConstraint() {
        return constraint;
    }

    void addConstraints(Constraint* newConstraint){constraint.push_back(newConstraint);}

    void setSolution(const std::vector <solutionPath> &newSolution)
    {
        // 释放原有solution的内存
        paths.clear();
        paths.reserve(newSolution.size()); // 预分配空间提升效率

        this->paths=newSolution;
        return;
    }






};


class CBS{
public:
    Map *map;
    Task *task;
    heuristic h_values;
    lowLevel planner;
    std::vector<CTNode> tree;
    std::map<int,int> deadlockmap;
    Solution solution;
    std::chrono::high_resolution_clock::time_point startTime;
    Solution findSolution(Map &map,Task &task);
    bool initRoot(Map &map, Task &task);
    bool conflictCheck(CTNode &ctnode);
    CTNode retrieveAndPopCTNodeWithLowestCost();
    void handleNotEqualTimeStep(CTNode* newCTNode, const Conflict& conflict, int i,
                                     const std::pair<std::pair<int,solutionNode>,std::pair<int,solutionNode>>& timeAndV,
                                     CTNode* p);
    void handleEqualTimes(CTNode* newCTNode, const Conflict& conflict, int i, CTNode* p);
    void handleUnequalTimes(CTNode* newCTNode, const Conflict& conflict, int i, CTNode* p);
    bool UpdateSolutionByLowLevel(CTNode &node, int agentIndex);
    bool checkNewSolution(CTNode& node,int agent0,int agent1);
    int getSIC(std::vector<solutionPath> &solution);
    void insertCTNodeByCost(CTNode& newCTNode);
    Solution setRes(CTNode& ctNode);
    double crossProduct(solutionNode P1,solutionNode P2,solutionNode P3,solutionNode P4);
    bool doIntersect(solutionNode A,solutionNode B,solutionNode C,solutionNode D);
};


#endif //RAILWAYCBS_HIGHLEVEL_H
