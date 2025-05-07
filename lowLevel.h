#ifndef RAILWAYCBS_LOWLEVEL_H
#define RAILWAYCBS_LOWLEVEL_H

#include "dataStruct.h"
#include "map.h"
#include "const.h"
#include "heuristic.h"


#include <unordered_map>
#include <map>
#include <set>
#include <list>
#include <vector>

class lowLevel {
public:
    lowLevel(){}
    ~lowLevel(){}
    Path find_path(Agent agent, Map &map,heuristic &h_values,std::vector<Constraint*> &constraints);  //constraint  std::list<Constraint> cons

private:
    Agent agent;
    std::list<Node> open;
    std::unordered_map<int,Node> close;
    Path path;
    std::unordered_map<int, std::pair<double, bool>> visited;
    void clear();
    Path pathPlan(Node start,Node goal, Map &map, double agentSpeed,heuristic &h_values, std::vector<Constraint*> &constraints,double max_f=CN_INFINITY);
    Node getNode(int index,int node_id, double node_x, double node_y);
    Node find_min();
    void add_open(Node newNode);
    std::vector<Node> reconstructPath(Node curNode);
    double dist(const Node& a, const Node& b);
    void findSuccessors(Node curNode,Map map,std::list<Node>&succs,heuristic &h_values,Node goal,double agentSpeed,std::vector<Constraint*> &constraints);
    bool isConstraint(int agentID, int id, int time, const std::vector<Constraint*> &constraints);
    std::vector<Node> expandPathByG(const std::vector<Node>& path);

};


#endif //RAILWAYCBS_LOWLEVEL_H
