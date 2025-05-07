#ifndef RAILWAYCBS_DATASTRUCT_H
#define RAILWAYCBS_DATASTRUCT_H
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <vector>
#include <chrono>

using boost::multi_index_container;
using namespace boost::multi_index;

//探索的具体节点，通过地图索引节点创建
struct Node{
    int index;
    int id;
    double f,g,x,y;
    Node* parent;
    Node(int index=-1,int id=-1, double f=-1, double g=-1, double x=-1, double y=-1, Node* parent=nullptr)
            : index(index),id(id), f(f), g(g), x(x), y(y), parent(parent){};

    void ConvertNode(){if(this->index==0) this->index=1; else if(this->index==1) this->index=0;}
};


struct subVertex{
    //双节点的子节点，地图索引节点
    int index;   //区分左右子节点   0右 1左
    double x;
    double y;
    std::vector<int> neighbors;
    subVertex(int index=-1,double x=-1, double y=-1):index(index),x(x), y(y) {}
    ~subVertex()  {neighbors.clear();}
};



struct Vertex{    //地图索引节点
    std::vector<subVertex> subVertices;
    Vertex(){}
    Vertex(std::vector<subVertex> subVertices): subVertices(subVertices) {}
    ~Vertex() {subVertices.clear();}
};

struct Agent {
    double start_x,start_y,goal_x,goal_y;
    int start_id,goal_id,s_index,g_index;
    int id;
    double speed;
    Agent(int id=-1,int speed=-1,double length=5,int startID=-1,int goalID=-1,int sindex=-1,int gindex=-1):
            start_id(startID),goal_id(goalID),s_index(sindex),g_index(gindex),id(id), speed(speed){}
};

struct Path{
    std::vector<Node> nodes;
    double cost;
    int agentID;
    int expanded;
    double agentSpeed;
    Path(std::vector<Node> nodes= std::vector<Node>(0), double cost=-1,int agentID=-1)
            : nodes(nodes), cost(cost), agentID(agentID) {expanded=0;agentSpeed=-1;}
};

struct solutionNode
{
    int id;
    double g;
    int index;
    solutionNode(int id_=-1, double g_=-1,int index=-1):id(id_),g(g_),index(index){}
    solutionNode(const Node &n)
    {
        index=n.index;
        id = n.id;
        g = n.g;
    }

    inline bool operator ==( solutionNode &v)
    {
        return v.id==this->id&&v.id==this->id;
    }

};

struct solutionPath{

    std::vector<solutionNode> nodes;
    double cost;
    double agentSpeed;
    int agentID;
    int expanded;
    solutionPath(std::vector<solutionNode> _nodes = std::vector<solutionNode>(0), double _cost = -1, int _agentID = -1)
            : nodes(_nodes), cost(_cost), agentID(_agentID) {expanded = 0;agentSpeed=-1;}
    solutionPath operator= (const Path &path)
    {
        cost = path.cost;
        agentID = path.agentID;
        expanded = path.expanded;
        agentSpeed=path.agentSpeed;
        nodes.clear();
        for(auto n:path.nodes)
            nodes.push_back(solutionNode(n));
        return *this;
    }




};


struct Constraint{
    int agent;
    solutionNode node;
    double time;
    Constraint(int agentID, solutionNode node, int time) :
            agent(agentID), node(node), time(time) { }

};


struct Conflict {
    std::pair<int, int> conflictedAgentsID;
    solutionNode node1;
    solutionNode node2;
    int time1;
    int time2;
    int timeStep;

    Conflict(int firstAgentID, int secondAgentID, solutionNode node1, solutionNode node2, int time1,int time2,int timeStep) :
            node1(node1), node2(node2), time1(time1),time2(time2),timeStep(timeStep) {
        conflictedAgentsID = std::make_pair(firstAgentID, secondAgentID);
    }

};


struct Solution{
    bool found;
    double flowtime;
    double makespan;
    double initCost;
    int high_level_expanded;
    int low_level_expanded;
    std::chrono::duration<double> initTime;
    std::vector<solutionPath> paths;
    std::chrono::duration<double> time;
    Solution(double _flowtime = -1, double _makespan = -1, std::vector<solutionPath> _paths = {})
            : flowtime(_flowtime), makespan(_makespan), paths(_paths) { initCost = -1; low_level_expanded = 0;}
    ~Solution() { paths.clear(); found = false;}

};

#endif //RAILWAYCBS_DATASTRUCT_H
