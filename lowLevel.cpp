#include <cmath>
#include "lowLevel.h"


void lowLevel::clear(){
    open.clear();
    close.clear();
    visited.clear();
    path.cost = -1;
}


Path lowLevel::find_path(Agent agent, Map &map,heuristic &h_values,std::vector<Constraint*> &constraints){

    this->clear();
    this->agent=agent;
    //make_constraints(cons);
    Node start,goal;
    Path path;
    int expanded=0;

    start= getNode(agent.s_index,agent.start_id,agent.start_x,agent.start_y);
    goal= getNode(agent.g_index,agent.goal_id,agent.goal_x,agent.goal_y);
    path = pathPlan(start,goal,map,agent.speed,h_values,constraints);
    expanded=int(close.size());
    if(path.cost<0)
        return Path();
    path.nodes.shrink_to_fit();
    path.cost=path.nodes.back().g;
    path.agentID=agent.id;
    path.expanded=expanded;
    path.agentSpeed=agent.speed;

    return path;

}

Path lowLevel::pathPlan(Node start, Node goal, Map &map, double agentSpeed,heuristic &h_values, std::vector<Constraint*> &constraints,double max_f) {
    open.clear();
    close.clear();
    path.cost=-1;
    visited.clear();
    Path path;
    int pathFound=0;
    start.parent= nullptr;
    open.push_back(start);
    visited.insert({start.id+start.index*map.getMapSize(),{start.g,false}});
    Node curNode;

    while (!open.empty()){
        curNode=find_min();
        auto v = visited.find(curNode.id+(curNode.index+curNode.g)*map.getMapSize());
        if(v->second.second)
            continue;
        v->second.second = true;
        curNode.ConvertNode();

        auto parent=&close.insert({curNode.id+(curNode.index+curNode.g)*map.getMapSize(),curNode}).first->second;

        if(curNode.id==goal.id){
            path.nodes = reconstructPath(curNode);
//            if(path.nodes.back().g < goal.interval.first){
//                curNode.g = goal.interval.first;
//                path.nodes.push_back(curNode);
//            }
            path.cost = curNode.g;
            path.expanded = int(close.size());
            pathFound++;
            return path;
        }

        std::list<Node> succs;
        succs.clear();
        findSuccessors(curNode,map,succs,h_values,goal,agentSpeed,constraints);

        std::list<Node>::iterator it =succs.begin();

        while(it != succs.end()){
            if(it->f>max_f){
                it++;
                continue;
            }
            it->parent=parent;
            add_open(*it);
            it++;
        }

    }
    return path;

}



Node lowLevel::getNode(int index, int node_id, double node_x, double node_y){
    Node node;
    node = Node{index,node_id, 0, 0, node_x, node_y, nullptr};
    return node;
}

Node lowLevel::find_min() {
    Node min = *open.begin();
    open.pop_front();
    return min;
}

void lowLevel::add_open(Node newNode) {
    if (open.empty() || open.back().f - CN_EPSILON < newNode.f)
    {
        open.push_back(newNode);
        return;
    }
    for(auto iter = open.begin(); iter != open.end(); ++iter)
    {
        if(iter->f > newNode.f + CN_EPSILON)
        {
            open.insert(iter, newNode);
            return;
        }
        else if(fabs(iter->f - newNode.f) < CN_EPSILON && newNode.g + CN_EPSILON > iter->g)
        {
            open.insert(iter, newNode);
            return;
        }
    }
    open.push_back(newNode);
    return;
}

std::vector<Node> lowLevel::reconstructPath(Node curNode){
    path.nodes.clear();
    Node* currentNode = &curNode; // 从当前节点开始

    // 沿着父节点指针回溯到起点
    while (currentNode != nullptr) {
        path.nodes.push_back(*currentNode);         // 添加当前节点副本
        currentNode = currentNode->parent;    // 移动到父节点
    }

    // 反转路径使顺序变为从起点到终点
    std::reverse(path.nodes.begin(), path.nodes.end());

    return expandPathByG(path.nodes);

}


std::vector<Node> lowLevel::expandPathByG(const std::vector<Node>& path) {
    std::vector<Node> detailedPath;
    if (path.empty()) return detailedPath;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const Node& current = path[i];
        const Node& next = path[i + 1];

        // 计算相邻节点的g值差（假设差值必须为整数且非负）
        int delta = static_cast<int>(next.g - current.g);
        if (delta < 0) {
            // 处理错误逻辑（例如抛出异常或返回空路径）
            return std::vector<Node>();
        }
        // 生成逐步递增的g值节点
        for (int j = 0; j < delta; ++j) {
            Node newNode = current; // 复制当前节点属性
            newNode.g = current.g + j; // 更新g值
            detailedPath.push_back(newNode);
        }
    }
    // 添加最后一个节点（保持原g值）
    detailedPath.push_back(path.back());
    return detailedPath;
}



double lowLevel::dist(const Node &a, const Node &b) {
    return std::sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}



void lowLevel::findSuccessors(Node curNode, Map map, std::list<Node> &succs, heuristic &h_values,Node goal, double agentSpeed,std::vector<Constraint*> &constraints) {
    Node newNode;
    int index=1;
    std::vector<Node> valid_moves = map.getValidMoves(curNode.index,curNode.id);
    std::vector<Node> append;
    if(curNode.index==0){
        index=1;
    }else if(curNode.index==1){
        index=0;
    }

    for(auto move: valid_moves){
        newNode.index=move.index;
        newNode.id=move.id;
        newNode.x=move.x;
        newNode.y=move.y;
        if(curNode.x==move.x&&curNode.y==move.y){
            newNode.g=curNode.g+1;
        }else{
            double cost = ceil(dist(curNode,newNode)/agentSpeed);
            newNode.g=curNode.g+cost;
        }


        //判断是否符合约束
        //没写完
        if (isConstraint(agent.id,newNode.id,newNode.g,constraints)){
            append.push_back(Node(index,curNode.id,0,curNode.g,curNode.x,curNode.y, nullptr));
            continue;
        }



        //符合约束加入succs中
        visited.insert({newNode.id + (newNode.index+newNode.g) * map.getMapSize(), {newNode.g, false}});
        newNode.f = newNode.g + h_values.get_value(newNode.id,agent.id);
        succs.push_back(newNode);

    }

    if(!append.empty()){
        for(auto move: append){
            newNode.index=move.index;
            newNode.id=move.id;
            newNode.x=move.x;
            newNode.y=move.y;
            if(curNode.x==move.x&&curNode.y==move.y){
                newNode.g=curNode.g+1;
            }else{
                double cost = ceil(dist(curNode,newNode)/agentSpeed);
                newNode.g=curNode.g+cost;
            }


            //判断是否符合约束
            //没写完
            if (isConstraint(agent.id,newNode.id,newNode.g,constraints)){
                continue;
            }



            //符合约束加入succs中
            visited.insert({newNode.id + (newNode.index+newNode.g) * map.getMapSize(), {newNode.g, false}});
            newNode.f = curNode.f + 1;
            succs.push_back(newNode);

        }
    }





}


bool lowLevel::isConstraint(int agentID, int id, int time, const std::vector<Constraint*> &constraints) {
        for(Constraint *c : constraints) {
            if (agentID == c->agent) {
                if (time == c->time && c->node.id==id) {
                    return true;
                }
            }
        }
        return false;
}
