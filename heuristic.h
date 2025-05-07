#ifndef RAILWAYCBS_HEURISTIC_H
#define RAILWAYCBS_HEURISTIC_H


#include "dataStruct.h"
#include "map.h"
#include <vector>
#include <unordered_map>

struct by_index_and_g{};


typedef multi_index_container<
        Node,
        indexed_by<
        //Node 类型中的 g 成员变量进行排序，允许具有相同 g 值的元素存在，并且按照升序排序   g为第一索引
        ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node , double, g)>,
        //Node 类型中的 id 成员变量进行哈希，确保每个元素都具有唯一的 id        id为第二索引
        hashed_unique<BOOST_MULTI_INDEX_MEMBER(Node, int, id)>,
        //按index分组，再按g值排序
        ordered_non_unique<tag<by_index_and_g>,composite_key<Node, BOOST_MULTI_INDEX_MEMBER(Node,int,index),
        BOOST_MULTI_INDEX_MEMBER(Node,double ,g)>>
        >
> Open;


class heuristic {

private:
    std::vector<std::vector<double>> h_values;
    Open open;
    Node find_min();
    Node find_min_by_index(int target_index);
    double dist(Node&a,Node&b){return std::sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));}
public:
    heuristic(){}
    void init(int size,int agents);
    void count(Map &map,Agent agent);
    double get_value(int id_node, int id_agent) {return h_values[id_node][id_agent];}

};


#endif //RAILWAYCBS_HEURISTIC_H
