#pragma once
// #include <queue>
#include <vector>
#include <map>

namespace GPS_MAP_ROUTER{

typedef unsigned long ID;
typedef double LOCATION[3];//location in wgs84
#define TOL 1e-5 //1e-5 degree is approx. 0.02 meter 
//enum PATH_PRIORITY {LOW = 0, HIGH, HIGHEST};

// Priority queue
// typedef std::pair<unsigned int, float> G_H_cost;
// typedef std::pair<ID, G_H_cost> Indice_cost_pair;
// typedef std::vector<Indice_cost_pair> Indice_cost_pair_list;
// 
// struct OrderByCost
// {
// 	bool operator() (const Indice_cost_pair& a, const Indice_cost_pair& b) const
// 	{
// 		return (a.second.first + a.second.second) > (b.second.first + b.second.second);
// 	}
// };
// 
// template<class _Ty,
// class _Container = vector<_Ty>,
// class _Pr = less<typename _Container::value_type> >
// class my_priority_queue : public std::priority_queue<_Ty,_Container,_Pr>
// {
// public:
// 	_Container getContainer() {
// 		return this->c;
// 	}
// }; 
// typedef my_priority_queue<Indice_cost_pair, std::vector<Indice_cost_pair>, OrderByCost> Indice_cost_pair_pqueue;
// 
// 
// typedef std::vector<Indice_cost_pair> Indice_cost_pair_vector;
// typedef std::map<long, long> Indice_pair_map;
}