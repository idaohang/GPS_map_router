#pragma once
#include <queue>
#include <vector>
#include <map>
#include "BasicDefs.h"

namespace Voxel_core{
	using namespace std;
	
	typedef pair<unsigned int, float> G_H_cost;
	typedef pair<Indice3i, G_H_cost> Indice_cost_pair;
	typedef vector<Indice_cost_pair> Indice_cost_pair_list;

	struct OrderByCost
	{
		bool operator() (const Indice_cost_pair& a, const Indice_cost_pair& b) const
		{
			return (a.second.first + a.second.second) > (b.second.first + b.second.second);
		}
	};

	template<class _Ty,
	class _Container = vector<_Ty>,
	class _Pr = less<typename _Container::value_type> >
	class my_priority_queue : public std::priority_queue<_Ty,_Container,_Pr>
	{
	public:
		_Container getContainer() {
			return this->c;
		}
	}; 
	typedef my_priority_queue<Indice_cost_pair, vector<Indice_cost_pair>, OrderByCost> Indice_cost_pair_pqueue;

	//
	typedef vector<Indice_cost_pair> Indice_cost_pair_vector;
	typedef map<long, long> Indice_pair_map;
}