#include "GPSMap.h"
#include <fstream>

namespace GPS_MAP_ROUTER{
using namespace std;

CGPSMap::CGPSMap(void)
{
}

CGPSMap::CGPSMap(string file_name)
{
	if (read_geojson(file_name))
	{
		construct_network_topology();
		//construct_indexing();
	}
}

CGPSMap::~CGPSMap(void)
{
}

//read geojson file and parse the path network
bool 
CGPSMap::read_geojson(string file_name)
{
	ofstream myfile(file_name);
	if (myfile.is_open())
	{
		//use TOL to snap the same node
		//calculate the length of lanes
		myfile.close();
	}
	return true;
}

//
void 
CGPSMap::construct_network_topology()
{
	//construct the topology
	map<ID, CLane>::iterator it = path_map.begin();
	while(it != path_map.end())
	{
		ID start_node_id = it->second.node_ids[0];
		ID end_node_id = it->second.node_ids[it->second.node_ids.size() - 1];
		map<ID, CLane>::iterator it2 = it;
		while(it2 != path_map.end())
		{
			ID start_node_id2 = it2->second.node_ids[0];
			ID end_node_id2 = it2->second.node_ids[it2->second.node_ids.size() - 1];
			if (start_node_id == end_node_id2)
			{
				it->second.previous_path_ids.push_back(it2->first);
				it2->second.next_path_ids.push_back(it->first);
			}
			else if (start_node_id2 == end_node_id)
			{
				it2->second.previous_path_ids.push_back(it->first);
				it->second.next_path_ids.push_back(it2->first);
			}
			it2++;
		}
		it++;
	}
}

// bool 
// CGPSMap::tackle_obstructions(LOCATION obstruction_pos)
// {
// 	//insert an obstruction and break a lane
// 	//whether is it possible to change lanes
// 	//
// }

// void 
// CGPSMap::clean_up_temp_data()
// {
// 
// }

vector<LOCATION> 
CGPSMap::router(LOCATION start_pos, LOCATION end_pos, bool is_a_star)
{
	vector<LOCATION> result;
	shared_ptr<vector<CLane>> ptr_lanes = router_ex(start_pos, end_pos, is_a_star);
	vector<CLane>::iterator it = ptr_lanes->begin();
	for (; it != ptr_lanes->end(); ++it)
	{
		vector<ID>::iterator vect_itr = it->node_ids.begin();
		while(vect_itr != it->node_ids.end())
		{
			result.push_back(node_map[*vect_itr].coords);
			vect_itr++;
		}
	}
	return result;
}

// void
// CGPSMap::add_poses_to_lane_network(LOCATION start_pos, LOCATION end_pos)
// {
// 	//find the nearest lane and create the temp nodes and lanes
// 
// }

shared_ptr<vector<CLane>>
CGPSMap::router_ex(LOCATION start_pos, LOCATION end_pos, bool is_a_star)
{
// 	//find the nearest lane and create the temp nodes and lanes
// 	add_poses_to_lane_network(start_pos, end_pos);
	//routing 
	

}
}