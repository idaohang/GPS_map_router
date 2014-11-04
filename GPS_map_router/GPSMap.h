#pragma once

#include <vector>
#include <map>
#include <string>
#include "defs.h"
#include "Events.h"
#include "lane.h"
#include "node.h"

namespace GPS_MAP_ROUTER{
using namespace std;
//class constructing path map made from GPS points
//perquisite: the path map is stored in Geojson format and is topologically correct and the orientation is implied from order of nodes
class CGPSMap
{
private:
	//please check
	//Node list
	std::map<ID, CNode> node_map;
	//Path list
	std::map<ID, CLane> path_map;

	//temporal node-lane list
	//vector<ID> temp_node_ids;
	//vector<ID> temp_lane_ids;

public:
	CGPSMap(void);
	~CGPSMap(void);
	CGPSMap(std::string filename);

private:
	bool read_geojson(std::string);
	void construct_network_topology();
	//void construct_indexing();
	//void add_poses_to_lane_network(LOCATION start_pos, LOCATION end_pos);
	//void clean_up_temp_data();
public:
	//dynamic events
	//bool tackle_obstructions(LOCATION);//tackle with obstructions
	//routers
	vector<LOCATION> router(LOCATION start_pos, LOCATION end_pos, bool is_a_star);
	shared_ptr<vector<CLane>> router_ex(LOCATION start_pos, LOCATION end_pos, bool is_a_star);
};

}