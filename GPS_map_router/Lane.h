#pragma once
#include "defs.h"
#include "Events.h"
namespace GPS_MAP_ROUTER{
using namespace std;
struct ATTR_LANE
{
	//string name;
	float length;
	//unsigned int speed_limit;
	//PATH_PRIORITY priority;
	//float slop;
	//float curvature;
	//bool is_enable_lane_changing;
};

class CLane
{
public:
	ID id;
	vector<ID> node_ids;
	//vector<CEvents> events;
	shared_ptr<ATTR_LANE> attribute;
	//topology
	vector<ID> next_path_ids;
	vector<ID> previous_path_ids;
	//
	//ID left_sibling_path_id;
	//ID right_sibling_path_id;
public:
	CLane(void);
	~CLane(void);
};
}

