#pragma once
#include "defs.h"
namespace GPS_MAP_ROUTER{
class CEvents
{
public:
	bool is_stop_line;
	bool is_pedestrial_cross;
	bool is_signal_light;
	bool is_traffic_sign;
	LOCATION presice_location;//the presiece_location of the light or the sign
public:
	CEvents(void);
	~CEvents(void);
};
}
