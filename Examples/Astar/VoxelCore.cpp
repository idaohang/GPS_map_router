#include "Voxelcore.h"
#include "math.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include "AABBTri.h"

using namespace Voxel_core;

//////////////////////////////////////////////////////////////////////////
//CVoxel
void CVoxel::set_state(char chState)
{
	chState = chState;
}

// void CVoxel::SetCost(int iVal)
// {
// 	iCost = iVal;
// }
// 
// void CVoxel::SetBBox(BoundingBox bbox)
// {
// 	BBox = bbox;
// }

char CVoxel::get_state()
{
	return chState;
}

// int CVoxel::GetCost()
// {
// 	return iCost;
// }
// 
// BoundingBox CVoxel::GetBBox()
// {
// 	return BBox;
// }



//////////////////////////////////////////////////////////////////////////
//CVoxel_core
CVoxel_core::CVoxel_core()
{
	Voxels.clear();
	fVoxelSize = 0.0;
	iExtent = Indice3i(0, 0, 0);
	ptOrigin = Point3d(0.0, 0.0, 0.0);
	lNumberOccupied = 0;
	lNumberOverlap = 0;
}

CVoxel_core::~CVoxel_core()
{
	Voxels.clear();
	fVoxelSize = 0.0;
	iExtent = Indice3i(0, 0, 0);
	ptOrigin = Point3d(0.0, 0.0, 0.0);
	lNumberOccupied = 0;
	lNumberOverlap = 0;
}

//construct voxels based on a set of point3d
CVoxel_core::CVoxel_core(const Pointlist& pts, const float& dVoxel, const bool isRegular)
{
	//calculate the regular bbox of pts
	fVoxelSize = dVoxel;
	BoundingBox ptsbbox(pts);
	//
	ptOrigin = ptsbbox.Min ;
	
	//regularized size
	if (isRegular)
	{
		double dLength = ptsbbox.get_max_size();
		iExtent.x = static_cast<int>(dLength/dVoxel + 1.0);
		iExtent.y = iExtent.x;
		iExtent.z = iExtent.x;
	}
	else
	{
		Vector3d vLength = ptsbbox.get_size();
		iExtent.x = static_cast<int>(vLength.x/dVoxel + 1.0);
		iExtent.y = static_cast<int>(vLength.y/dVoxel + 1.0);
		iExtent.z = static_cast<int>(vLength.z/dVoxel + 1.0);
	}

	//Initialize voxels
	Voxels.reserve(iExtent.z);
	for (int i = 0; i < iExtent.z; i++)
	{
		ListListPtrCVoxel list;
		Voxels.push_back(list);
	}

	ListListListPtrCVoxel::iterator itr_Voxel3 = Voxels.begin();
	for (;itr_Voxel3 != Voxels.end(); itr_Voxel3++)
	{
		(*itr_Voxel3).reserve(iExtent.y);
		for (int i = 0; i < iExtent.y; i++)
		{
			ListPtrCVoxel list;
			(*itr_Voxel3).push_back(list);
		}

		ListListPtrCVoxel::iterator itr_Voxel2 = (*itr_Voxel3).begin();
		for (; itr_Voxel2 != (*itr_Voxel3).end(); itr_Voxel2++)
		{
			(*itr_Voxel2).reserve(iExtent.x);
			for (int i = 0; i < iExtent.x; i++)
			{
				(*itr_Voxel2).push_back(make_shared<CVoxel>());
			}
		}
	}

	lNumberOccupied = 0;
	lNumberOverlap = 0;
}

Indice3i CVoxel_core::regular_pos(const Indice3i& in)
{
	Indice3i out = in;
	if (out.x > iExtent.x - 1)
		out.x = iExtent.x - 1;
	if (out.x < 0)
		out.x = 0;

	if (out.y > iExtent.y - 1)
		out.y = iExtent.y - 1;
	if (out.y < 0)
		out.y = 0;

	if (out.z > iExtent.z - 1)
		out.z = iExtent.z - 1;
	if (out.z < 0)
		out.z = 0;

	return out;
}

Point3d CVoxel_core::get_centercoord(const Indice3i& pos)
{
	return Point3d((pos.x) * fVoxelSize, (pos.y) * fVoxelSize, (pos.z) * fVoxelSize) + ptOrigin;
}

Point3d CVoxel_core::get_origin()
{
	return ptOrigin;
}

PtrCVoxel CVoxel_core::get_voxel(const Indice3i& pos)
{
	if (regular_pos(pos) == pos)
	{
		return Voxels[pos.z][pos.y][pos.x];
	}
	else
	{
		return nullptr;
	}
}

//Query operations
Indice3i CVoxel_core::query_by_point_ex(const Point3d& pt)
{
	Point3d curpt = pt;
	Vector3d dSize = (curpt - ptOrigin) / fVoxelSize;
	Indice3i iSize(static_cast<int>(dSize.x), static_cast<int>(dSize.y), static_cast<int>(dSize.z));
    return iSize;
}

PtrCVoxel CVoxel_core::query_by_point(const Point3d& pt)
{
	Indice3i ipos = query_by_point_ex(pt);
	if (regular_pos(ipos) == ipos)
	{
		return get_voxel(ipos);
	}
	else
		return nullptr;
}

shared_ptr<IndicesList> CVoxel_core::query_by_points(const Pointlist& pts)
{
	shared_ptr<IndicesList> result = make_shared<IndicesList>();
	Pointlist::const_iterator itr_pt = pts.begin();
	for (; itr_pt != pts.end(); itr_pt++)
	{
		Indice3i iIndex = query_by_point_ex(*itr_pt);
		if (regular_pos(iIndex) == iIndex)
		{
			if (find(result->begin(), result->end(), iIndex) == result->end())
			{
				result->push_back(iIndex);
			}
		}
	}
	return result;
}

shared_ptr<IndicesList> CVoxel_core::query_by_bbox(const BoundingBox& bbox)
{
	 Indice3i iMax = regular_pos(query_by_point_ex(bbox.Max));
	 Indice3i iMin = regular_pos(query_by_point_ex(bbox.Min));
	 shared_ptr<IndicesList> result = make_shared<IndicesList>();
	 result->reserve(((iMax - iMin).x + 1) * ((iMax - iMin).y + 1)* ((iMax - iMin).z + 1));
	 int iCount = 0;
	 for (int i = iMin.x; i <= iMax.x; i++)
	 {
		 for (int j = iMin.y; j <= iMax.y; j++)
		 {
			 for (int k = iMin.z; k <= iMax.z; k++)
			 {
				 (*result).push_back(Indice3i(i, j, k));
			 }
		 }
	 }
	 
	 return result;
}

shared_ptr<IndicesList> CVoxel_core::query_by_shell(const Polyhedron3& poly, const Pointlist& pts)
{
	shared_ptr<IndicesList> result = make_shared<IndicesList>();

	return result;
}

shared_ptr<IndicesList> CVoxel_core::query_by_surface(const Surface3& surface, const Pointlist& pts)
{
	shared_ptr<IndicesList> result = make_shared<IndicesList>();

	Trianglelist::const_iterator itr_Tri = surface.facets.begin();
	for (; itr_Tri != surface.facets.end(); itr_Tri++)
	{
		//
		BoundingBox bbox_tri;
		bbox_tri.Max.x = max(max(pts[(*itr_Tri).vert[0]].x, pts[(*itr_Tri).vert[1]].x), pts[(*itr_Tri).vert[2]].x);
		bbox_tri.Max.y = max(max(pts[(*itr_Tri).vert[0]].y, pts[(*itr_Tri).vert[1]].y), pts[(*itr_Tri).vert[2]].y);
		bbox_tri.Max.z = max(max(pts[(*itr_Tri).vert[0]].z, pts[(*itr_Tri).vert[1]].z), pts[(*itr_Tri).vert[2]].z);
		bbox_tri.Min.x = min(min(pts[(*itr_Tri).vert[0]].x, pts[(*itr_Tri).vert[1]].x), pts[(*itr_Tri).vert[2]].x);
		bbox_tri.Min.y = min(min(pts[(*itr_Tri).vert[0]].y, pts[(*itr_Tri).vert[1]].y), pts[(*itr_Tri).vert[2]].y);
		bbox_tri.Min.z = min(min(pts[(*itr_Tri).vert[0]].z, pts[(*itr_Tri).vert[1]].z), pts[(*itr_Tri).vert[2]].z);

		shared_ptr<IndicesList> rough_result = query_by_bbox(bbox_tri);
		//
		IndicesList::iterator itr_pos = rough_result->begin();
		for (; itr_pos != rough_result->end(); itr_pos++)
		{
			Point3d cenPt = get_centercoord((*itr_pos));
			float cent[3];
			float boxhalfsize[3];
			float triverts[3][3];
			cent[0] = static_cast<float>(cenPt.x);
			cent[1] = static_cast<float>(cenPt.y);
			cent[2] = static_cast<float>(cenPt.z);
			boxhalfsize[0] = static_cast<float>(fVoxelSize / 2.0 + 1e-6);
			boxhalfsize[1] = static_cast<float>(fVoxelSize / 2.0 + 1e-6);
			boxhalfsize[2] = static_cast<float>(fVoxelSize / 2.0 + 1e-6);
			triverts[0][0] = static_cast<float>(pts[(*itr_Tri).vert[0]].x);
			triverts[0][1] = static_cast<float>(pts[(*itr_Tri).vert[0]].y);
			triverts[0][2] = static_cast<float>(pts[(*itr_Tri).vert[0]].z);
			triverts[1][0] = static_cast<float>(pts[(*itr_Tri).vert[1]].x);
			triverts[1][1] = static_cast<float>(pts[(*itr_Tri).vert[1]].y);
			triverts[1][2] = static_cast<float>(pts[(*itr_Tri).vert[1]].z);
			triverts[2][0] = static_cast<float>(pts[(*itr_Tri).vert[2]].x);
			triverts[2][1] = static_cast<float>(pts[(*itr_Tri).vert[2]].y);
			triverts[2][2] = static_cast<float>(pts[(*itr_Tri).vert[2]].z);

			if(triBoxOverlap(cent, boxhalfsize, triverts))
			{
				if (find(result->begin(), result->end(), *itr_pos) == result->end())
				{
					result->push_back(*itr_pos);
				}
			}
		}
	}

	return result;
}

//
Indice3i CVoxel_core::find_neighbor_ex(const Indice3i& in_pos, const Indice3i& direction, int iStep = 1)
{
	Indice3i cp_in_pos = in_pos;
	Indice3i cp_direction = direction;
	//direction
	Indice3i new_pos = cp_in_pos + cp_direction * iStep;
	if (regular_pos(new_pos) == new_pos)
	{
		return new_pos;
	}
	else
		return OUTOFRANGE;
}

//Output voxels
bool CVoxel_core::write_voxels(string strPath)
{
	string fileName = strPath.substr(strPath.rfind("\\") + 1);
	ofstream fileMeta,  fileRaw;
	//
	fileMeta.open((strPath + string(".mhd")).c_str(), ios::out);
	fileMeta << "NDims = 3\n";
	fileMeta << "Offset= "<< get_origin().x << " " << get_origin().y << " " << get_origin().z <<endl;
	fileMeta << "DimSize = " + to_string(static_cast<long long>(iExtent.x)) + " " + to_string(static_cast<long long>(iExtent.y)) + " " + to_string(static_cast<long long>(iExtent.z)) + "\n";
	fileMeta << "ElementSize = " + to_string(static_cast<long double>(fVoxelSize)) + " " + to_string(static_cast<long double>(fVoxelSize)) + " " + to_string(static_cast<long double>(fVoxelSize)) + "\n";
	fileMeta << "ElementSpacing = " + to_string(static_cast<long double>(fVoxelSize)) + " " + to_string(static_cast<long double>(fVoxelSize)) + " " + to_string(static_cast<long double>(fVoxelSize)) + "\n";
	fileMeta << "ElementType = MET_UCHAR\n";
	fileMeta << "ElementByteOrderMSB = False\n";
	fileMeta << "ElementDataFile = " + fileName + string(".raw") + "\n";
	fileMeta.close();
	//
	fileRaw.open((strPath + string(".raw")).c_str(), ios::out | ios::binary);
	ListListListPtrCVoxel::iterator itr_Voxel3 = Voxels.begin();
	for (;itr_Voxel3 != Voxels.end(); itr_Voxel3++)
	{
		ListListPtrCVoxel::iterator itr_Voxel2 = (*itr_Voxel3).begin();
		for (;itr_Voxel2 != (*itr_Voxel3).end(); itr_Voxel2++)
		{
			ListPtrCVoxel::iterator itr_Voxel1 = (*itr_Voxel2).begin();
			for (;itr_Voxel1 != (*itr_Voxel2).end(); itr_Voxel1++)
			{
				char chout = (*itr_Voxel1)->get_state();
				fileRaw.write((char*) &chout, sizeof(char));
			}
		}
	}
	fileRaw.close();

	return true;
}

bool CVoxel_core::write_route(string strPath, shared_ptr<IndicesList> path)
{
	string fileName = strPath.substr(strPath.rfind("\\") + 1);
	ofstream fileVtk;
	//
	fileVtk.open((strPath + string(".vtk")).c_str(), ios::out);
	if (!fileVtk.is_open())
	{
		return false;
	}
	fileVtk << "# vtk DataFile Version 1.0\n";
	fileVtk << "Line representation of vtk\n";
	fileVtk << "ASCII\n"<<endl;
	fileVtk << "DATASET POLYDATA\n";
	fileVtk << "POINTS " + to_string(static_cast<long long>(path->size())) + " float\n";
	IndicesList::iterator itr_index = path->begin();
	for (; itr_index != path->end(); ++itr_index)
	{
		Point3d pt = get_centercoord(*itr_index);
		fileVtk << pt.x << " " << pt.y << " " << pt.z <<endl;
	}
	fileVtk<< endl << "VERTICES 2 4" << endl;
	fileVtk<< "1 0" << endl;
	fileVtk<< "1 " <<to_string(static_cast<long long>(path->size()) - 1)<< endl;

	fileVtk <<endl<< "LINES 1 " + to_string(static_cast<long long>(path->size()) + 1)<<endl;
	fileVtk << to_string(static_cast<long long>(path->size())) << " ";
	for (int i = 0; i < path->size(); ++i)
	{
		fileVtk << i << " ";
	}
	fileVtk <<endl;
	fileVtk.close();
	return true;
}
//////////////////////////////////////////////////////////////////////////
bool CVoxel_core::weight_voxel_base_on_floor_and_Ceiling(int step)
{
	for (int z = 0 ; z < iExtent.z; z++)
	{
		for (int y = 0 ; y < iExtent.y; y++)
		{
			for (int x = 0 ; x < iExtent.x; x++)
			{
				Indice3i cur_pos(x, y, z);
				//
				if (get_voxel(cur_pos)->get_state() == Unknow)
				{
					//search for the occupied voxel in negative Z direction
					Indice3i iVoxelL = search_least_voxel(cur_pos, NEGATIVE_Z, step);
					//tag voxels
					if (iVoxelL != OUTOFRANGE)
					{
						//search for the occupied voxel in positive Z direction
						Indice3i iVoxelH = search_least_voxel(iVoxelL, POSITIVE_Z, step);
						if (iVoxelH != OUTOFRANGE)
						{
							search_set_weight_voxel_state(iVoxelL, iVoxelH, POSITIVE_Z, WheelChair, 1);
							//count
							lNumberOverlap += (iVoxelH - iVoxelL).z + 1;
						}
						else
						{
							Indice3i iEnd(iVoxelL.x, iVoxelL.y, iExtent.z - 1);
							search_set_weight_voxel_state(iVoxelL, iEnd, POSITIVE_Z, WheelChair, 1);
						}
					}
					else
					{
						//this vertical space can only be used for flying
						search_set_const_voxel_state(cur_pos, NEGATIVE_Z, Fly);
					}
				}
				else
				{
					//count
					lNumberOverlap++;
				}
			}
		}
	}
	return true;
}

//set the intersecting voxels as occupied
void CVoxel_core::tag_voxel_base_on_geometry(const Surface3& tris, const Pointlist& pts)
{
	if (tris.facets.size() > 0)
	{
		//by input triangles
		shared_ptr<IndicesList> voxelIndices = query_by_surface(tris, pts);
		
		IndicesList::iterator itr_voxelIndex = voxelIndices->begin();
		for (; itr_voxelIndex != voxelIndices->end(); itr_voxelIndex ++)
		{
			get_voxel(*itr_voxelIndex)->set_state(Occupied);
		}
		//count
		lNumberOccupied = voxelIndices->size();
	}
	else if(pts.size() > 0)
	{
		//by input points
		shared_ptr<IndicesList> voxelIndices = query_by_points(pts);
		IndicesList::iterator itr_voxelIndex = voxelIndices->begin();
		for (; itr_voxelIndex != voxelIndices->end(); itr_voxelIndex ++)
		{
			get_voxel(*itr_voxelIndex)->set_state(Occupied);
		}
		//count
		lNumberOccupied = voxelIndices->size();
	}
}


Indice3i CVoxel_core::search_least_voxel(const Indice3i& in_pos, const Indice3i& iDirection, int step)
{
	Indice3i cur_pos = in_pos;
	Indice3i pre_pos;
	while(get_voxel(cur_pos)->get_state() != Occupied)
	{
		pre_pos = cur_pos;
		cur_pos = find_neighbor_ex(cur_pos, iDirection, step);
		if (cur_pos == OUTOFRANGE)
		{
			return OUTOFRANGE;
		}
	}
	return pre_pos;
}

void CVoxel_core::search_set_const_voxel_state(const Indice3i& in_pos, const Indice3i& iDirection, char chState)
{
	Indice3i cur_pos = in_pos;
	while(get_voxel(cur_pos)->get_state() != Occupied)
	{
		get_voxel(cur_pos)->set_state(chState);
		cur_pos = find_neighbor_ex(cur_pos, iDirection, 1);
		if (cur_pos == OUTOFRANGE)
		{
			break;
		}
	}
}

void CVoxel_core::search_set_weight_voxel_state(const Indice3i& st_pos, const Indice3i& ed_pos, const Indice3i& iDirection, char st_State, int iStateStep)
{
	if (st_pos.x != ed_pos.x || st_pos.y != ed_pos.y)
	{
		return;
	}
	//
	int iCount = static_cast<int>(st_State);
	Indice3i cur_pos = st_pos;
	while(cur_pos != ed_pos)
	{
		char chstate = iCount;
		if (iCount < Fly)
		{
			get_voxel(cur_pos)->set_state(chstate);
		}
		else
		{
			get_voxel(cur_pos)->set_state(Fly);
		}
		cur_pos = find_neighbor_ex(cur_pos, iDirection, 1);
		iCount += iStateStep;
	}
	if (iCount < Fly)
	{
		get_voxel(ed_pos)->set_state((char)iCount);
	}
	else
	{
		get_voxel(ed_pos)->set_state(Fly);
	}
}

//http://en.wikipedia.org/wiki/A*_search_algorithm
//http://blog.sciencenet.cn/blog-5422-538894.html
shared_ptr<IndicesList> CVoxel_core::query_path_using_Astar(const Indice3i& st_ivoxel, const Indice3i& ed_ivoxel, int iLocomoion)
{
	if (regular_pos(st_ivoxel) != st_ivoxel || regular_pos(ed_ivoxel) != ed_ivoxel )
	{
		return nullptr;
	}

	if (get_voxel(st_ivoxel)->get_state() == Occupied || get_voxel(st_ivoxel)->get_state() == Useless
		||get_voxel(ed_ivoxel)->get_state() == Occupied || get_voxel(ed_ivoxel)->get_state() == Useless
		||iLocomoion > Fly || iLocomoion < WheelChair)
	{
		return nullptr;
	}
	//
	if (get_voxel(st_ivoxel)->get_state() > iLocomoion || get_voxel(ed_ivoxel)->get_state() > iLocomoion)
	{
		return nullptr;
	}
	//
	Indice_cost_pair_pqueue OpenTable;
	Indice_cost_pair_vector CloseTable;
	Indice_pair_map PathTracer;
	
	G_H_cost score = G_H_cost(0, heuristic_H_cost_estimate(st_ivoxel, ed_ivoxel));
	OpenTable.push(Indice_cost_pair(st_ivoxel, score));
	while (OpenTable.size() != 0 )
	{
		Indice_cost_pair cur_pos = OpenTable.top();
		OpenTable.pop();
		//
		if (cur_pos.first == ed_ivoxel)
		{
			//construct path
			shared_ptr<IndicesList> result = make_shared<IndicesList>();
			construct_path(PathTracer, indice3_to_index(cur_pos.first), result);
			return result;
		}
		//find neighbors
		shared_ptr<Indice_cost_pair_list> neighbours = find_neighbor_Cost(cur_pos, ed_ivoxel, iLocomoion);
		Indice_cost_pair_list::iterator itr_pair = neighbours->begin();
		for (; itr_pair != neighbours->end(); itr_pair++)
		{
			//
			Indice_cost_pair FoundPair;
			if (update_closeset_by_less_cost(*itr_pair, CloseTable, FoundPair))
			{
				if (FoundPair.first != OUTOFRANGE)
				{
					OpenTable.push(FoundPair);
				}
			}
			else if(!update_openset_by_less_cost(*itr_pair, OpenTable))
			{
				//add <child parent> pair to the tracer
				PathTracer[indice3_to_index((*itr_pair).first)] = indice3_to_index(cur_pos.first);
			}
		}
		CloseTable.push_back(cur_pos);
	}

	return nullptr;
}

float CVoxel_core::heuristic_H_cost_estimate(const Indice3i& st_ivoxel, const Indice3i& ed_ivoxel)
{
	//g_score part should be adjusted
	Point3d stPt = get_centercoord(st_ivoxel);
	Point3d edPt = get_centercoord(ed_ivoxel);
	return  (sqrt((edPt - stPt).get_squared_length())/(fVoxelSize));
}

int CVoxel_core::heuristic_G_cost_estimate(const Indice3i& st_ivoxel, const Indice3i& ed_ivoxel, int iStep = 1)
{
	int idiff = get_voxel(st_ivoxel)->get_state() - get_voxel(ed_ivoxel)->get_state();
	if (idiff < 0)
	{
		//go up adding bonus!
		idiff = abs(idiff);
	}
	else
	{
		//go down
		idiff = 0;
	}
	return idiff + iStep;
}

shared_ptr<Indice_cost_pair_list> CVoxel_core::find_neighbor_Cost(const Indice_cost_pair& cur_pos, const Indice3i& ed_ivoxel, int iLocomoion)
{
	shared_ptr<Indice_cost_pair_list> result = make_shared<Indice_cost_pair_list>();
	//+x
	Indice3i neighbour = find_neighbor_ex(cur_pos.first, POSITIVE_X);
	if (neighbour != OUTOFRANGE && get_voxel(neighbour)->get_state() <= iLocomoion && get_voxel(neighbour)->get_state() > Useless)
	{
		result->push_back(Indice_cost_pair(neighbour, G_H_cost(cur_pos.second.first +
			heuristic_G_cost_estimate(cur_pos.first, neighbour),  heuristic_H_cost_estimate(neighbour, ed_ivoxel))));
	}
	//-x
	neighbour = find_neighbor_ex(cur_pos.first, NEGATIVE_X);
	if (neighbour != OUTOFRANGE && get_voxel(neighbour)->get_state() <= iLocomoion && get_voxel(neighbour)->get_state() > Useless)
	{
		result->push_back(Indice_cost_pair(neighbour, G_H_cost(cur_pos.second.first +
			heuristic_G_cost_estimate(cur_pos.first, neighbour),  heuristic_H_cost_estimate(neighbour, ed_ivoxel))));
	}
	//+y
	neighbour = find_neighbor_ex(cur_pos.first, POSITIVE_Y);
	if (neighbour != OUTOFRANGE && get_voxel(neighbour)->get_state() <= iLocomoion && get_voxel(neighbour)->get_state() > Useless)
	{
		result->push_back(Indice_cost_pair(neighbour, G_H_cost(cur_pos.second.first +
			heuristic_G_cost_estimate(cur_pos.first, neighbour),  heuristic_H_cost_estimate(neighbour, ed_ivoxel))));
	}
	//-y
	neighbour = find_neighbor_ex(cur_pos.first, NEGATIVE_Y);
	if (neighbour != OUTOFRANGE && get_voxel(neighbour)->get_state() <= iLocomoion && get_voxel(neighbour)->get_state() > Useless)
	{
		result->push_back(Indice_cost_pair(neighbour, G_H_cost(cur_pos.second.first +
			heuristic_G_cost_estimate(cur_pos.first, neighbour),  heuristic_H_cost_estimate(neighbour, ed_ivoxel))));
	}
	//+z
	neighbour = find_neighbor_ex(cur_pos.first, POSITIVE_Z);
	if (neighbour != OUTOFRANGE && get_voxel(neighbour)->get_state() <= iLocomoion && get_voxel(neighbour)->get_state() > Useless)
	{
		result->push_back(Indice_cost_pair(neighbour, G_H_cost(cur_pos.second.first +
			heuristic_G_cost_estimate(cur_pos.first, neighbour),  heuristic_H_cost_estimate(neighbour, ed_ivoxel))));
	}
	//-z
	neighbour = find_neighbor_ex(cur_pos.first, NEGATIVE_Z);
	if (neighbour != OUTOFRANGE && get_voxel(neighbour)->get_state() <= iLocomoion && get_voxel(neighbour)->get_state() > Useless)
	{
		result->push_back(Indice_cost_pair(neighbour, G_H_cost(cur_pos.second.first +
			heuristic_G_cost_estimate(cur_pos.first, neighbour),  heuristic_H_cost_estimate(neighbour, ed_ivoxel))));
	}
	
	return result;
}

//for open table
bool CVoxel_core::update_openset_by_less_cost(const Indice_cost_pair& in_pair, Indice_cost_pair_pqueue& queue)
{
	Indice_cost_pair_list container = queue.getContainer();
	Indice_cost_pair_list::iterator itr_pair = container.begin();
	bool isIn = 0;
	for (; itr_pair != container.end(); itr_pair++)
	{
		if ((*itr_pair).first == in_pair.first)
		{
			isIn = 1;
			//replace with the smaller cost of G && locally resort the queue
			if ((*itr_pair).second.first > in_pair.second.first)
			{
				container.erase(itr_pair);
				queue.push(in_pair);
				
				//for the tracer
				return false;
			}
			break;
		}
	}
	//insert the pair for open table
	if(isIn == 0)
		queue.push(in_pair);

	return isIn;
}

//for close table
bool CVoxel_core::update_closeset_by_less_cost(const Indice_cost_pair& in_pair, Indice_cost_pair_vector& container, Indice_cost_pair& out_pair)
{
	Indice_cost_pair_list::iterator itr_pair = container.begin();
	for (; itr_pair != container.end(); itr_pair++)
	{
		if ((*itr_pair).first == in_pair.first)
		{
			//eraser the smaller cost
			if (((*itr_pair).second.first) > (in_pair.second.first))
			{
				out_pair = (*itr_pair);
				(*itr_pair).second = in_pair.second;
			}
			else
			{
				out_pair = Indice_cost_pair(OUTOFRANGE, G_H_cost(0, 0));
			}
			return true;
		}
	}
	return false;
}

void CVoxel_core::construct_path(Indice_pair_map& tracer, long index, shared_ptr<IndicesList> result)
{
	if (1 == tracer.count(index))
	{
		result->push_back(index_to_indice3(index));
		construct_path(tracer, tracer[index], result);
	}
	else if (1 < tracer.count(index))
	{
		return;
	}
	else
	{
		result->push_back(index_to_indice3(index));
	}
};

long CVoxel_core::indice3_to_index(const Indice3i& node)
{
	return node.z * iExtent.x * iExtent.y + node.y * iExtent.x + node.x;
}

Indice3i CVoxel_core::index_to_indice3(const long& index)
{
	Indice3i result;
	result.z = index / (iExtent.x * iExtent.y);
	long temp = index % (iExtent.x * iExtent.y);
	result.y = temp / iExtent.x;
	result.x = temp % iExtent.x;
	return result;
}

//return the number of all the occupied voxels
long CVoxel_core::number_of_occupied_voxels()
{
	return lNumberOccupied;
}

//return the number of all the voxels that are covered by the shape
long CVoxel_core::number_of_overlapping_voxels()
{
	return lNumberOverlap;
}