#pragma once

#include "BasicDefs.h"
#include "A_star.h"

namespace Voxel_core{

	using namespace std;

	//ascending
	enum VoxelState {Occupied = 10, Route = 20, STEDPoint = 30, Unknow = -1, Useless, WheelChair, Child, Adult1, Adult2, Fly};
	const Indice3i OUTOFRANGE = Indice3i(-1,-1,-1);
	const Indice3i POSITIVE_Z = Indice3i(0, 0, 1);
	const Indice3i NEGATIVE_Z = Indice3i(0, 0, -1);
	const Indice3i POSITIVE_Y = Indice3i(0, 1, 0);
	const Indice3i NEGATIVE_Y = Indice3i(0, -1, 0);
	const Indice3i POSITIVE_X = Indice3i(1, 0, 0);
	const Indice3i NEGATIVE_X = Indice3i(-1, 0, 0);

	class CVoxel
	{
	private:
		//properties
		char chState;
	public:
		//construction
		CVoxel(){chState = Unknow;}
		CVoxel(char chTag):chState(chTag){}
		virtual ~CVoxel(){};

	public:
		//operations
		void set_state(char);
		//
		char get_state();
	};

	typedef shared_ptr<CVoxel> PtrCVoxel;
	typedef vector<PtrCVoxel> ListPtrCVoxel;
	typedef vector<ListPtrCVoxel> ListListPtrCVoxel;
	typedef vector<ListListPtrCVoxel> ListListListPtrCVoxel;

	class CVoxel_core  
	{
	private:

		//Data
		ListListListPtrCVoxel Voxels;

		//voxel size
		double fVoxelSize;

		//Voxel Extent
		Indice3i iExtent;

		//spatial extent
		Point3d ptOrigin;

		//statistics
		long lNumberOccupied;
		long lNumberOverlap;
	private:
		//manipulate operations
		Indice3i regular_pos(const Indice3i&);
		//
		//the second value should be normalized
		Indice3i find_neighbor_ex(const Indice3i&, const Indice3i&, int);
		//the second value should be normalized
		Indice3i search_least_voxel(const Indice3i& , const Indice3i&, int );
		//the second value should be normalized
		void search_set_const_voxel_state(const Indice3i&, const Indice3i&, char);
		void search_set_weight_voxel_state(const Indice3i&, const Indice3i&, const Indice3i&, char, int);
		//////////////////////////////////////////////////////////////////////////
		//A star path finding
		//g_score part should be adjusted
		float heuristic_H_cost_estimate(const Indice3i& , const Indice3i& );
		int heuristic_G_cost_estimate(const Indice3i& , const Indice3i&, int);
		shared_ptr<Indice_cost_pair_list> find_neighbor_Cost(const Indice_cost_pair& , const Indice3i& , int );
		bool update_openset_by_less_cost(const Indice_cost_pair& , Indice_cost_pair_pqueue&);
		bool update_closeset_by_less_cost(const Indice_cost_pair& , Indice_cost_pair_vector&, Indice_cost_pair& );
		void construct_path(Indice_pair_map& , long , shared_ptr<IndicesList> );
		long indice3_to_index(const Indice3i&);
		Indice3i index_to_indice3(const long& );
	public:
		//////////////////////////////////////////////////////////////////////////
		//generic Operations
		PtrCVoxel get_voxel(const Indice3i&);
		Point3d get_centercoord(const Indice3i&);
		Point3d get_origin();
		//Query operations
		Indice3i query_by_point_ex(const Point3d&);//the result should be checked by regular_pos
		PtrCVoxel query_by_point(const Point3d&);
		shared_ptr<IndicesList> query_by_points(const Pointlist&);
		shared_ptr<IndicesList> query_by_bbox(const BoundingBox&);
		shared_ptr<IndicesList> query_by_shell(const Polyhedron3&, const Pointlist&);
		shared_ptr<IndicesList> query_by_surface(const Surface3&, const Pointlist&);
		shared_ptr<IndicesList> query_path_using_Astar(const Indice3i&, const Indice3i&, int);
		//Output voxels to vtk (*.raw)
		bool write_voxels(string);
		//Ouput path to vtk file
		bool write_route(string, shared_ptr<IndicesList>);
		//////////////////////////////////////////////////////////////////////////
		//Specified operations
		bool weight_voxel_base_on_floor_and_Ceiling(int step);
		void tag_voxel_base_on_geometry(const Surface3&, const Pointlist&);
		//////////////////////////////////////////////////////////////////////////
		//Fractal dimension analysis
		long number_of_occupied_voxels();
		long number_of_overlapping_voxels();
	public:
		//construction
		CVoxel_core();
		CVoxel_core(const Pointlist&, const float&, const bool);
		virtual ~CVoxel_core();
	};
}



















