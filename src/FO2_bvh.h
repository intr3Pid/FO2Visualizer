#ifndef BVH_PARSER
#define BVH_PARSER
#ifdef __cplusplus

#include "FO2_collision.h"


using namespace std;

#define FO2_BVH_ID 0xDEADC0DE
// max filesize in bytes which is considered to be plausible, 100MiB default
// (these files are usualy about 100Kb in size)
#define FO2_BVH_MAX_FILESIZE 104857600

// here in pathtracer, y and z are again swapped, so its in order x, y, z
#define BVH_GET_AABB(bb, bvh_ptr, f) \
		bb.min.x = -*(float*)bvh_ptr * f; \
		bb.min.y = *((float*)bvh_ptr + 1) * f; \
		bb.min.z = *((float*)bvh_ptr + 2) * f; \
		bb.max.x = -*((float*)bvh_ptr + 3) * f; \
		bb.max.y = *((float*)bvh_ptr + 4) * f; \
		bb.max.z = *((float*)bvh_ptr + 5) * f; 




typedef struct
{
	BBox bb;
	Vector3D center;
	unsigned surfIdx;

} aabb_with_surfIdx;


class cBVH_listEntry {
public:
	Vector3D min, max;
	unsigned int unkn;
	unsigned int index;
	cBVH_listEntry() {
		index = 0; unkn = 0;
	}
};


class cTree_out_el {
public:
	unsigned int counter, res_mask;
	cTree_out_el() { counter = res_mask = 0; }
	~cTree_out_el() {}
};

class cTrav_stack_el {
public:
	float *child_off;
	unsigned int res_mask;
	cTrav_stack_el() {}
	~cTrav_stack_el() {}
};

class cLeaves_list_el {
public:
	float *child_off; 
	unsigned int res_mask;
	unsigned int counter;
	cLeaves_list_el() {}
	~cLeaves_list_el() {}
};

class cLeaves_helper
{
public:
	cLeaves_helper()
	{
		v_off = size = 0;
	}
	~cLeaves_helper(){}
	unsigned v_off, size;
};

class cLeaves_helper_list
{
public:
	cLeaves_helper_list()
	{
		leaves_size = 0;
	}
	//helper for the leaves
	vector<cLeaves_helper> list;
	unsigned leaves_size;
};

class cTrackBVH {
public:
	unsigned int ID; // defaults do 0xDEADC0DE
	unsigned int version; // defaults to 1
	unsigned int surf_count;
	unsigned int surf_count2;
	byte *buffer; 
	unsigned int filesize;
	string pathName;
	vector<cBVH_listEntry> BVHlist;
	vector<cBVH_listEntry> BVHlist2; 

	//helper for the leaves
	cLeaves_helper_list leaves_helper;

	tree<cCDB2_node> AABB_tree;
	tree<cCDB2_node> AABB_tree2;
	bool tree_build;
	tree<cCDB2_node>::iterator first_el_tree;
	tree<cCDB2_node>::iterator first_el_flat;
	vector<Primitive> *primitivesPtr;

	static float some_comp_hard[42];

	vector<cTree_out_el> tree_out;// to save some output from tree parsing func

	
	cTrackBVH(string path = "-1");

	~cTrackBVH()
	{
		delete[] buffer;
	};

	

	int read();
	int build_list_tree(vector<Primitive> *prPtr);
	int read_tree(int omit_flag);
	int write(string filename);

};


#endif // #ifdef __cplusplus
#endif // #ifndef BVH_PARSER
