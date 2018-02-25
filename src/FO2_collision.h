#ifndef FO2_COLLISION_H
#define FO2_COLLISION_H
#ifdef __cplusplus

#include "FO2_geometry_scale.h"


#include <vector>
#include <string>
#include <fstream>

// for get_filesize method
#include <fcntl.h>
#ifdef _MSC_VER
#include <corecrt_io.h>
#else
#include <iostream> // ? 
#endif

#define NOMINMAX 
// need winapi
#include <Windows.h>

// nice stl-like tree library
#include "tree.hpp"
#include "bbox.h"
#include "static_scene/triangle.h"

using namespace std;
using namespace CMU462;
using namespace StaticScene;

/* the render renders a much smaller area, so the map is scaled,
* the same must be done for the tree
*/

#define CDB2_HEADERSIZE 64
#define CDB2_ID 2557572641
// max plausible filesize, these files are usualy under 1Mib
#define CDB2_MAX_FILESIZE 104857600 

#define MIN_MAX_VERTS(cnt, vert, min, max) \
        if( !cnt ) { \
            max.x = vert.x; \
            max.y = vert.y; \
            max.z = vert.z; \
            min.x = vert.x; \
            min.y = vert.y; \
            min.z = vert.z; \
        } \
        else { \
            if( max.x < vert.x) \
                max.x = vert.x; \
            else if( min.x > vert.x) \
                min.x = vert.x; \
            if( max.y < vert.y ) \
                max.y = vert.y; \
            else if( min.y > vert.y) \
                min.y = vert.y ; \
            if( max.z < vert.z) \
                max.z = vert.z; \
            else if( min.z > vert.z) \
                min.z = vert.z; \
        } 

#define CALC_MIN_MAX( min, max, lwh, center) \
        max.x = center[0] + lwh[0]; \
        max.y = center[1] + lwh[1]; \
        max.z = center[2] + lwh[2]; \
        min.x = max.x - lwh[0] * 2; \
        min.y = max.y - lwh[1] * 2; \
        min.z = max.z - lwh[2] * 2; \

#define OBJ_CENTER( min, max, center ) \
        center.x = (max.x + min.x) / 2; \
        center.y = (max.y + min.y) / 2; \
        center.z = (max.z + min.z) / 2; 


#define CPY_AABB(a, b, s) \
		a.min.x = -b.min.x * s; \
		a.min.y = b.min.z * s; \
		a.min.z = b.min.y * s; \
		a.max.x = -b.max.x * s; \
		a.max.y = b.max.z * s; \
		a.max.z = b.max.y * s; 


#define VERT_IN_AABB(min, max, vert) \
	vert.x <= max.x && vert.x >= min.x && \
	vert.y <= max.y && vert.y >= min.y && \
	vert.z <= max.z && vert.z >= min.z 


//-----------------------------------------------------------------------------
// Name   : cPoint3D
// Usage  : 3D point class definition
//-----------------------------------------------------------------------------
class cPoint3D
{
public:
    float x,y,z;

    cPoint3D() { x = y = z = 0.0f; };
    ~cPoint3D() {};

}; // End class cPoint3D


// one node in AABB tree
class cCDB2_node {
public:
    vector<unsigned int>		t_f; // all faces of one node as indices of faces in faceLIst
	float						tlw[3]; //size of bounding box
	float						tcnr[3];
	BBox						bb;
	float						hiBound;
	float						loBound;
	// flags for fast select while rendering
	bool l;
	bool r;
	bool isLeaf;
	bool isRoot;
    byte t_flag;
    byte t_axis; // axis
    // set these 2 only to 0 or 1 ! 
    byte flag_r; // road flag ?
    byte flag_e; // no-road flag ?
  

    byte all_flags;
    unsigned int c1_off;


	// for bvh 
	unsigned int ukn_index, nmb_in_list;
	float * c_off;
	unsigned int res_mask;
	unsigned int cnt;

    cCDB2_node() { 
    tlw[0] = 0.0f; tlw[1] = 0.0f; tlw[2] = 0.0f; 
    tcnr[0] = 0.0f; tcnr[1] = 0.0f; tcnr[2] = 0.0f;
    hiBound = 0.0f; loBound = 0.0f; l = false; 
    r = false; isLeaf = false; isRoot = false;
    t_flag = 0; t_axis = 0; flag_r = 0; flag_e = 0;
    all_flags = 0; c1_off = 0;
    }
    ~cCDB2_node(){}
};


class cTrackCollision {
public:
   
   string pathName; // full path to cdb2 file
    
// header vars
    unsigned int plate;// constant
    unsigned int flag; 
    int min_x;
    int min_y;
    int min_z;
    unsigned int max_x;
    unsigned int max_y;
    unsigned int max_z;
    // multiplicative inverse of mults
    float div_x;
    float div_z;
    float div_y;
    float mult_x;
    float mult_z;
    float mult_y;
    unsigned int offset_1;
    unsigned int offset_2;
// end header vars

    tree<cCDB2_node> AABB_tree;
	bool tree_build;
    tree<cCDB2_node>::iterator tree_head;	

private:

	vector<Primitive*> *primitivesPtr;
    unsigned int polyN; // size of elements in polyList
    bool is_valid; // true if file format is valid cdb2 
    bool use_largeOff; // using large offsets
    unsigned int max_polys_leaf; // maximal poly count in one leaf in AABB_tree
    bool matsAdded;
    unsigned int tree_size;
    unsigned int totLeaves; // total number of leaves
    cPoint3D biggest_vert;
    cPoint3D smalles_vert;
    // for writing the file, maybe we can omit this and read from blender mesh
    unsigned int totVert;
    unsigned int totFace;
    byte *buffer;  // only needed for reading
	size_t filesize;

    
public: 
   
    
    cTrackCollision( string path = "");
    ~cTrackCollision() {

            delete[] buffer;
			buffer = nullptr;
    }

    int read_header();
    int read_tree(vector<Primitive*> *_primitives);
	static uint64_t  get_filesize(wstring path);


};

#endif //__cplusplus
#endif // #define FO2_COLLISION_H
