// Intr3Pid

#ifdef WITH_FO2_COLLISION


#ifndef FO2_COLLISION_H
#define FO2_COLLISION_H


#ifdef __cplusplus

#include <vector>
#include <string>
// more cpp stuff ...
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#define NOMINMAX // important ! 
#include <Windows.h>// need winapi

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
#define CDB2_SCALE 0.05
#define CDB2_HEADERSIZE 64
#define CDB2_ID 2557572641
// dividers are used to convert short to float
#define CDB2_CPY_POLY( po, p ) \
        po.v1[0] = (float)(*((short*)p[0])) * div_x; \
        po.v1[1] = (float)(*((short*)p[0] + 2)) * div_y; \
        po.v1[2] = (float)(*((short*)p[0] + 1)) * div_z; \
        po.v2[0] = (float)(*((short*)p[1])) * div_x; \
        po.v2[1] = (float)(*((short*)p[1] + 2)) * div_y; \
        po.v2[2] = (float)(*((short*)p[1] + 1)) * div_z; \
        po.v3[0] = (float)(*((short*)p[2])) * div_x; \
        po.v3[1] = (float)(*((short*)p[2] + 2)) * div_y; \
        po.v3[2] = (float)(*((short*)p[2] + 1)) * div_z; \
// stuff for cdb2
#define CPY_AR( it, tmp_lwh, center) \
            it->tlw[0] = tmp_lwh[0]; \
            it->tlw[1] = tmp_lwh[1]; \
            it->tlw[2] = tmp_lwh[2]; \
            it->tcnr[0] = center[0]; \
            it->tcnr[1] = center[1]; \
            it->tcnr[2] = center[2]; 
            
#define CPY_R_AR( it, tmp_lwh, center) \
            tmp_lwh[0] = it->tlw[0]; \
            tmp_lwh[1] = it->tlw[1]; \
            tmp_lwh[2] = it->tlw[2]; \
            center[0] = it->tcnr[0]; \
            center[1] = it->tcnr[1]; \
            center[2] = it->tcnr[2]; 
            

// bigger offsets possible 
#define MAKE_OFF_EXT( off1, off2, off3, buffPtr, mat1, mat2, mat3 ) \
            *(unsigned int *)buffPtr = off1; \
            *((unsigned int *)buffPtr + 1) = off2; \
            *((unsigned int *)buffPtr + 2) = off3; \
            buffPtr[12] = (mat1 & 63); \
            buffPtr[13] = (mat2 & 63); \
            buffPtr[14] = (mat3 & 3);
            
            
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

#define CALC_AABB( min, max, lwh, center) \
        lwh[0] = ( max.x - min.x ) / 2; \
        lwh[1] = ( max.y - min.y ) / 2; \
        lwh[2] = ( max.z - min.z ) / 2; \
        center[0] = max.x - lwh[0]; \
        center[1] = max.y - lwh[1]; \
        center[2] = max.z - lwh[2];     

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

//-----------------------------------------------------------------------------
// Name   : cPoint3D
// Usage  : 3D point class definition
//-----------------------------------------------------------------------------
class cPoint3D
{
public:
    float x,y,z;

    cPoint3D() { x = y = z = 0.0; };
    ~cPoint3D() {};

}; // End class cPoint3D


// for dbg
//#define TREE_DBG
//#define DEL_E_FLAG 


// one node in AABB tree
class cCDB2_node {
public:
    std::vector<unsigned int> t_f; // all faces of one node as indices of faces in faceLIst
    float tlw[3]; //size of bounding box
    float tcnr[3];
	BBox bb;
    float hiBound;
	float loBound;
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
    //unsigned int num_leav;


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

    
// the cdb2 mesh consists of only one surface which has some polys and 
// every one of these is represented by the following class 
// info: the cdb2 file has also index and vertex tables/streams which can 
// be represented as streams like in geom.w32 with their own classes ( or using 
// the existing ones ..) but they are pretty simple ( only one surface, no objects ..)
// and it doesn't make much sense and therefore they are parsed directly and saved to 
// a list of such objects 
class cCDB2_Poly {
public:
// 3 vertices
// coordinates are normal blender order -> x,y,z
    float v1[3];
    float v2[3];
    float v3[3];
    // 3 materials / flags, max is 63
    byte mat[3];

#ifdef TREE_DBG
// for dbg
    unsigned int leafIDX; // index of leaf the face is belonging to 
    
#endif // #ifdef TREE_DBG

    cCDB2_Poly() {}
    ~cCDB2_Poly() {}
};

//-----------------------------------------------------------------------------
// Name   : cTrackCollision
// Usage  : Track collision (*_cdb2.gen file ) class definition
//-----------------------------------------------------------------------------

class cTrackCollision {
public:
    
    std::string pathName; // full path to cdb2 file
    unsigned int fileSize; // size if file in bytes
    //std::vector<cCDB2_Poly> polyList; // list containing the polys, for reading
    unsigned int polyN; // size of elements in polyList
    bool is_valid; // true if file format is valid cdb2 
    bool use_largeOff; // using large offsets
    unsigned int max_polys_leaf; // maximal poly count in one leaf in AABB_tree
    bool matsAdded;
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
	// the AABB tree must be created every time a track is exported,
    // it is not read from the file
    tree<cCDB2_node> AABB_tree;
	bool tree_build;
    tree<cCDB2_node>::iterator tree_head;
	std::vector<Primitive*> primitivesPtr;

private:
    
    unsigned int tree_size;
    unsigned int totLeaves; // total number of leaves
    unsigned int totFace_leaves; // total count of faces in the leaves, could be more than totFace
    // I just don't want to reinvent the point, do not(!) use the methods from cPoint3D for cdb2
    cPoint3D biggest_vert;
    cPoint3D smalles_vert;
    // for writing the file, maybe we can omit this and read from blender mesh
    unsigned int totVert;
    unsigned int totFace;
    byte *buffer;  // only needed for reading
    
// for dbg
    unsigned int p1_should;
    unsigned int p2_should;

    unsigned int prt1; // size of part1
    unsigned int prt2; // size of part2
    
    
public: 
    
    unsigned int _facOff;
    
    cTrackCollision( std::string path = "");
    ~cTrackCollision() {
        if ( buffer != NULL ) {
            delete[] buffer;
        }
    }


    int read();
    int make_faces( unsigned int leafHI, unsigned int leafLO, unsigned int leaf_cnt);
    int read_header();
    int build_tree();
    int serialize_tree( byte *part1, byte *part2 );
    int read_tree(const std::vector<Primitive *> &_primitives, unsigned int some_flag = 1);
	__int64  cTrackCollision::get_filesize(std::wstring path);
	inline bool cTrackCollision::vert_in_AABB(Vector3D min, Vector3D max, Vector3D vert);

};

#endif //__cplusplus
#endif // #define FO2_COLLISION_H
#endif // #ifdef WITH_FO2_COLLISION