// Intr3Pid


#include "FO2_collision.h"

#ifdef WITH_FO2_COLLISION
// default constructor
// reads whole file in memory if file Path provided
cTrackCollision::cTrackCollision( std::string path) {
    pathName = "";
    fileSize = 0;
    buffer = NULL;
    //primitivesPtr = NULL;
    //part1 = NULL;
    //part2 = NULL;
    prt1 = 0;
    prt2 = 0;
    polyN = 0;
    is_valid = false;
    use_largeOff = false;
	tree_build = false;
    matsAdded = false;
    totFace_leaves = 0;
    totFace_leaves = 0;
    tree_size = 0;
    //
    // initialize header vars
    //
    plate = CDB2_ID; /*const*/ flag = 0; min_x = -32767;
    min_y = -32767; min_z = -32767; max_x = 32767; max_y = 32767;
    max_z = 32767; mult_x = 0.0f; mult_z = 0.0f; mult_y = 0.0f;
    div_x = 0.0f; div_z = 0.0f; div_y = 0.0f; offset_1 = 0;
    offset_2 = 0;
    //
    // initialize other
    //
    max_polys_leaf = 7; // lower crashes game
    totVert = 0;
    totFace = 0;
    
    do {
    if ( path.length()) {
        
        pathName = path;
		std::wstring t_wpath( pathName.begin(), pathName.end() );
		__int64 fSize = 0;
		fSize = get_filesize(t_wpath);
        if( fSize > 0 && fSize < 39212020) { /*random number*/
            fileSize = (unsigned int)fSize;
        } else {
            break;
        }
        //
        // read whole file in memory
        //
        std::ifstream cdb2S( pathName, std::ios::binary );
        if( !cdb2S.is_open() ){
            break;
        }
        buffer = new byte[ fileSize ];
        cdb2S.read ( (char*)buffer, fileSize );
        cdb2S.close();
        //
        // read header
        //
        read_header();
        
    }
    
    } while(0);
    
}
    
#if 0   
// transform the leaves of AABB tree to offsets in index stream, and save faces
// returns: 1 == success; -1 == error
int cTrackCollision::make_faces( unsigned int leafHI, unsigned int leafLO, unsigned int leaf_cnt) {
    int ret = -1;
    unsigned int i = 0;
    cCDB2_Poly poly;
    unsigned int polyNum = 0;
    byte sw_flag = 0;
    byte *Offs[3];
    byte *pIdx = NULL;
    byte *ofs2 = buffer + CDB2_HEADERSIZE + offset_2; // ptr to beg of vertex stream
    byte *pI2 = NULL; // custom ptr to indexstream + custom offset
    unsigned int *pI3 = NULL; // large offsets are a full int, pointer to 

    
    do {
        
    if( buffer == NULL ) {
        break;
    }   
    // ptr to faces in indexStream 
    pIdx  = buffer + CDB2_HEADERSIZE + offset_1 + (leafLO >> 9); 
    // triangle count 
    polyNum = (leafHI & 127);
    i = polyNum;
    polyN += polyNum;
    // switch flag 
    sw_flag = (leafLO >> 6) & 7;
    
    // these subs in the cases basically do the same , pushing offsets to the vertices
    // in the correct order for one face, but one can push higher offsets than other
    // this was done most probably to save space in memory 
    switch ( sw_flag ) {
        case 0: 
            poly.mat[0] = (leafHI >> 7) & 63;
            poly.mat[1] = pIdx[0] & 63;
            poly.mat[2] = pIdx[0] >> 6;
    
            Offs[0] = ofs2 + 2 * (leafHI >> 13);
            Offs[1] = ofs2 + 2 * ( pIdx[1] | (( pIdx[2] | ( (pIdx[3] & 7) << 8)) << 8));
            Offs[2] = ofs2 + 2 * (( pIdx[3] >> 3) | 32 * ( pIdx[4] | ( pIdx[5] << 8)));
            pIdx += 6;
            // assign vertices
            CDB2_CPY_POLY( poly, Offs )
            // push poly back in std::vector
            polyList.push_back( poly );
        
            if ( polyNum > 1 ) {
                i = polyNum - 1; 
                do {
                    poly.mat[0] = pIdx[1] & 63;
                    poly.mat[1] = pIdx[0] & 63;
                    poly.mat[2] = pIdx[0] >> 6;

                    Offs[0] = ofs2 + 2 * 
                    ((pIdx[1] >> 7) | 2 * ( pIdx[2] | (( pIdx[3] | ((pIdx[4] & 3) << 8)) << 8)));
                    Offs[1] = ofs2 + 2 * ((pIdx[4] >> 2) | (( pIdx[5] | (( pIdx[6] & 31) << 8)) << 6));
                    Offs[2] = ofs2 + 2 * ((pIdx[6] >> 5) | 8 * ( pIdx[7] | (pIdx[8] << 8)));
                    CDB2_CPY_POLY( poly, Offs )
                    polyList.push_back( poly );
                    pIdx += 9;
                } while ( --i );
            }
        break;
        case 1: 
            
            // first values
            poly.mat[0] = (leafHI >> 7) & 63;
            poly.mat[1] = pIdx[0] & 63;
            poly.mat[2] = pIdx[0] >> 6;
  
            Offs[0] = ofs2 + 2 * (leafHI >> 13); // max off is 174762
            Offs[1] = ofs2 + 2 * ( pIdx[1] | (( pIdx[2] | (( pIdx[3] & 7) << 8)) << 8)); // max is 2796202
            Offs[2] = ofs2 + 2 * ((pIdx[3] >> 3) | 32 * (pIdx[4] | (pIdx[5] << 8))); // max is 699050
            pIdx += 6;
            CDB2_CPY_POLY( poly, Offs )
            polyList.push_back( poly );
        
            if( polyNum > 1 ) {
                i = polyNum - 1;
                do {

                    poly.mat[0] = (leafHI >> 7) & 63; 
                    poly.mat[1] = pIdx[0] & 63;
                    poly.mat[2] = (pIdx[0] >> 6) & 1;

                    Offs[0] = ofs2 + 2 * 
                        ((pIdx[0] >> 7) | 2 * ( pIdx[1] | (( pIdx[2] | ((pIdx[3] & 3) << 8)) << 8)));// max off is 174762
                    Offs[1] = ofs2 + 2 * ((pIdx[3] >> 2) | (( pIdx[4] | ((pIdx[5] & 31) << 8)) << 6));// max off is 174762
                    Offs[2] = ofs2 + 2 * ((pIdx[5] >> 5) | 8 * ( pIdx[6] | (pIdx[7] << 8)));// max off is 174762
                    pIdx += 8;
                    CDB2_CPY_POLY( poly, Offs )
                    polyList.push_back( poly );
                } while ( --i );
            }
        break;
        case 2: 
            // ptr to index stream + custom offset
            pI2 = ofs2 + 2 * (leafHI >> 13); 
            if( i ) {
                do {    
                    poly.mat[0] = pIdx[1] & 63;
                    poly.mat[1] = pIdx[0] & 63;
                    poly.mat[2] = pIdx[0] >> 6;
                    Offs[0] = pI2 + 2 * pIdx[2]; 
                    Offs[1] = pI2 + 2 * pIdx[3];
                    Offs[2] = pI2 + 2 * pIdx[4];
                    pIdx += 5;
                    CDB2_CPY_POLY( poly, Offs )
                    polyList.push_back( poly );
                } while ( --i );
            }
  
        break;
        case 3: 
            pI2 = ofs2 + 2 * (leafHI >> 13); 
            if ( i ) {
                do {
                    poly.mat[0] = (leafHI >> 7) & 63;
                    poly.mat[1] = pIdx[0] & 63;
                    poly.mat[2] = pIdx[0] >> 6;
 
                    Offs[0] = pI2 + 2 * pIdx[1];
                    Offs[1] = pI2 + 2 * pIdx[2];
                    Offs[2] = pI2 + 2 * pIdx[3];
                    pIdx += 4;
                    CDB2_CPY_POLY( poly, Offs )
                    polyList.push_back( poly );
                } while ( --i );
            }
            
        break;
        case 4: 
            pI2 = ofs2 + 2 * (leafHI >> 13);
            if ( i ) {
                do {    
                    poly.mat[0] = (leafHI >> 7) & 63;
                    poly.mat[1] = pIdx[0] & 63;
                    poly.mat[2] = (pIdx[0] >> 6) & 1;
      
                    Offs[0] = pI2 + 2 * ( (pIdx[0] >> 7) | (2 * (pIdx[1] | (( pIdx[2] & 3) << 8) )) );
                    Offs[1] = pI2 + 2 * (((pIdx[3] & 31) << 6) | ( pIdx[2] >> 2));
                    Offs[2] = pI2 + 2 * ( (8 * pIdx[4]) | (pIdx[3] >> 5));
                    pIdx += 5;
                    CDB2_CPY_POLY( poly, Offs )
                    polyList.push_back( poly );
 
                } while ( --i );
            }
        break;
        case 5: 
            pI2 = ofs2 + 2 * (leafHI >> 13); 
            if ( i ) {
                do {
                    poly.mat[0] = (leafHI >> 7) & 63;
                    poly.mat[1] = pIdx[0] & 63;
                    poly.mat[2] = (pIdx[0] >> 6);
      
                    Offs[0] = pI2 + 2 * (pIdx[1] & 31);
                    Offs[1] = pI2 + 2 * ( (8 * (pIdx[2] & 3)) | (pIdx[1] >> 5)); 
                    Offs[2] = pI2 + 2 * (pIdx[2] >> 2);
                    pIdx += 3;
                    CDB2_CPY_POLY( poly, Offs )
                    polyList.push_back( poly );
   
                } while ( --i );
            }
  
        break;
        // large offsets:
        case 6: 
            if ( i ) {
                pI3 = (unsigned int*)pIdx;
                do {        
                    Offs[0] = ofs2 + 2 * pI3[0];
                    Offs[1] = ofs2 + 2 * pI3[1]; 
                    Offs[2] = ofs2 + 2 * pI3[2];
                    poly.mat[0] = *((byte *)pI3 + 12);
                    poly.mat[1] = *((byte *)pI3 + 13);
                    poly.mat[2] = *((byte *)pI3 + 14);
                    pI3 = (unsigned int *)((byte *)pI3 + 15);
                    CDB2_CPY_POLY( poly, Offs )
                    polyList.push_back( poly );
                } while ( --i );
            }
        break;
    }
        ret = 1;
        
    } while(0);

    return ret;
}
// read the cdb2 file
// return: 1 == OK; 0 == ERROR; -1 == could not open error
// -2 == not a valid file format
// -3 == tree parsing error
// -4 == memory error
int cTrackCollision::read() {
    int ret = 0;
    unsigned int leaves_written = 0;
    unsigned int *pCon = NULL;
    int tr_ret = -1;
    unsigned int *endPrt1 = NULL;
    do {
        
        if( buffer == NULL ) {
            std::ifstream cdb2S( pathName, std::ios::binary );
            if( !cdb2S.is_open() ){
                ret = -1;
                break;
            }
            buffer = new byte[ fileSize ];
            cdb2S.read ( (char*)buffer, fileSize );
            cdb2S.close();
        }
        pCon = (unsigned int*)buffer;
        if( buffer == NULL ) {
            ret = -4;
            break;
        }
        read_header();
        if( !is_valid ) {
            ret = -2;
            break;
        }
        // clear vector 
        polyList.clear();
        polyList.reserve( 60000 );
        // increment from begin of header to begin of AABB tree 
        pCon += (CDB2_HEADERSIZE / 4); // 64 bytes
        endPrt1 = (unsigned int*)((byte*)pCon + offset_1 ); // end of part 1 of cdb2 ( the AABB tree )

#ifndef TREE_DBG    
        // parse tree and find leaves
        do {
            // flag 3 is leave
            if ( ( pCon[0] & 3 ) == 3 ){ 
                // only if we have at least one triangle 
                if ( pCon[1] & 0x7F ) {
                    // lower 4 bytes and high 4 bytes of leaf 
                    tr_ret = make_faces( pCon[1], pCon[0], leaves_written);
                    if( tr_ret != 1 ) {
                        ret = -3;
                        break;  
                    }
                    leaves_written++;
                }
            }   
            
            //(nodes and leaves are all 8 bytes in size)
            pCon += 2;
        } while ( pCon < endPrt1 ); 
        
#else   
// for dbg  
        read_tree( pCon, endPrt1, 1 );
    
#endif // #ifdef TREE_DBG

        // has no effect when importing, just info
        if( polyN > 58000 ) { // max poly count is around 58000 with normal offsets
            use_largeOff = true;
        }
        
        if( ret == -3 )
            break;
        
        ret = 1;
        
    } while(0);
    
    delete[] buffer;
    buffer = NULL;
        
    return ret;
}
#endif

int cTrackCollision::read_tree(const std::vector<Primitive *> &_primitives, unsigned int some_flag ) {
    int ret = -1;
    int tr_ret = -1;
    cPoint3D _v1;
    cPoint3D _v2;
    cPoint3D _v3;
    cPoint3D o_max;
    cPoint3D o_min;
    Vector3D f_center;
	byte t_flag_r = 0;
	byte t_flag_e = 0;
	byte t_all_flags = 0;
    byte * Bptr = (byte*)(buffer + CDB2_HEADERSIZE);
	unsigned int tree_depth = 0;
    unsigned int nodeHI = 0;
    unsigned int nodeLO = 0;
    short val1 = 0, val2 = 0;
    byte flags = 0;
    unsigned int c1_off = 0;
    byte * absOff = Bptr;
    unsigned int childCountr = 0;
    byte ** offs = new byte*[1048576]; // stack
    unsigned int lvd_wr = 0; // leaves written
    unsigned int lastFIDX = 0; // last face idx
    unsigned int max_F_L = 0;
    float avg = 0.0f;
    float mults[3] = { div_x, div_y, div_z };
    std::vector<Vector3D> fac_center;
    unsigned int runCnt = 0;
    bool gets_bigger = false;
	unsigned int leafCnt_should = 0;
	unsigned int leafCnt_is = 0;
	bool leaf_match = false;
    primitivesPtr = _primitives; 
    cCDB2_node dummy;
    // points to the inner lists 
    std::vector<unsigned int>::iterator tmpF_it;
    BBox maxBB;
    
    do {

		if (Bptr == NULL)
			break;

    // clear tree
    AABB_tree.clear();
    tree_size = 0;
    totLeaves = 0;
    totFace_leaves = 0;
    AABB_tree.set_head( dummy );
    tree<cCDB2_node>::iterator cd1, cd2;
    tree<cCDB2_node>::iterator dbIt = AABB_tree.begin();
    tree_head = dbIt;
    dbIt->isRoot = true;
    dbIt->t_f.reserve( primitivesPtr.size() );
    // get biggest and smallest verts
    for( unsigned int i = 0; i < primitivesPtr.size(); ++i) {
		dbIt->bb.expand(primitivesPtr[i]->get_bbox());
		dbIt->t_f.push_back(i);	
    }

    
    // cache triagle centers
    fac_center.reserve( primitivesPtr.size() ); 
    for( unsigned int i = 0; i < primitivesPtr.size(); ++i) {
        OBJ_CENTER( primitivesPtr[i]->get_bbox().min, primitivesPtr[i]->get_bbox().max, f_center )
        fac_center.push_back(f_center);
    }
    
    // save max bb
    maxBB = dbIt->bb;
    
    
    do {
      
    while ( 1 ) {
        
    // normal node (first 4 bytes)
    nodeLO = *((unsigned int*)Bptr); 
    nodeHI = *((unsigned int*)Bptr + 1);
    
    //if( !(nodeLO & (some_flag << 2)) )
        //goto GO_BACK;

    
    // break if we have child
    // flags mean x, y, or z (0, 1, 2); 3 is then leaf
    flags = nodeLO & 3;
	t_all_flags = nodeLO & 0xFF;
    dbIt->t_flag = flags;
    dbIt->all_flags = t_all_flags; 

    // make axis, in cdb2 2 is y, 1 is z
    if( dbIt->t_flag == 0) {
        dbIt->t_axis = 0;
    } else if( dbIt->t_flag == 1) {
        dbIt->t_axis = 2;
    } else if( dbIt->t_flag == 2){
        dbIt->t_axis = 1;
    } else if( dbIt->t_flag == 3 ) {
        dbIt->t_axis = 3;
    }

    //flags ? (dbIt->t_axis = ( (flags == 1) ? 2 : 1 )) : dbIt->t_axis = 0;

    if ( flags == 3 ) {
        dbIt->isLeaf = true; 
        cd1->l = false; cd2->r = false;/*not really necessary*/
        break;
    } 
 
    dbIt->flag_r = ((nodeLO & 32) >> 5);
	t_flag_r = ((nodeLO & 32) >> 5);
    dbIt->flag_e = ((nodeLO & 4) >> 2);
	t_flag_e = ((nodeLO & 4) >> 2);
    
    dbIt->loBound = (float)(*(short *)(Bptr + 4) * mults[dbIt->t_axis] * CDB2_SCALE);
	val1 = *(short *)(Bptr + 4); 
    dbIt->hiBound = (float)(*(short *)(Bptr + 6) * mults[dbIt->t_axis] * CDB2_SCALE);
    val2 = *(short *)(Bptr + 6);
  

    cd1 = AABB_tree.append_child(dbIt, dummy );
    cd1->bb = dbIt->bb;

    cd2 = AABB_tree.append_child(dbIt, dummy );
    cd2->bb= dbIt->bb;
    cd1->l = true; cd2->r = true;

    /* resize the left CHILD AABB according to bounds */
	switch( dbIt->t_axis ){
            case 0:
            if(cd1->bb.max.x < -dbIt->hiBound || cd1->bb.min.x > -dbIt->loBound)
                gets_bigger = true; /* for dbg*/
			cd1->bb.max.x = -dbIt->hiBound; cd1->bb.min.x = -dbIt->loBound; break;
            case 2: /* in the renderer axis is swapped*/
            if(cd1->bb.max.y < dbIt->hiBound || cd1->bb.min.y > dbIt->loBound)
                gets_bigger = true; /* for dbg*/

			cd1->bb.max.y = dbIt->hiBound; cd1->bb.min.y = dbIt->loBound; 
 
            break;
            case 1:
            if(cd1->bb.max.z < dbIt->hiBound || cd1->bb.min.z > dbIt->loBound)
                gets_bigger = true; /* for dbg*/
                cd1->bb.max.z = dbIt->hiBound; cd1->bb.min.z = dbIt->loBound; 

            break;
            default: break;
        } 

	

    /*check which triangles are in this AABB.*/
    if( runCnt ) {
		
		tmpF_it = AABB_tree.parent(dbIt)->t_f.begin();
        for( unsigned int i = 0; i < AABB_tree.parent(dbIt)->t_f.size(); ++i)
            if( vert_in_AABB( dbIt->bb.min, dbIt->bb.max, fac_center[tmpF_it[i]] ) ) {
                dbIt->t_f.push_back(tmpF_it[i]);
            } else {
                cd2->t_f.push_back(tmpF_it[i]);
            }
                
    }
    
    // resize right chilf node bb 
    for( unsigned int i = 0; i < cd2->t_f.size(); ++i) {
		cd2->bb.expand(primitivesPtr[(cd2->t_f[i])]->get_bbox());
	}

    dbIt = cd1; 
    
    // node map :
    //                  nodeLO                          nodeHI
    // 000000000000000000000000 00000000 | 0000000000000000 0000000000000000 |
    //   child 1 offset          flags         val1            val2
    
    // child 1 offset
    c1_off = nodeLO >> 8;   

    //val1f = (float)val1 * *((float *)BuffPtr + 3 + flags);    
    //val2f = (float)val2 * *((float *)BuffPtr + 3 + flags);
          
    // goto his first child
    Bptr = absOff + c1_off;
    // ptr to sibling node
    offs[childCountr] = Bptr + 8;
    childCountr++;
    runCnt++;
    
    } // while (1)
        
    // only if whe have leaf, if num in leaf is not zero 
	leafCnt_should = nodeHI & 0x7F;
	tmpF_it = AABB_tree.parent(dbIt)->t_f.begin();
		for (unsigned int i = 0; i < AABB_tree.parent(dbIt)->t_f.size(); ++i)
			if (vert_in_AABB(dbIt->bb.min, dbIt->bb.max, fac_center[tmpF_it[i]]))
				dbIt->t_f.push_back(tmpF_it[i]);

		leafCnt_is = dbIt->t_f.size();
		leaf_match = leafCnt_is == leafCnt_should;
        lvd_wr++;
        totLeaves++;

    
	/* goes back*/
    if ( !childCountr )
      break;
  
    // else decrement and assign values
    childCountr--;  
    Bptr = offs[childCountr];
    
    ++dbIt;
    
    } while ( 1 );
  
        tree_size = AABB_tree.size();
		tree_depth = AABB_tree.max_depth();
		tree_build = true;
        ret = 1;
    } while(0);
    
    delete[] offs;
    
    return ret;
}


#if 0
// TODO: improve : try to create a tree similar to orig one
// builds AABB tree of the mesh
// return: 1 == success; -1 == error; -2 == logic error
int cTrackCollision::build_tree() {
        
    unsigned int ret = -1; 
    // start with z axis 
    byte axis = 2; 
    unsigned int remaining_fac = faces.size();
    
    cPoint3D o_max, o_min, cd1_min, cd1_max, cd2_min, cd2_max;
    
    unsigned int in_cnt = 0;
    unsigned int out_cnt = 0;
    unsigned int try_cnt = 0;
    float upnLo[2] = {};
    float upnLo2[2] = {};
    // sometimes splitting fails, didn't checked it geometrically
    // but two triangles with same vertices could be possible
    unsigned max_split_fail = 0;
    unsigned int tmp_nmbr = 0;
    unsigned int depth_cnt = 0;
    
    float mults[3] = { mult_x, mult_y, mult_z };
    
    if( remaining_fac && vertices.size() ){
    
    ret = 1;
    std::vector<cCDB2_f>::iterator fi = faces.begin();
    std::vector<cPoint3D>::iterator vi = vertices.begin();
    
    
    
    // dimensions of map box 
    // length, width, height
    float lwH[3] = { biggest_vert.x - smalles_vert.x, biggest_vert.y - smalles_vert.y, biggest_vert.z - smalles_vert.z };
    // box center point
    float b_center[3] = { biggest_vert.x - lwH[0] / 2,  biggest_vert.y - lwH[1] / 2,  biggest_vert.z - lwH[2] / 2 };
    
    // current box 
    float tmp_lwh[3] = { lwH[0] / 2, lwH[1] / 2, lwH[2] / 2 };
    float tmp_lwh2[3];
    float tmp_center[3] = { b_center[0], b_center[1], b_center[2] };
    float tmp_center2[3];

    // empty dummy 
    cCDB2_node dummy;
    cPoint3D f_center;
    // points to the inner lists 
    std::vector<unsigned int>::iterator tmp_it;
    
    // clear tree
    AABB_tree.clear();
    tree_size = 0;
    totLeaves = 0;
    totFace_leaves = 0;
    AABB_tree.set_head( dummy );
    tree<cCDB2_node>::iterator cd1;
    tree<cCDB2_node>::iterator cd2;
    tree<cCDB2_node>::iterator dbIt = AABB_tree.begin();
    
    // set in head the BB of whole mesh
    dbIt->t_axis = axis;
    upnLo[0] = (tmp_center[axis] + tmp_lwh[axis]); upnLo[1] = (tmp_center[axis] - tmp_lwh[axis]);
    dbIt->tv1 = (short)( upnLo[0] > 32767 ? 32767 : ( upnLo[0] < -32767 ? -32767 : upnLo[0] ) );
    dbIt->tv2 = (short)( upnLo[1] > 32767 ? 32767 : ( upnLo[1] < -32767 ? -32767 : upnLo[1] ) ); 
    CPY_AR( dbIt, tmp_lwh, tmp_center)  
    /* make flag */
    axis ? (dbIt->t_flag = ( (axis == 1) ? 2 : 1 )) : (dbIt->t_flag = 0);
            

    // write table with center of faces 
    std::vector<cPoint3D> fac_center;
    fac_center.reserve(remaining_fac);
    for(unsigned int cnt = 0; cnt < remaining_fac; cnt++) {
        
        o_min.x = FastMin( FastMin( vi[fi[cnt].f1].x, vi[fi[cnt].f2].x ), vi[fi[cnt].f3].x );
        o_max.x = FastMax( FastMax( vi[fi[cnt].f1].x, vi[fi[cnt].f2].x ), vi[fi[cnt].f3].x );
        o_min.y = FastMin( FastMin( vi[fi[cnt].f1].y, vi[fi[cnt].f2].y ), vi[fi[cnt].f3].y );   
        o_max.y = FastMax( FastMax( vi[fi[cnt].f1].y, vi[fi[cnt].f2].y ), vi[fi[cnt].f3].y );
        o_min.z = FastMin( FastMin( vi[fi[cnt].f1].z, vi[fi[cnt].f2].z ), vi[fi[cnt].f3].z );
        o_max.z = FastMax( FastMax( vi[fi[cnt].f1].z, vi[fi[cnt].f2].z ), vi[fi[cnt].f3].z );       
        OBJ_CENTER( o_min, o_max, f_center )
        fac_center.push_back(f_center);
    }
    
    do {
            
        if( depth_cnt ) { 
        
            remaining_fac = dbIt->t_f.size();
            tmp_it = dbIt->t_f.begin();
                
            // copy the box from the node 
            CPY_R_AR ( dbIt, tmp_lwh, tmp_center)               
            
            // copy axis back from node 
            dbIt->t_axis = axis;
                
            // increment axis 
            axis == 2 ? axis = 0 : axis++;                  
        }   
        if( remaining_fac > max_polys_leaf ) {
        
        cd1 = AABB_tree.append_child(dbIt, dummy );
        cd2 = AABB_tree.append_child(dbIt, dummy );
        dbIt = cd1;
        // nice
        goto SUBBOX;
        
SUBBOX_AXIS_INC:
        
        // try only 12 times to split 
        if( try_cnt > 12) {
            
            if( ( in_cnt > out_cnt ? in_cnt : out_cnt ) > max_split_fail) {
                max_split_fail = ( in_cnt > out_cnt ? in_cnt : out_cnt );
            }
            
            in_cnt ?  (cd1->t_flag = 3) : (cd2->t_flag = 3);
            
            goto WRITE_NODE;
        }
        
        // increment axis 
        axis == 2 ? axis = 0 : axis++;
        try_cnt++;
        
SUBBOX:             
        // the axis for the vals, not the next axis
        cd1->t_axis = axis;
        cd2->t_axis = axis; 
        // make flag 
        if( axis ) {
            // axis is swapped, in the cdb2 2 is y, 1 is z
            cd1->t_flag = ( (axis == 1) ? 2 : 1 );
            cd2->t_flag = cd1->t_flag;
        }
        else {
            cd1->t_flag = 0;
            cd2->t_flag = 0;
        }
        

        // new dimensions
        tmp_lwh[axis] /= 2;
        tmp_center[axis] -= tmp_lwh[axis];
                
        in_cnt = 0;
        out_cnt = 0;
        
        cd1->t_f.clear();
        cd2->t_f.clear();
        
        // calculate min and max of aabb 
        CALC_MIN_MAX( o_min, o_max, tmp_lwh, tmp_center)    

        for(unsigned int i = 0; i < remaining_fac; i++) {
        
        // first, parse all faces 
        depth_cnt ? (tmp_nmbr = tmp_it[i]) : (tmp_nmbr = i);
            
        // check if the triangle intersects the box 
        if( vert_in_AABB( o_min, o_max, fac_center[tmp_nmbr] ) ) {          
            cd1->t_f.push_back(tmp_nmbr);   
            // set min and max to be able to resize the boxes 
            MIN_MAX_VERTS( in_cnt, vi[fi[tmp_nmbr].f1], cd1_min, cd1_max )
            MIN_MAX_VERTS( in_cnt, vi[fi[tmp_nmbr].f2], cd1_min, cd1_max )
            MIN_MAX_VERTS( in_cnt, vi[fi[tmp_nmbr].f3], cd1_min, cd1_max )
            in_cnt++;

        }
        else {          
            cd2->t_f.push_back(tmp_nmbr);
            MIN_MAX_VERTS( out_cnt, vi[fi[tmp_nmbr].f1], cd2_min, cd2_max )
            MIN_MAX_VERTS( out_cnt, vi[fi[tmp_nmbr].f2], cd2_min, cd2_max )
            MIN_MAX_VERTS( out_cnt, vi[fi[tmp_nmbr].f3], cd2_min, cd2_max )
            out_cnt++;
        }
    
        }   

        // if split was not successfull
        if( !in_cnt || !out_cnt)
            goto SPLIT_TRY;
        
WRITE_NODE:     
        try_cnt = 0;
        // resize the box, calc the new one
        CALC_AABB( cd1_min, cd1_max, tmp_lwh, tmp_center )
        CALC_AABB( cd2_min, cd2_max, tmp_lwh2, tmp_center2 )    
        
        // copy upper n lower bound values to node 
        // make the upper n lower bounds be a bit bigger than box
#define BIG_F 0.00
        upnLo[0] = (tmp_center[axis] + tmp_lwh[axis]); upnLo[1] = (tmp_center[axis] - tmp_lwh[axis]);
        upnLo2[0] = (tmp_center2[axis] + tmp_lwh2[axis]); upnLo2[1] = (tmp_center2[axis] - tmp_lwh2[axis]);
        float dista1 = 
            ( FastMax(upnLo[0], upnLo[1]) - FastMin(upnLo[0], upnLo[1]) );
        float dista2 =
            ( FastMax(upnLo2[0], upnLo2[1]) - FastMin(upnLo2[0], upnLo2[1]) );
        
        //
        // make bigger bounds outwards
        //
        ( upnLo[0] > upnLo[1] ? upnLo[0] : upnLo[1]) += (dista1 * BIG_F); // make bigger one even larger...
        ( upnLo[0] < upnLo[1] ? upnLo[0] : upnLo[1]) -= (dista1 * BIG_F); // ... and smaller even smaller;
        // same for second one:
        ( upnLo2[0] > upnLo2[1] ? upnLo2[0] : upnLo2[1]) += (dista2 * BIG_F); 
        ( upnLo2[0] < upnLo2[1] ? upnLo2[0] : upnLo2[1]) -= (dista2 * BIG_F); 

        
        // mult with multiplier for short
        upnLo[0] *= mults[axis]; upnLo[1] *= mults[axis]; upnLo2[0] *= mults[axis]; upnLo2[1] *= mults[axis];
        // truncate to max range of short 
        cd1->tv1 = (short)( upnLo[0] > 32767 ? 32767 : ( upnLo[0] < -32767 ? -32767 : upnLo[0] ) );
        cd1->tv2 = (short)( upnLo[1] > 32767 ? 32767 : ( upnLo[1] < -32767 ? -32767 : upnLo[1] ) ); 
        CPY_AR( cd1, tmp_lwh, tmp_center)  
        
        cd2->tv1 = (short)( upnLo2[0] > 32767 ? 32767 : ( upnLo2[0] < -32767 ? -32767 : upnLo2[0] ) );
        cd2->tv2 = (short)( upnLo2[1] > 32767 ? 32767 : ( upnLo2[1] < -32767 ? -32767 : upnLo2[1] ) );  
        CPY_AR( cd2, tmp_lwh2, tmp_center2) 
        
        goto NEXT;
SPLIT_TRY:      
            
        if( (in_cnt && in_cnt < max_polys_leaf ) || (out_cnt && out_cnt < max_polys_leaf )) {
            ret = -2; // "error: poly_cnt && poly_cnt < max_polys_leaf"
            break;
        }
                
            
        // if all faces were in the subbox, go again and split the subbox itself another time
        if( !out_cnt ) {
            goto SUBBOX_AXIS_INC;
        }
            
        // go to sibling box and split it
        if( !in_cnt ) {
            tmp_center[axis] += 2 * tmp_lwh[axis];
            goto SUBBOX_AXIS_INC;
        }
    
NEXT:       
        if ( in_cnt <= max_polys_leaf || cd1->t_flag == 3 ) {
                    
            // goto child2
            if( out_cnt > max_polys_leaf && cd2->t_flag != 3 ) {
                ++dbIt;
            } else {    
                // goto next
                std::advance( dbIt, 2);
            }
            // count faces in leaves
            totFace_leaves += remaining_fac;
            totLeaves++;
        }
        
        // set the flags 
        if( in_cnt <= max_polys_leaf ) {
            cd1->t_flag = 3;
        }
        if( out_cnt <= max_polys_leaf ) {
            cd2->t_flag = 3;
        }
        
        }
        else {
            dbIt->t_flag = 3;
            ++dbIt;
            // count faces in leaves
            totFace_leaves += remaining_fac;
            totLeaves++;
        } // if remaining_fac < max poly leaf and therefore node is leaf
            
        depth_cnt++;
        
    }
    while( dbIt != AABB_tree.end() );
    
    }
    tree_size = AABB_tree.size();
    
    return ret; 
}


// serialize the tree and write to buffer
// returns: 1 == success; -1 == error; -2 == some logic error, set to prevent mem corruptions
// and AVs, cuz we have here much ptr arithmetic
int cTrackCollision::serialize_tree( byte *part1, byte *part2 ) { 

    int ret = -1;
    std::vector<unsigned int>::iterator tri_it;
    std::vector<cCDB2_f>::iterator fi2 = faces.begin();
    bool reverse = false;
    
    unsigned int ac_prt2off = 0, remaining_fac = 0, facs_written = 0, 
    abs_offs = 0, nxt_off = 8, cnt1 = 0, frst_fac = 0, recur_cn = 0,
    // some temp vars
    tmp_f1 = 0, tmp_f2 = 0, tmp_f3 = 0, tmp_mat1 = 0,
    // save 1st mat, to save it later in leaf
    _mat1 = 0,
    tmp_mat2 = 0, tmp_mat3 = 0,
    hi_t = 0, sw_flag = 0, leaves_written = 0;
    
    prt1 = 0;
    prt2 = 0;
    
    // this iterator runs depth_first
    tree<cCDB2_node>::iterator dbIt = AABB_tree.begin();
    unsigned int * offsets_buff = new unsigned int[ tree_size ];
    
    
    do {

        if( dbIt == AABB_tree.end() ) {
            ret = 1;
            break;
        }

        // for dbg
        if( prt1 > p1_should) {
            break;
        } 
        
        // points to begin of face list of node  
        tri_it = dbIt->t_f.begin();
        // get size of current list 
        remaining_fac = dbIt->t_f.size();



        if( !cnt1 ) {
            // the first offset is 8
            *(unsigned int *)part1 = ( (8 << 8) | ( dbIt->flag_r << 5) | 
            ( dbIt->flag_e << 2) |  dbIt->t_flag | dbIt->all_flags ); 
            *(short *)(part1 + 4) = dbIt->tv1;
            *(short *)(part1 + 6) = dbIt->tv2;
            prt1 += 8;
            abs_offs += 8;
            cnt1++;
                
        }
        else {
        
        // write leaves and faces in part 2 
        if(  dbIt->t_flag == 3 ) {

            for(unsigned int cnt2 = 0; cnt2 < remaining_fac; cnt2++ ) {
                    
                // 3 vertices of face 
                tmp_f1 = fi2[tri_it[cnt2]].f1;
                tmp_f2 = fi2[tri_it[cnt2]].f2;
                tmp_f3 = fi2[tri_it[cnt2]].f3;
                // materials
                tmp_mat1 = fi2[tri_it[cnt2]].mat[0];
                // save 1st mat to write it in leave 
                if( !cnt2 )
                    _mat1 = tmp_mat1;
                tmp_mat2 = fi2[tri_it[cnt2]].mat[1];
                tmp_mat3 = fi2[tri_it[cnt2]].mat[2];
                            
                // for dbg
                if( prt2 > p2_should) {
                    break;
                }
                
                // default method, max suitable for 58254 faces 
                if( !use_largeOff ) {
                
                // check not really necessary 
                if ( 
                     tmp_f1 < 174762 && 
                     tmp_f2 < 174762 &&
                     tmp_f3 < 174762
                     ) {
                                     
                        // write only the first time
                        if( !cnt2 ) { 
                                    
                            // the first offset is in the leaf      
                            MAKE_OFF_CE10_F( ( tmp_f2 * 3 ),
                                             ( tmp_f3 * 3 ),
                                             part2, tmp_mat2, tmp_mat3 
                                             )                                                      
                                                     
                            part2 += 6;
                            prt2 += 6;
                        
                        }
                        else { 
                                    
                            MAKE_OFF_CE10(  ( tmp_f1 * 3 ), 
                                            ( tmp_f2 * 3 ), 
                                            ( tmp_f3 * 3 ),
                                            part2, tmp_mat1, tmp_mat2, tmp_mat3
                                            )                                                       
                                            
                            part2 += 9;
                            prt2 += 9;

                        }
                                    
                } else {
                    ret = -2;
                    break;
                }
                
                //choose sw flag
                sw_flag = 0;
                }
                // use 32bit int for the offsets to be able to export more triangles, max is 2^32
                // but now needs code modification in the game 
                else {
                    
                    MAKE_OFF_EXT( ( tmp_f1 * 3 ), 
                                  ( tmp_f2 * 3 ), 
                                  ( tmp_f3 * 3 ),
                                   part2, tmp_mat1, tmp_mat2, tmp_mat3
                                  )
                    part2 += 15;
                    prt2 += 15;
                    
                    sw_flag = 6;
                }
                
                
                // for dbg
                if( prt2 > p2_should) {
                    break;
                }
                            
                            
            }
            
            if( ret == -2 ) {
                break; // break also out of do-while
            }
            
            // break also out of do-while
            if( prt2 > p2_should) {
                break;
            }
                
            // for the first offset in the tree block
            if( remaining_fac && !use_largeOff ) { 
                frst_fac = fi2[tri_it[0]].f1 * 3;
                if( frst_fac > 0x0007FFFF ) { // frst_fac offset can have only 19 bytes
                    ret = -2;
                    break;
                }
            }
            else { 
                frst_fac = 0;
            }
    
            
            // write leaf
            // high int
            *(unsigned int *)(part1 + abs_offs) = ( (ac_prt2off << 9) | 39 | ( sw_flag << 6 ) );
            // low int
            *(unsigned int *)(part1 + abs_offs + 4) = ( ( ( frst_fac & 0x0007FFFF ) << 13 ) | ( ( _mat1 & 63 ) << 7 ) | 
                            (remaining_fac & 127) );                
            
            leaves_written++;
            facs_written += remaining_fac;

            if( abs_offs >= prt1 )
                prt1 += 8;

            // for next time, absolute offset
            ac_prt2off = prt2;  
        
            // write in cdb2 
            reverse = true;
        
            if( recur_cn ) {
                
                recur_cn--;
                abs_offs = offsets_buff[recur_cn];
            }
            else {      
                ret = 1;
                break;
            }

            
            

        } else { // write normal node
    
                
                if( reverse && abs_offs < prt1  ) {
                    nxt_off = prt1;
                } else {
                    nxt_off = abs_offs + 16;
                }
                
                // flags for ?road and ?other track elem are set
                hi_t = ( ( dbIt->flag_r << 5) | ( dbIt->flag_e << 2) |
                // axis flag 
                dbIt->t_flag |
                // next child offset
                ( nxt_off << 8) | 
                dbIt->all_flags
                );


                *(unsigned int *)(part1 + abs_offs) = hi_t; 
                *(short *)(part1 + abs_offs + 4) = dbIt->tv1; 
                *(short *)(part1 + abs_offs + 6) = dbIt->tv2;
    

                if( abs_offs >= prt1 ) {

                    prt1 += 16;
                    abs_offs += 16;

                } else {

                abs_offs = prt1;

                }
                
                reverse = false;    
                
        } // end if normal node and not a leaf
        } // end if cnt, -> if not first 
        
        if( !reverse ) { 
        
        if( recur_cn >= tree_size ) {
            ret = -2;
            break;
        }
        
        // offset to sibling node
        offsets_buff[recur_cn] = nxt_off + 8;   
        recur_cn++;
        
        }
        
        // for dbg
        if( prt1 > p1_should) {
            bool tmp = reverse;
            reverse = false;
            reverse = tmp;
        }
                
                
    
        ++dbIt;

#pragma warning( push )
#pragma warning( disable : 4127)
        } while( 1 );       
#pragma warning( pop ) 
    

    delete[] offsets_buff;
    
    return ret;
    
}

#endif


// read cdb2 header
// return: 1 == success; -1 == ptr is NULL
int cTrackCollision::read_header() {
    byte *b = buffer;
    if( b != NULL ) {

        plate = *(unsigned int *)b;
        flag  = *(unsigned int *)(b + 4);
        min_x = *(int *)(b + 8);
        min_y = *(int *)(b + 12);
        min_z = *(int *)(b + 16);
        max_x = *(unsigned int *)(b + 20);
        max_y = *(unsigned int *)(b + 24);
        max_z = *(unsigned int *)(b + 28);
        div_x = *(float *)(b + 32);
        div_z = *(float *)(b + 36);
        div_y = *(float *)(b + 40);
        mult_x = *(float *)(b + 44);
        mult_z = *(float *)(b + 48);
        mult_y = *(float *)(b + 52);
        offset_1 = *(unsigned int *)(b + 56);
        offset_2 = *(unsigned int *)(b + 60);
        
        if( plate == CDB2_ID) {
            is_valid = true;
        }
        
        return 1;
    } 
    return -1;
}

__int64  cTrackCollision::get_filesize(std::wstring path)
{
	HANDLE hFile;
	hFile = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
		return -1;

	LARGE_INTEGER thesize;
	GetFileSizeEx(hFile, &thesize);
	CloseHandle(hFile);
	return thesize.QuadPart;
}


inline bool cTrackCollision::vert_in_AABB(Vector3D min, Vector3D max, Vector3D vert) {
	if (vert.x <= max.x && vert.x >= min.x &&
		vert.y <= max.y && vert.y >= min.y &&
		vert.z <= max.z && vert.z >= min.z) {
		return true;
	}
	else {
		return false;
	}
}

#endif // #ifdef WITH_FO2_COLLISION

