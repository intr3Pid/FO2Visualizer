
#include "FO2_collision.h"
#include <stack>

/**
 * default constructor
 * reads whole file in memory if file path provided
 * \param path path to file
 */
cTrackCollision::cTrackCollision(string path) {
    pathName = "";
    buffer = nullptr;
    primitivesPtr = nullptr;
    polyN = 0;
    is_valid = false;
    use_largeOff = false;
	tree_build = false;
    matsAdded = false;
    tree_size = 0;
	filesize = 0;
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
		if (path.length()) {

			pathName = path;
			wstring t_wpath(pathName.begin(), pathName.end());
			uint64_t fSize = 0;
			fSize = get_filesize(t_wpath);
			if (fSize > 0 && fSize < CDB2_MAX_FILESIZE) {
				filesize = (unsigned int)fSize;
			}
			else {
				break;
			}
			//
			// read whole file in memory
			//
			ifstream cdb2S(pathName, ios::binary);
			if (!cdb2S.is_open()) {
				break;
			}
			buffer = new byte[filesize];
			cdb2S.read((char*)buffer, filesize);
			cdb2S.close();
			//
			// read header
			//
			read_header();

		}

	} while (false);
    
}
  
/**
 * Reads the tree from the internal buffer and fills creates the AABB_tree member
 * tree. Calculates the primitives(only trianlges)  which are inside every node
 * \param _primitives pointer to primitives std::vector, they areput in the nodes
 *						of the tree
 * \return	-1 Failed, internal file buffer is null
 *			-2 Failed, encoutered invalid offset while reading the tree 
 *			1 Success
 *			
 */
int cTrackCollision::read_tree(vector<Primitive*> *_primitives) {

 
    
    cPoint3D _v1;
    cPoint3D _v2;
    cPoint3D _v3;
    cPoint3D o_max;
    cPoint3D o_min;
    Vector3D f_center;
	byte t_all_flags = 0;
    byte * Bptr = (byte*)(buffer + CDB2_HEADERSIZE);
	unsigned int tree_depth = 0;
    unsigned int nodeHI = 0;
    unsigned int nodeLO = 0;
    byte flags = 0;
    unsigned int c1_off = 0;
    byte * absOff = Bptr;
    
	stack<byte *, vector<byte*>> offs;
   
    unsigned int lvd_wr = 0; // leaves written

    float mults[3] = { div_x, div_y, div_z };
    vector<Vector3D> fac_center;
    unsigned int runCnt = 0;
	unsigned int leafCnt_should = 0;
	unsigned int leafCnt_is = 0;

	primitivesPtr = _primitives;

    cCDB2_node dummy;
    // points to the inner lists 
    vector<unsigned int>::iterator tmpF_it;

    
	do {

		if (Bptr == nullptr)
			return -1;

		// clear tree
		AABB_tree.clear();
		tree_size = 0;
		totLeaves = 0;


		// set and head and head iterator
		AABB_tree.set_head(dummy);
		tree<cCDB2_node>::iterator cd1, cd2;
		tree<cCDB2_node>::iterator dbIt = AABB_tree.begin();
		tree_head = dbIt;


		dbIt->isRoot = true;
		dbIt->t_f.reserve((*primitivesPtr).size());

		// get biggest and smallest verts
		for (unsigned int i = 0; i < (*primitivesPtr).size(); ++i) {
			dbIt->bb.expand((*primitivesPtr)[i]->get_bbox());
			// root node has all triangles
			dbIt->t_f.push_back(i);
		}


		// save triagle centers
		fac_center.reserve((*primitivesPtr).size());
		for (unsigned int i = 0; i < (*primitivesPtr).size(); ++i) {
			OBJ_CENTER((*primitivesPtr)[i]->get_bbox().min, (*primitivesPtr)[i]->get_bbox().max, f_center)
				fac_center.push_back(f_center);
		}

		do {

			while (true) {

				// normal node (first 4 bytes)
				nodeLO = *((unsigned int*)Bptr);
				nodeHI = *((unsigned int*)Bptr + 1);

				// break if we have child
				// flags mean x, y, or z (0, 1, 2); 3 is then leaf
				flags = nodeLO & 3;
				t_all_flags = nodeLO & 0xFF;
				dbIt->t_flag = flags;
				dbIt->all_flags = t_all_flags;

				// make axis, in cdb2 2 is y, 1 is z
				if (dbIt->t_flag == 0) {
					dbIt->t_axis = 0;
				}
				else if (dbIt->t_flag == 1) {
					dbIt->t_axis = 2;
				}
				else if (dbIt->t_flag == 2) {
					dbIt->t_axis = 1;
				}
				else if (dbIt->t_flag == 3) {
					dbIt->t_axis = 3;
				}

				if (flags == 3) {
					dbIt->isLeaf = true;
					cd1->l = false; cd2->r = false;/*not really necessary*/
					break;
				}

				dbIt->flag_r = ((nodeLO & 32) >> 5);		
				dbIt->flag_e = ((nodeLO & 4) >> 2);			
				dbIt->hiBound = (float)(*(short *)(Bptr + 4) * mults[dbIt->t_axis] * FO2_SCALE);			
				dbIt->loBound = (float)(*(short *)(Bptr + 6) * mults[dbIt->t_axis] * FO2_SCALE);
			


				cd1 = AABB_tree.append_child(dbIt, dummy);
				cd1->bb = dbIt->bb;

				cd1 = AABB_tree.begin(dbIt);

				cd2 = AABB_tree.append_child(dbIt, dummy);
				cd2->bb = dbIt->bb;

				cd1->l = true; cd2->r = true;

				/* resize the left CHILD AABB according to bounds */
				switch (dbIt->t_axis) {
				case 0:
					cd1->bb.max.x = -dbIt->hiBound; 
					cd1->bb.min.x = -dbIt->loBound; break;
				case 2: /* in the renderer axis is swapped*/			
					cd1->bb.max.y = dbIt->hiBound; 
					cd1->bb.min.y = dbIt->loBound;

					break;
				case 1:
					cd1->bb.max.z = dbIt->hiBound; 
					cd1->bb.min.z = dbIt->loBound;

					break;
				default: break;
				}


				/*check, which triangles are in this AABB.*/

					tmpF_it = dbIt->t_f.begin();
					size_t tmpP_size = dbIt->t_f.size();

					for (size_t i = 0; i < tmpP_size; i++)
						if (VERT_IN_AABB(cd1->bb.min, cd1->bb.max, fac_center[tmpF_it[i]])) {
							cd1->t_f.push_back(tmpF_it[i]);
						}
						else {
							cd2->t_f.push_back(tmpF_it[i]);
						}

						if (cd2->t_f.size())
						{
							// resize right chilf node bb, maybe not for the orig fo2 files ? 
						    // first, set minimal bbox which can hold the first triangle
							cd2->bb = (*primitivesPtr)[cd2->t_f[0]]->get_bbox();
							// now, expand
							for (unsigned int i = 0; i < cd2->t_f.size(); ++i) {
								cd2->bb.expand((*primitivesPtr)[cd2->t_f[i]]->get_bbox());
							}
						}
				

				dbIt = cd1;

				// child 1 offset
				c1_off = nodeLO >> 8;

				// goto his first child
				Bptr = absOff + c1_off;

				// check if the offset is valid
				if (Bptr >= absOff && Bptr <= (byte*)(absOff + filesize))
				{
					// ptr to sibling node
					offs.push(Bptr + 8);
				} else
				{
					// invalid offset
					return -2;
				}
				
			


			} // end while 1

		    // only if whe have leaf, if num in leaf is not zero 
			leafCnt_should = nodeHI & 0x7F;
			tmpF_it = AABB_tree.parent(dbIt)->t_f.begin();
			size_t tmpP_size = AABB_tree.parent(dbIt)->t_f.size();

			for (unsigned int i = 0; i < tmpP_size; i++)
				if (VERT_IN_AABB(dbIt->bb.min, dbIt->bb.max, fac_center[tmpF_it[i]]))
					dbIt->t_f.push_back(tmpF_it[i]);

			leafCnt_is = dbIt->t_f.size();
			lvd_wr++;
			totLeaves++;



			if (offs.size())
			{
				Bptr = offs.top();
				offs.pop();
			} else
			{
				break;
			}	

			++dbIt;

		} while (1);

		tree_size = AABB_tree.size();
		tree_build = true;
	
	} while (false);
 

    return 1;
}


// read cdb2 header
// return: 1 == success; -1 == ptr is NULL
int cTrackCollision::read_header() {
    byte *b = buffer;
    if( b != nullptr ) {

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

uint64_t  cTrackCollision::get_filesize(wstring path)
{

#ifndef FILESIZE_USE_WIN_SPECIFIC
	uint64_t
	file_size = -1;

	// force MSVC to use 64bit integer for st_size field, should work ..
#ifdef _MSC_VER
	struct _stat32i64 stbuf;
#else
	struct stat stbuf;
#endif
	
	int res;
	string s_path(path.begin(), path.end());
	do
	{
		res = open(s_path.c_str(), O_RDONLY);
		if (res == -1) {
			break;
		}

		if (
// MSVC specific fstat with 64bit integer for st_size field, should work .. 
#ifdef _MSC_VER
			_fstat32i64
#else
			fstat
#endif
			(res, &stbuf) != 0) {
			break;
		}
		// return is type is always 64bit, but I havent tested if 
		// on other platforms st_size is 64bit
		file_size = (uint64_t)stbuf.st_size;

	} while (false);
	
	return file_size;
#else

	// Windows specific code snipped with the same functionality, except is is safe to use for files
	// larger than 4GiB
	HANDLE hFile = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
		return -1;

	LARGE_INTEGER thesize;
	GetFileSizeEx(hFile, &thesize);
	CloseHandle(hFile);
	return (uint64_t)thesize.QuadPart;
#endif // ifndef FILESIZE_USE_WIN_SPECIFIC

}




