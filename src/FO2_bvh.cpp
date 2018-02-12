#include "FO2_bvh.h"
#include <stack>


// a dump of coordinates used from the game for frustum culling check
float cTrackBVH::some_comp_hard[42] = {
	// normal vectors, complementary have direction inverted, except the z coord 
	0.270623, 0.751731, 0.601385, -0.270623, -0.751731, 0.601385, 34.9181,
	0.270656, 0.751724, 0.601379, 0.270656, 0.751724, -0.601379, 265.001,
	0.944349, 0.328851, 0.00787188, -0.944349, 0.328851, 0.00787188, -53.6213,
	0.937264, 0.34853, 0.00787188, 0.937264, -0.34853, 0.00787188, 54.538,
	0.207226, 0.575627, 0.791019, -0.207226, -0.575627, -0.791019, 29.3451,
	0.200141, 0.555947, 0.806763, 0.200141, 0.555947, 0.806763, -28.4283
};

cTrackBVH::cTrackBVH(string path)
{
	ID = FO2_BVH_ID; version = 1; surf_count = 0; surf_count2 = 0; buffer = NULL;
	filesize = 0; tree_build = false; primitivesPtr = nullptr;

	do {
		if (path.length() && path != "-1") {

			pathName = path;
			wstring t_wpath(pathName.begin(), pathName.end());
			uint64_t fSize = 0;
			fSize = cTrackCollision::get_filesize(t_wpath);
			if (fSize > 0 && fSize < FO2_BVH_MAX_FILESIZE) {
				filesize = (unsigned)fSize;
			}
			else {
				break;
			}
			//
			// read whole file in memory
			//
			ifstream is(pathName, ios::binary);
			if (!is.is_open()) {
				break;
			}
			buffer = new byte[filesize];
			is.read((char*)buffer, filesize);
			is.close();
			unsigned int *uint_buff = (unsigned int*)buffer;
			if (buffer != nullptr) {
				ID = uint_buff[0];
				version = uint_buff[1];
				surf_count = uint_buff[2];
				surf_count2 = uint_buff[3 + surf_count * 8];

				BVHlist.reserve(surf_count);
				BVHlist2.reserve(surf_count2);
			}


			read();

		}

	} while (0);
}

int cTrackBVH::read() {

	int ret = -1;
	cBVH_listEntry dummy;

//#define EX_TEST
#ifdef EX_TEST
		cTrackBVH ex_ob;
		cLeaves_helper lv_dummy;
		ex_ob.surf_count = surf_count;
		ex_ob.leaves_helper.leaves_size = surf_count;
#endif

	if (buffer != nullptr) {

		float * l_1 = (float*)(buffer + 12);

		for (unsigned int i = 0; i < surf_count; i++) {

			dummy.min.x = l_1[i * 8];
			dummy.min.z = l_1[i * 8 + 1];
			dummy.min.y = l_1[i * 8 + 2];
			dummy.max.x = l_1[i * 8 + 3];
			dummy.max.z = l_1[i * 8 + 4];
			dummy.max.y = l_1[i * 8 + 5];
			dummy.unkn = *(unsigned int*)(l_1 + i * 8 + 6);
			dummy.index = *(unsigned int*)(l_1 + i * 8 + 7);

#ifdef EX_TEST
			ex_ob.BVHlist.push_back(dummy);
#endif
			BVHlist.push_back(dummy);
		}

		float * l_2 = l_1 + surf_count * 8 + 1;
		for (unsigned int i = 0; i < surf_count2; i++) {

			dummy.min.x = l_2[i * 8];
			dummy.min.z = l_2[i * 8 + 1];
			dummy.min.y = l_2[i * 8 + 2];
			dummy.max.x = l_2[i * 8 + 3];
			dummy.max.z = l_2[i * 8 + 4];
			dummy.max.y = l_2[i * 8 + 5];
			dummy.index = *(unsigned int*)(l_2 + i *
				8 + 6); // Offset !!
			dummy.unkn = *(unsigned int*)(l_2 + i * 
				8 + 7); // counter
#ifdef EX_TEST
			if (dummy.unkn > 0)
			{
				
				lv_dummy.v_off = dummy.index;
				lv_dummy.size = dummy.unkn;
				ex_ob.leaves_helper.list.push_back(lv_dummy);
			}
				
#endif
			BVHlist2.push_back(dummy);
		}

		ret = 1;
	}

#ifdef EX_TEST
	ex_ob.build_tree();
	ex_ob.serialize_tree();
	ex_ob.write("ex_test_bvh.gen");
#endif

	return ret;
}

int cTrackBVH::write(string filename) {
	int ret = -1;
	do {
		ofstream ofs(filename, ios_base::binary, ios_base::trunc);
		if (!ofs.is_open())
			break;

//#define SWAP_LISTS
#ifdef SWAP_LISTS
		vector<cBVH_listEntry> tmp;
		tmp.insert(tmp.begin(), BVHlist.begin(), BVHlist.end());
		BVHlist.clear();
		BVHlist.insert(BVHlist.begin(), BVHlist2.begin(), BVHlist2.end());
		BVHlist2.clear();
		BVHlist2.insert(BVHlist2.begin(), tmp.begin(), tmp.end());
#endif        

		surf_count = BVHlist.size();
		//surf_count2 = BVHlist2.size();


		ofs.write((char*)&ID, sizeof(int));
		ofs.write((char*)&version, sizeof(int));
		ofs.write((char*)&surf_count, sizeof(int));


		for (unsigned int i = 0; i < surf_count; i++) {
			ofs.write((char*)&(BVHlist[i].min.x), sizeof(float));
			ofs.write((char*)&(BVHlist[i].min.z), sizeof(float));
			ofs.write((char*)&(BVHlist[i].min.y), sizeof(float));
			ofs.write((char*)&(BVHlist[i].max.x), sizeof(float));
			ofs.write((char*)&(BVHlist[i].max.z), sizeof(float));
			ofs.write((char*)&(BVHlist[i].max.y), sizeof(float));
			ofs.write((char*)&(BVHlist[i].unkn), sizeof(int));
			ofs.write((char*)&(BVHlist[i].index), sizeof(int));
		}

		ofs.write((char*)&surf_count2, sizeof(int));

		for (unsigned int i = 0; i < surf_count2; i++) {
			ofs.write((char*)&(BVHlist2[i].min.x), sizeof(float));
			ofs.write((char*)&(BVHlist2[i].min.z), sizeof(float));
			ofs.write((char*)&(BVHlist2[i].min.y), sizeof(float));
			ofs.write((char*)&(BVHlist2[i].max.x), sizeof(float));
			ofs.write((char*)&(BVHlist2[i].max.z), sizeof(float));
			ofs.write((char*)&(BVHlist2[i].max.y), sizeof(float));
			ofs.write((char*)&(BVHlist2[i].index), sizeof(int));
			ofs.write((char*)&(BVHlist2[i].unkn), sizeof(int));
		}

		ofs.close();


		ret = 1;

	} while (0);

	return ret;
}


int cTrackBVH::build_list_tree(vector<Primitive> *prPtr) {
	int ret = -1;
	primitivesPtr = prPtr;
	do {
		if (!BVHlist.size() || !BVHlist2.size())
			break;

		cCDB2_node dummy;
		AABB_tree.clear();
		
		tree<cCDB2_node>::iterator dbIt = AABB_tree.begin();
		tree<cCDB2_node>::iterator cd1 = AABB_tree.
		append_child(dbIt, dummy);
		first_el_flat = cd1;
		dbIt->isRoot = true;

		for (unsigned int i = 0; i < BVHlist.size(); i++) {
			CPY_AABB(cd1->bb, BVHlist[i], FO2_SCALE)
		    cd1->nmb_in_list = BVHlist[i].index;
			cd1->ukn_index = BVHlist[i].unkn;
			cd1 = AABB_tree.append_child(dbIt, 
				dummy);
		}

		tree_build = true;
		ret = 1;
	} while (0);

	return ret;
}


int cTrackBVH::read_tree(int omit_flag) {
	// old proto : DWORD *__stdcall sub_58B060(DWORD *some_Ob)
	int ret = -1;

	do {

		if (buffer == NULL || !surf_count || !surf_count2) {
			break;
		}

		// points @ beg of tree 
		const float *tree_Ptr = (float *)(buffer +
			16 + surf_count * 32);

		AABB_tree2.clear();
		tree_out.clear();
		cCDB2_node dummy;
		cTree_out_el to_dummy;
		tree<cCDB2_node>::iterator cd1, cd2;
		cTrav_stack_el tr_dummy;

		stack<cTrav_stack_el> trav_stack;


		tree<cCDB2_node>::iterator dbIt = first_el_tree =
			AABB_tree2.set_head(dummy);;


		// initial value in trac stack
		tr_dummy.child_off = (float*)tree_Ptr;
		tr_dummy.res_mask = 63; 
		trav_stack.push(tr_dummy);

		cLeaves_list_el lv_dummy;
		vector<cLeaves_list_el> leaves;
		leaves.reserve(500);
		BBox bb;


		float *result; // eax
		unsigned int last_res_mask; // ebx
		float *bvh_ptr; // esi
		unsigned int some_mask; // edx
		unsigned int res_mask; // ebp
		float *next_off_tmp;
		float *some_comp = some_comp_hard; // edi
		double scalar1, scalar2;
		unsigned int cnt_tmp; // eax
		cPoint3D bvh_p_min, bvh_p_max, 
			cmp1, cmp2;
		float offs = 0.0f;
		bool first_set = false;
		result = (float*)tree_Ptr; 
		// was: result = bvh_beg_Ptr;

		if (tree_Ptr) { // if ptr != NULL

			do {

				bvh_ptr = trav_stack.top().child_off;
				size_t got_off = (byte*)trav_stack.top().child_off
					- (byte*)tree_Ptr;
				last_res_mask = trav_stack.top().res_mask;
				if(trav_stack.size())
					trav_stack.pop();
				some_mask = 1;
				res_mask = 0;

				bvh_p_min.x = bvh_ptr[0];
				bvh_p_min.z = bvh_ptr[1];
				bvh_p_min.y = bvh_ptr[2];
				bvh_p_max.x = bvh_ptr[3];
				bvh_p_max.z = bvh_ptr[4];
				bvh_p_max.y = bvh_ptr[5];
				cmp1.x = some_comp[0];
				cmp1.y = some_comp[1];
				cmp1.z = some_comp[2];
				cmp2.x = some_comp[3];
				cmp2.y = some_comp[4];
				cmp2.z = some_comp[5];
				offs = some_comp[6];


				if (last_res_mask >= 1) {
					some_comp = some_comp_hard;
					// maybe like in cdb2 aabb of car
					do {

						if (some_mask & last_res_mask) { // if same bit is set

							scalar1 = cmp1.y * bvh_p_max.y +
								cmp1.z * bvh_p_max.z +
								cmp1.x * bvh_p_max.x;// ; res is like 1.8, 
					//must be smth sym
							scalar2 = cmp2.y * bvh_p_min.y +
								cmp2.z * bvh_p_min.z +
								cmp2.x * bvh_p_min.x +
								offs;
							// min from bvh, some max
							if (scalar2 < -scalar1 && omit_flag)
								goto OMIT;
							if (scalar2 <= scalar1)
								res_mask |= some_mask;
							// must be the true condition
						}
						// leftshift by 1
						some_mask *= 2;

						some_comp += 7;
						// maybe runs max 5 times, cuz at begin 63 == first 6 bits are set
					} while (some_mask <= last_res_mask);
				}

				cnt_tmp = *((unsigned int *)bvh_ptr + 7);      // unkn in tree
				size_t off_t = *((unsigned int *)bvh_ptr + 6);
				next_off_tmp = (float*)(off_t + (byte*)tree_Ptr);  // next offset, or if leaf some toher flags

				// cpy to tree
				cd1 = AABB_tree2.append_child(dbIt, dummy);
					
				BVH_GET_AABB(cd1->bb, bvh_ptr, FO2_SCALE);
				cd2 = AABB_tree2.append_child(dbIt, dummy);
				cd1->l = true; cd2->r = true;
				cd1->c_off = next_off_tmp;
				cd1->cnt = cnt_tmp;
				cd1->res_mask = res_mask;
				dbIt = cd1;

				if (cnt_tmp) { // unkn is a counter related to first list

					lv_dummy.child_off = next_off_tmp;
					lv_dummy.counter = cnt_tmp;
					lv_dummy.res_mask = res_mask;
					leaves.push_back(lv_dummy);

					cd1->isLeaf = true;
					dbIt++; // leave; goto next
				}
				else {

					// push 2 ( both children ?) 
					tr_dummy.child_off = next_off_tmp + 8;  // sibling node
					tr_dummy.res_mask = res_mask;
					trav_stack.push(tr_dummy);
					tr_dummy.child_off = next_off_tmp;
					trav_stack.push(tr_dummy);

				}
			OMIT:
				;
			} while (trav_stack.size() > 0 && dbIt != AABB_tree2.end());// until all nodes R parsed, is correct btw; was: (unsigned int *)&bvh_beg_Ptr

			//result = (float *)beg_some_buff;

			// we can also use here leave itereator instead of leaves std::vector
			unsigned int i = 0;
			if (leaves.size()) { // if something was written in v21
				//out_Ptr = (float **)unsigned int_8D8180;
				//v24 = some_Ob[92];
				do {

					next_off_tmp = leaves[i].child_off;
					last_res_mask = leaves[i].res_mask;
					unsigned int counter2 = leaves[i].counter;
 					i++;

					do {

						if (1) { // was : (1 << (*((unsigned int*)next_off_ptr + 6) & 0x1F)) 
							//& *(_unsigned int *)(v24 + 4 * (*((unsigned int *)next_off_ptr + 6) >> 5))

							some_mask = 1;
							res_mask = 0;
							if (last_res_mask >= 1) {

								some_comp = some_comp_hard;

								do {

									if (some_mask & last_res_mask) {

										scalar1 = some_comp[1] * next_off_tmp[5] + some_comp[2] 
										* next_off_tmp[4] + some_comp[0] * next_off_tmp[3];
										scalar2 = some_comp[4] * next_off_tmp[2] + some_comp[5]
										* next_off_tmp[1] + some_comp[3] * next_off_tmp[0] + 
											some_comp[6];

										if ( scalar2 < -scalar1 && omit_flag)
											goto OMIT2;              // won't be visible if near-> LODs ?
										if ( scalar2 <= scalar1 )
											res_mask |= some_mask;                 // will be visible
									}

									some_mask *= 2;
									some_comp += 7;
								} while (some_mask <= last_res_mask);

							}

							//out_tmp = *out_Ptr;
							//*out_tmp = next_off_ptr[7];         // counter
							//*((_unsigned int *)out_tmp + 1) = res_mask;
							//*out_Ptr = out_tmp + 2;
							to_dummy.counter = *((unsigned int*)next_off_tmp + 7); // counter
							to_dummy.res_mask = res_mask;
							tree_out.push_back(to_dummy);
							// this buff is accessed by 004C9DC0

						}
					OMIT2:
						next_off_tmp += 8;
						--counter2;

					} while (counter2);

					//result = (float *)somePtr1;                      // doesnt make much sense for now

				} while (i < leaves.size());          
			}
		}


		//return result;
		/*
		ofstream txtO("tree_out_test.txt", ios_base::trunc);

		for (unsigned int i = 0; i < tree_out.size(); i++) {
			txtO << "----------------------------\n" <<
				"counter: " << tree_out[i].counter << " res_mask: " <<
				tree_out[i].res_mask << "\n";
		}
		*/


		ret = 1;

	} while (0);

	return ret;
}
#ifdef BVH_TESTS
/*
int cTrackBVH::build_tree(unsigned leaves)
{
	int ret = -1;
	// creates dummy tree which has the only purpose to hold
	// the leaves
	bool init = true;
	do
	{
		cCDB2_node dummy;
		AABB_tree.clear();
		//AABB_tree.set_head(dummy);
		tree<cCDB2_node>::iterator dbIt = first_el_tree  = AABB_tree.set_head(dummy);
		tree<cCDB2_node>::iterator cd1, cd2;
		
		size_t half_leaves = leaves / 2;

		do {

			if(init)
			{
				cd1 = AABB_tree.append_child(dbIt, dummy);
				//first_el_tree = cd1;
				cd2 = AABB_tree.append_child(dbIt, dummy);
				dbIt = cd1;
				init = false;
 			}
			else
			{
				if(leaves > 1)
				{	
					cd1 = AABB_tree.append_child(dbIt, dummy);
					cd2 = AABB_tree.append_child(dbIt, dummy);
					if( leaves == 2)
					{
						cd2->isLeaf = true;
					}
					
					
				} else
				{
					cd1 = AABB_tree.append_child(dbIt, dummy);
				}
				

				cd1->isLeaf = true;
				leaves--;
				dbIt = cd1;

				dbIt++;

				if (leaves == half_leaves)
				{
					dbIt->isLeaf = true;
					leaves--;
					dbIt++;
				}
			}
			

		} while (dbIt != AABB_tree.end() && leaves);
		
	} while (0);

	return ret;
}
*/


// TODO: improve  
// builds AABB tree of the mesh
int cTrackBVH::build_tree() {

	unsigned int ret = 1;

	// start with z axis 
	byte axis = 2;

	unsigned int in_cnt = 0, out_cnt = 0, try_cnt = 0,
		max_split_fail = 0, tmp_nmbr = 0, depth_cnt = 0;

	cPoint3D smalles_vert, biggest_vert;
	unsigned remaining_fac = BVHlist.size();


	const unsigned max_polys_leaf = 20;


	// empty dummy 
	cCDB2_node dummy;

	// points to the inner lists 
	vector<unsigned int>::iterator tmp_it;

	AABB_tree.clear();
	leaves_helper.list.clear();
	leaves_helper.leaves_size = 0;

	AABB_tree.set_head(dummy);
	tree<cCDB2_node>::iterator cd1, cd2, parent_bak;
	tree<cCDB2_node>::iterator dbIt = first_el_tree = AABB_tree.set_head(dummy);

	// write table with center of aabbs 
	vector<aabb_with_surfIdx> aabbs_center;
	aabbs_center.reserve(remaining_fac);
	cPoint3D o_min, o_max, cd1_min, cd1_max,
		cd2_min, cd2_max;
	aabb_with_surfIdx f_center;
	f_center.surfIdx = 0;


	for (unsigned int i = 0; i < remaining_fac; i++)
	{
		Vector3D t_center;
		MIN_MAX_VERTS(i, BVHlist[i].min, smalles_vert, biggest_vert);
		MIN_MAX_VERTS(i, BVHlist[i].max, smalles_vert, biggest_vert);
		OBJ_CENTER(BVHlist[i].min, BVHlist[i].max, t_center);
		f_center.center = t_center;
		f_center.bb.min= BVHlist[i].min;
		f_center.bb.max = BVHlist[i].max;
		f_center.surfIdx = BVHlist[i].unkn;
		aabbs_center.push_back(f_center);
	}

	cBVH_listEntry bvh_l_dummy;
	BVHlist.clear();

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

	do {

		if (depth_cnt) {

			if(dbIt->isLeaf)
			{
				dbIt++;
				goto CONTINUE;
			}

			remaining_fac = dbIt->t_f.size();
			tmp_it = dbIt->t_f.begin();

			// copy the box from the node 
			CPY_R_AR(dbIt, tmp_lwh, tmp_center)

			// copy axis back from node 
			dbIt->t_axis = axis;

			// increment axis 
			axis == 2 ? axis = 0 : axis++;

		}


		if (remaining_fac > max_polys_leaf && !dbIt->isLeaf) {


			cd1 = AABB_tree.append_child(dbIt, dummy);
			cd2 = AABB_tree.append_child(dbIt, dummy);
			cd1->l = true; cd2->r = true;
			// parent backup
			parent_bak = dbIt;
			dbIt = cd1;
			goto SUBBOX;

		SUBBOX_AXIS_INC:

			// try only 12 times to split 
			if (try_cnt > 12) {

				if ((in_cnt > out_cnt ? in_cnt : out_cnt) > max_split_fail) {
					max_split_fail = (in_cnt > out_cnt ? in_cnt : out_cnt);
				}

				dbIt = parent_bak;
				AABB_tree.erase_children(parent_bak);

				dbIt->t_flag = 3;
				dbIt->isLeaf = true;

				// TODO : fix duplication .. 
				cLeaves_helper hlp_dummy;
				hlp_dummy.size = dbIt->t_f.size();
				hlp_dummy.v_off = BVHlist.size();

				leaves_helper.list.push_back(hlp_dummy);
				size_t init_size = BVHlist.size();
				for (unsigned i = 0; i < dbIt->t_f.size(); i++)
				{
					bvh_l_dummy.max = aabbs_center[dbIt->t_f[i]].bb.max;
					bvh_l_dummy.min = aabbs_center[dbIt->t_f[i]].bb.min;
						bvh_l_dummy.index = init_size + i;
					bvh_l_dummy.unkn = aabbs_center[dbIt->t_f[i]].surfIdx;
					BVHlist.push_back(bvh_l_dummy);
				}

				//goto WRITE_NODE;
				goto CONTINUE;
			}

			// increment axis 
			axis == 2 ? axis = 0 : axis++;
			try_cnt++;

		SUBBOX:

			// new dimensions
			tmp_lwh[axis] *= .5f;
			tmp_center[axis] -= tmp_lwh[axis];


			cd1->t_f.clear();
			cd2->t_f.clear();

			// calculate min and max of aabb 
			CALC_MIN_MAX(o_min, o_max, tmp_lwh, tmp_center)

				for (unsigned int i = in_cnt = out_cnt = 0;
					i < remaining_fac; i++) {

				// first, parse all boxes 
				if (depth_cnt)
					tmp_nmbr = tmp_it[i];
				else
					tmp_nmbr = i;


				//
				if (VERT_IN_AABB(o_min, o_max, aabbs_center[tmp_nmbr].center)) {
					cd1->t_f.push_back(tmp_nmbr);
					// set min and max to be able to resize the boxes 
					MIN_MAX_VERTS(in_cnt, aabbs_center[tmp_nmbr].bb.min, cd1_min, cd1_max)
						MIN_MAX_VERTS(in_cnt, aabbs_center[tmp_nmbr].bb.max, cd1_min, cd1_max)
						in_cnt++;

				}
				else {
					cd2->t_f.push_back(tmp_nmbr);
					MIN_MAX_VERTS(out_cnt, aabbs_center[tmp_nmbr].bb.min, cd2_min, cd2_max)
						MIN_MAX_VERTS(out_cnt, aabbs_center[tmp_nmbr].bb.max, cd2_min, cd2_max)
						out_cnt++;
				}

			}

			// if split was not successfull
			if (!in_cnt || !out_cnt)
				goto SPLIT_TRY;

		WRITE_NODE:
			try_cnt = 0;
			// resize the box, calc the new one
			CALC_AABB(cd1_min, cd1_max, tmp_lwh, tmp_center)
				CALC_AABB(cd2_min, cd2_max, tmp_lwh2, tmp_center2)
				// copy to node
				CPY_AR(cd1, tmp_lwh, tmp_center)
				CPY_AR(cd2, tmp_lwh2, tmp_center2)
				// copy to node member bb
				CALC_MIN_MAX(cd1->bb.min, cd1->bb.max,
					tmp_lwh, tmp_center);

				CALC_MIN_MAX(cd2->bb.min, cd2->bb.max,
				tmp_lwh2, tmp_center2);

				goto NEXT;

		SPLIT_TRY:

			if ((in_cnt && (in_cnt < max_polys_leaf)) || (out_cnt && (out_cnt < max_polys_leaf)))
				throw logic_error("error: poly_cnt && poly_cnt < max_polys_leaf");

			// if all aabbs were in one subbox, go again and split the subbox itself another time
			if (!out_cnt) {
				goto SUBBOX_AXIS_INC;
			}

			// go to box in sibling node and split it
			if (!in_cnt) {
				tmp_center[axis] += 2 * tmp_lwh[axis];
				goto SUBBOX_AXIS_INC;
			}

		NEXT:
			/*if (in_cnt <= max_polys_leaf) {

				// goto child2
				if (out_cnt > max_polys_leaf ) {

					++dbIt;
				}
				// goto to next 
				else {

					// goto next
					advance(dbIt, 2);
				}
			}*/

			// set the flags 
			vector<tree<cCDB2_node>::iterator *> leav_nodes;
			if (in_cnt <= max_polys_leaf) {
				leav_nodes.push_back(&cd1);
			}
			if (out_cnt <= max_polys_leaf) {		
				leav_nodes.push_back(&cd2);
			}


			// copy to leaves list, only if we have leaf node 
			for (size_t j = 0; j < leav_nodes.size(); j++)
			{
				if (leav_nodes[j] != nullptr)
				{
					tree<cCDB2_node>::iterator &lv_node = *leav_nodes[j];
					// set flags
					lv_node->t_flag = 3;
					lv_node->isLeaf = true;

					cLeaves_helper hlp_dummy;
					hlp_dummy.size = lv_node->t_f.size();
					hlp_dummy.v_off = BVHlist.size();
					
					leaves_helper.list.push_back(hlp_dummy);
					size_t init_size = BVHlist.size();
					for (unsigned i = 0; i < lv_node->t_f.size(); i++)
					{
						CPY_PNT(bvh_l_dummy.max, aabbs_center[lv_node->t_f[i]].bb.max)
							CPY_PNT(bvh_l_dummy.min, aabbs_center[lv_node->t_f[i]].bb.min)
							bvh_l_dummy.index = init_size + i;
						bvh_l_dummy.unkn = aabbs_center[lv_node->t_f[i]].surfIdx;
						BVHlist.push_back(bvh_l_dummy);
					}

				}
			}
			
			

		}
		else {
			dbIt->t_flag = 3;
			dbIt->isLeaf = true;

			// TODO : fix duplication .. 
			cLeaves_helper hlp_dummy;
			hlp_dummy.size = dbIt->t_f.size();
			hlp_dummy.v_off = BVHlist.size();
			
			leaves_helper.list.push_back(hlp_dummy);
			size_t init_size = BVHlist.size();
			for (unsigned i = 0; i < dbIt->t_f.size(); i++)
			{
				CPY_PNT(bvh_l_dummy.max, aabbs_center[dbIt->t_f[i]].bb.max)
					CPY_PNT(bvh_l_dummy.min, aabbs_center[dbIt->t_f[i]].bb.min)
					bvh_l_dummy.index = init_size + i;
				bvh_l_dummy.unkn = aabbs_center[dbIt->t_f[i]].surfIdx;
				BVHlist.push_back(bvh_l_dummy);
			}


			++dbIt;
		}
CONTINUE:

		depth_cnt++;

	} while (dbIt != AABB_tree.end());


	return ret;

}



// serialize the tree to be able to write it in the file
unsigned int cTrackBVH::serialize_tree() {

	const size_t NODE_SIZE = 32;

	// reverse true just because the first nodees's child
	// is just after his parent , so offset is +1, not +2
	// and we want to minimize some code duplication
	bool reverse = true;
	tree<cCDB2_node>::iterator dbIt = first_el_tree;
	
	unsigned hlp_cnt = 0,
	// how far it has been written already
	written_off = 0, cnt1 = 0,
	// next offset
	nxt_off = 0,
	// current absolute offset
	abs_offs = 0;

	// traversal stack
	stack<unsigned> trav_stack;


	cBVH_listEntry dummy;
	// some dummy box
	dummy.unkn = -1;
	dummy.index = -1;

	BVHlist2.clear();
	surf_count2 = AABB_tree.size() - 1;
	BVHlist2.reserve(surf_count2);

	for (size_t i = 0; i < surf_count2; i++)
		BVHlist2.push_back(dummy);

	for(unsigned i = leaves_helper.leaves_size = 0; 
		i < leaves_helper.list.size(); i++)
	{
		leaves_helper.leaves_size +=
			leaves_helper.list[i].size;
	}
	
	do {

		if (dbIt == AABB_tree.end()) 
			break;
		

			// write leaves and faces in part 2 
			if (dbIt->isLeaf) {
				
				
				//assert(abs_offs < surf_count2);
				
				size_t lv_size = leaves_helper.list[hlp_cnt].size;
				size_t voff = leaves_helper.list[hlp_cnt].v_off * NODE_SIZE;
				if (!lv_size)
				{
					lv_size = 1;
					voff = 1;
				}
				BVHlist2[abs_offs].index = voff;
				BVHlist2[abs_offs].unkn = lv_size;
			
				hlp_cnt++;


				if (abs_offs >= written_off)
					written_off += 1;

				reverse = true;

				 
				if(trav_stack.size()) {
					abs_offs = trav_stack.top();
					trav_stack.pop();
				} else 
					break;
				

			}
			// write normal node
			else {

				if(reverse)
				{
					// if we are on reverse walk
					if(abs_offs < written_off)
					{
						nxt_off = written_off;
					}  else
					{
						
						nxt_off = abs_offs + 1;
					}

				} else
				{
					// normal, trav, after we are going depth first, we go to first (left) child, one node
					/// is left free to write on the reverse walk the second(right) node
					nxt_off = abs_offs + 2;
				}

				BVHlist2[abs_offs].unkn = 0;
				BVHlist2[abs_offs].index = nxt_off * NODE_SIZE;
			
				// first time it writes only one elem
				if( !cnt1)
				{
					written_off = 1;
					cnt1++;
				} else
				{
					if (abs_offs >= written_off) {
						written_off += 2;
					}
				}
				

				abs_offs = nxt_off;

				reverse = false;
				trav_stack.push(nxt_off + 1);

			}
		
			dbIt++;
		
		
	} while (1);

	return 1 ;

}
#endif // ifdef BVH_TESTS

