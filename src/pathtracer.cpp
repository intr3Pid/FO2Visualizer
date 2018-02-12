#include "pathtracer.h"
#include "bsdf.h"
#include "ray.h"

#include "CMU462/vector3D.h"
#include "CMU462/matrix3x3.h"
#include "CMU462/lodepng.h"

#include "GL/glew.h"

using namespace StaticScene;
using namespace Collada;

using Collada::CameraInfo;
using Collada::LightInfo;
using Collada::MaterialInfo;
using Collada::PolymeshInfo;
using Collada::SceneInfo;
using Collada::SphereInfo;

using std::min;
using std::max;

namespace CMU462 {

//#define ENABLE_RAY_LOGGING 1

	PathTracer::PathTracer(AppConfig config): other_file_type(0), mode(), num_tiles_w(0)
	{
		state = INIT,
			this->ns_aa = config.pathtracer_ns_aa;
		this->max_ray_depth = config.pathtracer_max_ray_depth;
		this->ns_area_light = config.pathtracer_ns_area_light;
		this->ns_diff = config.pathtracer_ns_diff;
		this->ns_glsy = config.pathtracer_ns_glsy;
		this->ns_refr = config.pathtracer_ns_refr;

		if (config.pathtracer_envmap)
		{
			this->envLight = new EnvironmentLight(config.pathtracer_envmap);
		}
		else
		{
			this->envLight = nullptr;
		}

		cdb2Ptr = nullptr; // dont't delete ! ->deleted in main
		node_info = "";
		scene = nullptr;
		camera = nullptr;

		gridSampler = new UniformGridSampler2D();
		hemisphereSampler = new UniformHemisphereSampler3D();

		show_rays = true;

		set_up = false;
		show_pt_info = true;
		tree_mode = true;
		draw_mesh = true;
		draw_non_hl_aabbs = true;

		imageTileSize = 32;
		numWorkerThreads = config.pathtracer_num_threads;
		workerThreads.resize(numWorkerThreads);

		tm_gamma = 2.2f;
		tm_level = 1.0f;
		tm_key = 0.18;
		tm_wht = 5.0f;

		cnode[0] = .5f;
		cnode[1] = .5f;
		cnode[2] = .5f;
		cnode[3] = .25f;
		cnode_hl[0] = 1.f;
		cnode_hl[1] = .25f;
		cnode_hl[2] = .0f;
		cnode_hl[3] = 1.f;
		cnode_hl_child[0] = 1.f;
		cnode_hl_child[1] = 1.f;
		cnode_hl_child[2] = 1.f;
		cnode_hl_child[3] = .6f;
		s_blue[0] = 0.1725f;
		s_blue[1] = 0.2901f;
		s_blue[2] = 0.3882f;
		s_blue[3] = .6f;

		cprim_hl_leaves[0] = 0.95f;
		cprim_hl_leaves[1] = 0.95f;
		cprim_hl_leaves[2] = 0.254f;
		cprim_hl_leaves[3] = 1.f;
		cprim_hl_left[0] = 0.8f;
		cprim_hl_left[1] = 0.0f;
		cprim_hl_left[2] = 0.0f;
		cprim_hl_left[3] = 1.f;
		cprim_hl_right[0] = 0.0f;
		cprim_hl_right[1] = 0.8f;
		cprim_hl_right[2] = 0.0f;
		cprim_hl_right[3] = 1.f;
		cprim_hl_edges[0] = .3f;
		cprim_hl_edges[1] = .3f;
		cprim_hl_edges[2] = .3f;
		cprim_hl_edges[3] = 1.f;

		ctext[0] = 1.f;
		ctext[1] = 1.f;
		ctext[2] = 1.f;
		ctext[3] = 1.f;


	} // end PathTracer constructor

	PathTracer::~PathTracer() {

  delete gridSampler;
  delete hemisphereSampler;

 // primitives must not be freed, see object.cpp

}



void PathTracer::init() {


	// init all HUDs
	textManager.init(use_hdpi);
	pt_info.init(use_hdpi);

	// Setup all the basic internal state to default values,
	// as well as some basic OpenGL state (like depth testing
	// and lighting).

	// Set the integer bit vector representing which keys are down.
	leftDown = false;
	rightDown = false;
	middleDown = false;

	show_coordinates = true;
	show_hud = true;

	// Lighting needs to be explicitly enabled.
	glEnable(GL_LIGHTING);

	// Enable anti-aliasing and circular points.
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

	// Initialize styles (colors, line widths, etc.) that will be used
	// to draw different types of mesh elements in various situations.
	initialize_style();

	mode = EDIT_MODE;
	scene = nullptr;

	// Make a dummy camera so resize() doesn't crash before the scene has been
	// loaded.
	// NOTE: there's a chicken-and-egg problem here, because loadScene
	// requires init, and init requires init_camera (which is only called by
	// loadScene).
	screenW = screenH = 600; // Default value
	CameraInfo cameraInfo;
	cameraInfo.hFov = 50;
	cameraInfo.vFov = 35;
	cameraInfo.nClip = 0.01;
	cameraInfo.fClip = 100;
	appCamera.configure(cameraInfo, screenW, screenH);

	init_bars();
}

void TW_CALL PathTracer::SetMyStdStringCB(const void *value, void *clientData)
{
	// Set: copy the value of s3 from AntTweakBar
	const string *srcPtr = static_cast<const string *>(value);
	string *destPtr = static_cast<string *>(clientData);
	*destPtr = *srcPtr;
}
void TW_CALL PathTracer::GetMyStdStringCB(void *value, void * clientData)
{
	// Get: copy the value of s3 to AntTweakBar
	string *destPtr = static_cast<string *>(value);
	string *srcPtr = static_cast<string *>(clientData);
	TwCopyStdStringToLibrary(*destPtr, *srcPtr);
}


void PathTracer::set_node_nmb_bvh() {
	tree<cCDB2_node>::iterator it = nullptr;
	unsigned index = -1, unk = -1;
	
	if (cdb2selectionHistory.size())
	{
		it = cdb2selectionHistory.top();
		if (it != nullptr)
		{
			index = it->nmb_in_list;
			unk = it->ukn_index;
		}
	}

		node_info = "  BVH node index: " + to_string(index) +
		", unk: " + to_string(unk);
	
	
}

void PathTracer::set_node_nmb_cdb2() {
	tree<cCDB2_node>::iterator it = nullptr;
	bool l = false, r = false, isLeaf = false;
	size_t poly_size = -1;
	

	if (cdb2selectionHistory.size())
	{
		it = cdb2selectionHistory.top();
		if (it != nullptr)
		{

			isLeaf = it->isLeaf;
			l = it->l;
			r = it->r;
			poly_size = it->t_f.size();
		}

		node_info = "Node:";
		node_info += ((!l && !r) ? "root":(l ? +"l" : "r"));
		node_info += " is Leaf:";
		node_info += isLeaf?"yes":"no";
		node_info += " Poly num:" + to_string(poly_size);

	}
	else
	{
		node_info = "Invalid Node";
	}
	

}
void TW_CALL Toggle_tree(void * clientData)
{
	PathTracer *tptr = reinterpret_cast<PathTracer*>(clientData);
	if (tptr != nullptr)
	{


		tptr->tree_mode = !tptr->tree_mode;
		tptr->update_stack();
	
	}

}


void PathTracer::update_stack() {

		
	while (cdb2selectionHistory.size() > 0)
		cdb2selectionHistory.pop();

	if (state != READY && state != INIT)
	{
		// initial visualization //
		if (tree_mode) {
			cdb2selectionHistory.push(bvhPtr->first_el_tree);
			//set_node_nmb(bvhPtr->first_el_tree->nmb_in_list,
				//bvhPtr->first_el_tree->ukn_index);
		}
		else {
			cdb2selectionHistory.push(bvhPtr->first_el_flat);
			set_node_nmb_bvh();
		}
	}
	
	
}


void PathTracer::init_bars() {

	string name = "Settings";
	TwBar * mainBar = TwNewBar(name.c_str());

	TwDefine(" Settings label='Settings TweakBar' refresh=0.5 position='16 16' size='260 320' alpha=0");

	TwAddVarRW(mainBar, "Node color", TW_TYPE_COLOR4F, cnode,
		" group='Colors' help='Change the Node color .' "); 
	TwAddVarRW(mainBar, "hl Node color", TW_TYPE_COLOR4F, cnode_hl,
		" group='Colors' help='Change the highlighted Node color.' ");
	TwAddVarRW(mainBar, "Child color", TW_TYPE_COLOR4F, cnode_hl_child,
		" group='Colors' help='Change the child color .' ");  
	TwAddVarRW(mainBar, "Leaves color", TW_TYPE_COLOR4F, cprim_hl_leaves,
		" group='Colors' help='Change the leaves color.' ");
	TwAddVarRW(mainBar, "Edges color", TW_TYPE_COLOR4F, cprim_hl_edges,
		" group='Colors' help='Change the edges color.' ");
	TwAddVarRW(mainBar, "Faces color", TW_TYPE_COLOR4F, s_blue,
		" group='Colors' help='Change the faces color.' ");
	TwAddVarRW(mainBar, "Text color", TW_TYPE_COLOR4F, ctext,
		" group='Colors' help='Change the display text color.' ");
	TwAddVarRW(mainBar, "Show Coordinates", TW_TYPE_BOOLCPP, &show_coordinates,
		" group='Display' key=w help='Toggle wireframe display mode.' ");
	TwAddVarRW(mainBar, "Draw Mesh", TW_TYPE_BOOLCPP, &draw_mesh,
		" group='Display' key=d help='Toggle mesh drwaing' ");
	TwAddVarRW(mainBar, "Draw non highlighted AABBs", TW_TYPE_BOOLCPP, &draw_non_hl_aabbs,
		" group='Display' key=q help='Toggle non highlighted AABBs drwaing' ");



	if (other_file_type == FO2_BVH_FILETYPE)
	{
		TwAddButton(mainBar, "Toggle tree", Toggle_tree, this,
			" group='Display' label='Tree?' help='tree?' ");
	}

	const char * lst = TwGetLastError();
		if (lst != nullptr ) {
			cout << "Tw last Error : " << lst;
		}

}

void PathTracer::set_scene(Scene *scene, cTrackCollision * pcdb2, cTrackBVH * pbvh, int other_fType

) {

  if (state != INIT) {
    return;
  }


  cdb2Ptr = pcdb2; /* set object pointer*/
  bvhPtr = pbvh; /*set other ... */
  this->other_fType = other_fType;


  if (this->scene != nullptr) {

    delete this->scene;
	this->scene = nullptr;

	if (cdb2selectionHistory.size())
		cdb2selectionHistory.pop();

  }

  if (this->envLight != nullptr) {
    scene->lights.push_back(this->envLight);
  }

  this->scene = scene;

  if (has_valid_configuration()) {
    state = READY;
  }
}

void PathTracer::set_camera(Camera *camera) {

  if (state != INIT) {
    return;
  }

  this->camera = camera;
  if (has_valid_configuration()) {
    state = READY;
  }

}

void PathTracer::set_frame_size(size_t width, size_t height) {
  if (state != INIT && state != READY) {
    stop();
  }
  sampleBuffer.resize(width, height);
  frameBuffer.resize(width, height);
  if (has_valid_configuration()) {
    state = READY;
  }
}

bool PathTracer::has_valid_configuration() {
  return scene && camera && gridSampler && hemisphereSampler &&
         (!sampleBuffer.is_empty());
}

void PathTracer::update_screen() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      visualize_accel();
      break;
    case RENDERING:
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      break;
    case DONE:
        //sampleBuffer.tonemap(frameBuffer, tm_gamma, tm_level, tm_key, tm_wht);
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      break;
  }
}

void PathTracer::stop() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
	  while (cdb2selectionHistory.size() > 1) {
		cdb2selectionHistory.pop();
	  }
      state = READY;
      break;
    case RENDERING:
      continueRaytracing = false;
    case DONE:
      for (int i=0; i<numWorkerThreads; i++) {
            workerThreads[i]->join();
            delete workerThreads[i];
        }
      state = READY;
      break;
  }
}

void PathTracer::clear() {

	if (state != READY) 
		return;

	if(cdb2selectionHistory.size())
		cdb2selectionHistory.pop();

	state = INIT;
}

void PathTracer::start_visualizing() {
  if (state != READY) {
    return;
  }
  state = VISUALIZE;
}


void PathTracer::build_accel() {

	// collect primitives, assumes they don't change
	if (!primitives.size() || !set_up) {
		primitives.clear();

		fprintf(stdout, "[PathTracer] Collecting primitives... "); fflush(stdout);
		timer.start();

		for (SceneObject *obj : scene->objects) {
			const vector<Primitive*> *ob_primes = obj->get_primitives();
			size_t obs_size = (*ob_primes).size();
			primitives.reserve(primitives.size() + obs_size);
			for(unsigned int i = 0; i < obs_size; i++)
				primitives.push_back((*ob_primes)[i]);
		}

		timer.stop();

		fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());
	}

	if (other_fType == FO2_CDB2_FILETYPE) {
		if (!cdb2Ptr->tree_build) {
			// build FO2 AABB tree //
			if (!cdb2Ptr->tree_build) {
				fprintf(stdout, "[PathTracer] Reading FlatOut2 AABB tree... "); fflush(stdout);
				timer.start();
				cdb2Ptr->read_tree(&primitives);
				timer.stop();
				fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

			}	
		}
		// initial visualization //
		cdb2selectionHistory.push(cdb2Ptr->tree_head);
	}
	else if (other_fType == FO2_BVH_FILETYPE) {
		if (!bvhPtr->tree_build)
		{
			fprintf(stdout, "[PathTracer] Building FlatOut2 BVH List dummy tree... "); fflush(stdout);
			timer.start();
			bvhPtr->build_list_tree(nullptr);
			timer.stop();
			fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());
			
			fprintf(stdout, "[PathTracer] Reading FlatOut2 BVH second List tree... "); fflush(stdout);
			timer.start();
			bvhPtr->read_tree(0);
			timer.stop();
			fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());
	


		}
		// initial visualization //
		if(tree_mode) {
			cdb2selectionHistory.push(bvhPtr->first_el_tree);
			set_node_nmb_bvh();
		} else {
			cdb2selectionHistory.push(bvhPtr->first_el_flat);
			set_node_nmb_bvh();
		}
		
			
	}
}

void PathTracer::log_ray_miss(const Ray& r) {
    rayLog.push_back(LoggedRay(r, -1.0));
}

void PathTracer::log_ray_hit(const Ray& r, double hit_t) {
    rayLog.push_back(LoggedRay(r, hit_t));
}

void PathTracer::visualize_accel() const {

  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);
  glEnable(GL_DEPTH_TEST);

  do {

	  if (!cdb2selectionHistory.size())
		  break; // can't continue 

	  tree<cCDB2_node>::iterator top_sel = cdb2selectionHistory.top(),
	  lN, rN, trav, end_trav;
	
	  if (top_sel == nullptr)
		  break;

	  // render solid geometry (with depth offset)
	  glPolygonOffset(1.0, 1.0);
	  glEnable(GL_POLYGON_OFFSET_FILL);


	  lN = cdb2Ptr->AABB_tree.begin(top_sel); /* ptr to left child node*/

	  if(lN == nullptr)
	  {
		  break;
		  // some error
	  }
	  rN = cdb2Ptr->AABB_tree.next_sibling(lN);

	  if (rN == nullptr)
	  {
		  break;
		  // some error
	  }

	  if (draw_mesh)
	  {
		  // draw faces of triangles which are in aabb of node with color 
		  if (other_fType == FO2_CDB2_FILETYPE) {


			  if (top_sel->isLeaf) {
				  for (size_t i = 0; i < top_sel->t_f.size(); ++i) {
					  primitives[top_sel->t_f[i]]->draw(TO_C_COLOR(cprim_hl_leaves));
				  }
			  }
			  else {
				  for (size_t i = 0; i < top_sel->t_f.size(); ++i) {
					  primitives[top_sel->t_f[i]]->draw(TO_C_COLOR(cprim_hl_left));
				  }
			  }
		  }
		  else if (other_fType == FO2_BVH_FILETYPE) {
			  for (unsigned int i = 0; i < primitives.size(); i++)
				  primitives[i]->draw(TO_C_COLOR(s_blue));

		  }


		  glDisable(GL_POLYGON_OFFSET_FILL);

		  // draw edges if triangles with color(only the ones within the aabb)
		  if (other_fType == FO2_CDB2_FILETYPE) {
			  // draw geometry outline
			  for (size_t i = 0; i < top_sel->t_f.size(); ++i) {
				  primitives[top_sel->t_f[i]]->drawOutline(TO_C_COLOR(cprim_hl_edges));
			  }
		  }
		  else if (other_fType == FO2_BVH_FILETYPE) {
			  for (unsigned int i = 0; i < primitives.size(); i++)
				  primitives[i]->drawOutline(TO_C_COLOR(cprim_hl_edges));
		  }
	  }
	 

	  if (draw_non_hl_aabbs)
	  {
		  // keep depth buffer check enabled so that mesh occluded bboxes, but
		  // disable depth write so that bboxes don't occlude each other.
		  glDepthMask(GL_FALSE);

		  // draw not higlighted aabbs
		  if (other_fType == FO2_CDB2_FILETYPE) {
			  trav = cdb2Ptr->AABB_tree.begin();
		  }
		  else if (other_fType == FO2_BVH_FILETYPE) {
			  if (tree_mode)
			  {
				  trav = bvhPtr->first_el_tree;
				  end_trav = bvhPtr->AABB_tree.end();
			  }
			  else
			  {
				  trav = bvhPtr->first_el_flat;
				  end_trav = nullptr;
			  }

		  }
	  	
	  	for (; trav != end_trav && trav != nullptr; ++trav)
		  trav->bb.draw(TO_C_COLOR(cnode));
	  }
	
	  
	  
	  

	  // draw selected aabbs
	  if (tree_mode && !top_sel->isLeaf) {
		  glLineWidth(3.f);
		  top_sel->bb.draw(TO_C_COLOR(cnode_hl));
	  }
	  else {
		  glLineWidth(3.f);
		  top_sel->bb.draw(TO_C_COLOR(cnode_hl));
	  }


  } while (0);

  glDepthMask(GL_TRUE);
  glPopAttrib();

}

// handles keyborad key press event in render mode
void PathTracer::key_press(int key) {

	tree<cCDB2_node>::iterator tree_it, head_it;
	if (cdb2selectionHistory.size())
		tree_it = cdb2selectionHistory.top();

	tree<cCDB2_node>::iterator c1;

	tree<cCDB2_node> &tree = other_fType == FO2_CDB2_FILETYPE ? cdb2Ptr->AABB_tree:
		bvhPtr->AABB_tree2;


	head_it = other_fType == FO2_CDB2_FILETYPE ? cdb2Ptr->tree_head :
		bvhPtr->first_el_tree;

	if(other_fType == 1)
		assert(tree_mode);

	switch (key) {

	case ']':
		ns_aa *= 2;
		printf("Samples per pixel changed to %lu\n", ns_aa);
		//tm_key = clamp(tm_key + 0.02f, 0.0f, 1.0f);
		break;

	case '[':
		//tm_key = clamp(tm_key - 0.02f, 0.0f, 1.0f);
		ns_aa /= 2;
		if (ns_aa < 1) ns_aa = 1;
		printf("Samples per pixel changed to %lu\n", ns_aa);
		break;

	case KEYBOARD_UP:
		if (tree_mode) {
			if (tree_it != head_it) {
				if(cdb2selectionHistory.size())
					cdb2selectionHistory.pop();
			}

			if (other_fType == FO2_CDB2_FILETYPE)
			{
				set_node_nmb_cdb2();
			}

		}
		break;

	case KEYBOARD_LEFT:
		if (tree_mode) {
			if (!tree_it->isLeaf)
			{
				c1 = tree.begin(tree_it);
				if(c1 != nullptr)
				{
					cdb2selectionHistory.push(c1);
				}
				
			}	
			

			if (other_fType == FO2_CDB2_FILETYPE)
			{
				set_node_nmb_cdb2();
			}
		}
		else {			
			tree_it--;
				if (tree_it.node != nullptr) {
					// TODO:
					// ugly fix for some bug, that there are at the beginning pft he tree
					// some unexpected nodes; they aren't initialized, so this works
					if (tree_it->bb.max.x < 10e35 && tree_it->bb.max.x > -10e35)
					{
						if (cdb2selectionHistory.size())
							cdb2selectionHistory.pop();
						cdb2selectionHistory.push(tree_it);
						set_node_nmb_bvh();
					}
					else
					{
						tree_it++;
					}

				}
		}
		break;

	case KEYBOARD_RIGHT:

		if (tree_mode) {
			if (!tree_it->isLeaf)
			{
				c1 = tree.begin(tree_it);
				if(c1 != nullptr)
				{
					c1 = tree.next_sibling(c1);
					if(c1 != nullptr)
					{
						cdb2selectionHistory.push(c1);
					}
					
				} else
				{
					// some error
				}
				if(other_fType == FO2_CDB2_FILETYPE)
				{
					set_node_nmb_cdb2();
				}
				
				//cdb2selectionHistory.push(tree.next_sibling(tree_it));
			}		
		}
		else {
			tree_it++;
			if (tree_it.node != nullptr) {
				// TODO:
				// ugly fix for some bug, that there are at the beginning pft he tree
				// some unexpected nodes; they aren't initialized, so this works
				if (tree_it->bb.max.x < 10e35 && tree_it->bb.max.x > -10e35)
				{
					if (cdb2selectionHistory.size())
						cdb2selectionHistory.pop();
					cdb2selectionHistory.push(tree_it);
					set_node_nmb_bvh();
				}
				else
				{
					tree_it--;
				}

			}

		}
		break;

	case 'a': case 'A':
		show_rays = !show_rays;
	default:
		return;
	}
}


void PathTracer::increase_area_light_sample_count() {
  ns_area_light *= 2;
  fprintf(stdout, "[PathTracer] Area light sample count increased to %zu!\n", ns_area_light);
}

void PathTracer::decrease_area_light_sample_count() {
  if (ns_area_light > 1) ns_area_light /= 2;
  fprintf(stdout, "[PathTracer] Area light sample count decreased to %zu!\n", ns_area_light);
}

void PathTracer::save_image() {

  if (state != DONE) return;

  time_t rawtime;
  time (&rawtime);

  string filename = "Screen Shot ";
  filename += string(ctime(&rawtime));
  filename.erase(filename.end() - 1);
  filename += string(".png");

  uint32_t* frame = &frameBuffer.data[0];
  size_t w = frameBuffer.w;
  size_t h = frameBuffer.h;
  uint32_t* frame_out = new uint32_t[w * h];
  for(size_t i = 0; i < h; ++i) {
    memcpy(frame_out + i * w, frame + (h - i - 1) * w, 4 * w);
  }

  fprintf(stderr, "[PathTracer] Saving to file: %s... ", filename.c_str());
  lodepng::encode(filename, (unsigned char*) frame_out, w, h);
  fprintf(stderr, "Done!\n");
}


float * PathTracer::get_color(unsigned int ID) {
	float *ret = nullptr;
	switch (ID) {
	case ID_CNODE: ret = cnode; break;
	case ID_CNODEHL: ret = cnode_hl; break;
	case ID_CNODEHLCHILD: ret = cnode_hl_child; break;
	case ID_CSBLUE: ret = s_blue; break;
	case ID_CHLLEAVES: ret = cprim_hl_leaves; break;
	case ID_CHLLEFT: ret = cprim_hl_left; break;
	case ID_CHLRIGHT: ret = cprim_hl_right; break;
	case ID_CHLEDGES: ret = cprim_hl_edges; break;
	case ID_CTEXT: ret = ctext; break;
	default:break;
	}	
	return ret;
}


void PathTracer::initialize_style() {
	// Colors.
	defaultStyle.halfedgeColor = Color(0.3, 0.3, 0.3, 1.0);
	hoverStyle.halfedgeColor = Color(0.6, 0.6, 0.6, 1.0);
	selectStyle.halfedgeColor = Color(1.0, 1.0, 1.0, 1.0);

	defaultStyle.faceColor = Color(0.3, 0.3, 0.3, 1.0);
	hoverStyle.faceColor = Color(0.6, 0.6, 0.6, 1.0);
	selectStyle.faceColor = Color(1.0, 1.0, 1.0, 1.0);

	defaultStyle.edgeColor = Color(0.3, 0.3, 0.3, 1.0);
	hoverStyle.edgeColor = Color(0.6, 0.6, 0.6, 1.0);
	selectStyle.edgeColor = Color(1.0, 1.0, 1.0, 1.0);

	defaultStyle.vertexColor = Color(0.3, 0.3, 0.3, 1.0);
	hoverStyle.vertexColor = Color(0.6, 0.6, 0.6, 1.0);
	selectStyle.vertexColor = Color(1.0, 1.0, 1.0, 1.0);

	// Primitive sizes.
	defaultStyle.strokeWidth = 1.0;
	hoverStyle.strokeWidth = 2.0;
	selectStyle.strokeWidth = 2.0;

	defaultStyle.vertexRadius = 4.0;
	hoverStyle.vertexRadius = 8.0;
	selectStyle.vertexRadius = 8.0;
}

void PathTracer::update_style() {

	float view_distance = (appCamera.position() - appCamera.view_point()).norm();
	float scale_factor = canonical_view_distance / view_distance;

	hoverStyle.strokeWidth = 2.0 * scale_factor;
	selectStyle.strokeWidth = 2.0 * scale_factor;

	hoverStyle.vertexRadius = 8.0 * scale_factor;
	selectStyle.vertexRadius = 8.0 * scale_factor;
}

void PathTracer::render() {
	update_gl_camera();
	switch (mode) {
	case EDIT_MODE:
		if (show_coordinates) draw_coordinates();
		appScene->render_in_opengl();
		if (show_hud) 
			draw_hud();
		break;
	case VISUALIZE_MODE:
		if (show_coordinates) 
			draw_coordinates();
		
		if (show_pt_info)
			draw_pt_info();
		
		
	case RENDER_MODE:
		update_screen();
		
		if (show_pt_info)
			draw_pt_info();
		
		break;
	}
		
}

void PathTracer::update_gl_camera() {

	// Call resize() every time we draw, since it doesn't seem
	// to get called by the Viewer upon initial window creation
	// (this should probably be fixed!).
	GLint view[4];
	glGetIntegerv(GL_VIEWPORT, view);
	if (view[2] != screenW || view[3] != screenH) {
		resize(view[2], view[3]);
	}

	// Control the camera to look at the mesh.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	const Vector3D& c = appCamera.position();
	const Vector3D& r = appCamera.view_point();
	const Vector3D& u = appCamera.up_dir();

	gluLookAt(c.x, c.y, c.z,
		r.x, r.y, r.z,
		u.x, u.y, u.z);
}

void PathTracer::resize(size_t w, size_t h) {
	screenW = w;
	screenH = h;
	appCamera.set_screen_size(w, h);

	// resize HUDs
	textManager.resize(w, h);
	pt_info.resize(w, h);

	set_projection_matrix();
	/* bugged and not needed
if (mode != EDIT_MODE) {
	set_frame_size(w, h);
}
	*/
}

void PathTracer::set_projection_matrix() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(appCamera.v_fov(),
		appCamera.aspect_ratio(),
		appCamera.near_clip(),
		appCamera.far_clip());
}

string PathTracer::name() {
	return "PathTracer : FO2Visualizer";
}

string PathTracer::info() {
	switch (mode) {
	case EDIT_MODE:
		return "MeshEdit";
	case RENDER_MODE:
	case VISUALIZE_MODE:
		return "PathTracer";
	}
	return "TheMode";//                        
}

PathTracer::Mode PathTracer::get_mode() {
	return this->mode;
}

void PathTracer::load(SceneInfo* sceneInfo) {

	vector<Node>& nodes = sceneInfo->nodes;
	vector<DynamicScene::SceneLight *> lights;
	vector<DynamicScene::SceneObject *> objects;

	// save camera position to update camera control later
	CameraInfo *c;
	Vector3D c_pos = Vector3D();
	Vector3D c_dir = Vector3D();

	int len = nodes.size();
	for (int i = 0; i < len; i++) {
		Node& node = nodes[i];
		Instance *instance = node.instance;
		const Matrix4x4& transform = node.transform;

		switch (instance->type) {
		case Instance::CAMERA:
			c = static_cast<CameraInfo*>(instance);
			c_pos = (transform * Vector4D(c_pos, 1)).to3D();
			c_dir = (transform * Vector4D(c->view_dir, 1)).to3D().unit();
			init_camera(*c, transform);
			break;
		case Instance::LIGHT:
		{
			lights.push_back(
				init_light(static_cast<LightInfo&>(*instance), transform));
			break;
		}
		case Instance::SPHERE:
			objects.push_back(
				init_sphere(static_cast<SphereInfo&>(*instance), transform));
			break;
		case Instance::POLYMESH:
			objects.push_back(
				init_polymesh(static_cast<PolymeshInfo&>(*instance), transform));
			break;
		case Instance::MATERIAL:
			init_material(static_cast<MaterialInfo&>(*instance));
			break;
		}
	}

	appScene = new DynamicScene::Scene(objects, lights);

	const BBox& bbox = appScene->get_bbox();
	if (!bbox.empty()) {

		Vector3D target = bbox.centroid();
		canonical_view_distance = bbox.extent.norm() / 2 * 1.5;

		double view_distance = canonical_view_distance * 2;
		double min_view_distance = canonical_view_distance / 10.0;
		double max_view_distance = canonical_view_distance * 20.0;

		canonicalCamera.place(target,
			acos(c_dir.y),
			atan2(c_dir.x, c_dir.z),
			view_distance,
			min_view_distance,
			max_view_distance);

		appCamera.place(target,
			acos(c_dir.y),
			atan2(c_dir.x, c_dir.z),
			view_distance,
			min_view_distance,
			max_view_distance);

		set_scroll_rate();
	}

	// set default draw styles for meshEdit -
	appScene->set_draw_styles(&defaultStyle, &hoverStyle, &selectStyle);

}

void PathTracer::init_camera(CameraInfo& cameraInfo,
	const Matrix4x4& transform) {
	appCamera.configure(cameraInfo, screenW, screenH);
	canonicalCamera.configure(cameraInfo, screenW, screenH);
	set_projection_matrix();
}

void PathTracer::reset_camera() {
	appCamera.copy_placement(canonicalCamera);
}

DynamicScene::SceneLight *PathTracer::init_light(LightInfo& light,
	const Matrix4x4& transform) {
	switch (light.light_type) {
	case LightType::NONE:
		break;
	case LightType::AMBIENT:
		return new DynamicScene::AmbientLight(light);
	case LightType::DIRECTIONAL:
		return new DynamicScene::DirectionalLight(light, transform);
	case LightType::AREA:
		return new DynamicScene::AreaLight(light, transform);
	case LightType::POINT:
		return new DynamicScene::PointLight(light, transform);
	case LightType::SPOT:
		return new DynamicScene::SpotLight(light, transform);
	default:
		break;
	}
	return nullptr;
}

/**
* The transform is assumed to be composed of translation, rotation, and
* scaling, where the scaling is uniform across the three dimensions; these
* assumptions are necessary to ensure the sphere is still spherical. Rotation
* is ignored since it's a sphere, translation is determined by transforming the
* origin, and scaling is determined by transforming an arbitrary unit vector.
*/
DynamicScene::SceneObject *PathTracer::init_sphere(
	SphereInfo& sphere, const Matrix4x4& transform) {
	const Vector3D& position = (transform * Vector4D(0, 0, 0, 1)).projectTo3D();
	double scale = (transform * Vector4D(1, 0, 0, 0)).to3D().norm();
	return new DynamicScene::Sphere(sphere, position, scale);
}

DynamicScene::SceneObject *PathTracer::init_polymesh(
	PolymeshInfo& polymesh, const Matrix4x4& transform) {
	return new DynamicScene::Mesh(polymesh, transform);
}

void PathTracer::set_scroll_rate() {
	scroll_rate = canonical_view_distance / 10;
}

void PathTracer::init_material(MaterialInfo& material) {
	// TODO : Support Materials.
}

void PathTracer::cursor_event(float x, float y, bool update_pos_only) {	
	if (!update_pos_only) {
		if (leftDown && !middleDown && !rightDown) {
			mouse1_dragged(x, y);
		}
		else if (!leftDown && !middleDown && rightDown) {
			mouse2_dragged(x, y);
		}
		else if (!leftDown && !middleDown && !rightDown) {
			mouse_moved(x, y);
		}
	}
	//the app should always know the pos of the mouse
	mouseX = x;
	mouseY = y;

}

void PathTracer::scroll_event(float offset_x, float offset_y) {
	update_style();

	switch (mode) {
	case EDIT_MODE:
	case VISUALIZE_MODE:
		appCamera.move_forward(/*invert zooming, like.. it should be ... */offset_y * scroll_rate);
		break;
	case RENDER_MODE:
		break;
	}
}

void PathTracer::mouse_key_event(int key, int event, unsigned char mods, bool update_key_only) {
	// the app should always know which key is pressed, that means it should handle press|release events 
	// always atleast to update the
	// leftDown .. vars; so if update_key_only is true, that means that the press is handle by other handler, so only
	// the buttons are updated but no other action is taken
	if (update_key_only) {
		bool pressed = event == EVENT_PRESS; // can we have other event than EVENT_PRESS, EVENT_RELEASE ? 
		switch (key) {
		case MOUSE_LEFT: leftDown = pressed; break;
		case MOUSE_RIGHT: middleDown = pressed; break;
		case MOUSE_MIDDLE: rightDown = pressed; break;
		default: break;
		}
	}
	else {
		switch (event) {
		case EVENT_PRESS:
			switch (key) {
			case MOUSE_LEFT:
				mouse_pressed(LEFT);
				break;
			case MOUSE_RIGHT:
				mouse_pressed(RIGHT);
				break;
			case MOUSE_MIDDLE:
				mouse_pressed(MIDDLE);
				break;
			}
			break;
		case EVENT_RELEASE:
			switch (key) {
			case MOUSE_LEFT:
				mouse_released(LEFT);
				break;
			case MOUSE_RIGHT:
				mouse_released(RIGHT);
				break;
			case MOUSE_MIDDLE:
				mouse_released(MIDDLE);
				break;
			}
			break;
		}
	}
}

void PathTracer::keyboard_event(int key, int event, unsigned char mods) {
	switch (mode) {
	case RENDER_MODE:
		if (event == EVENT_PRESS) {
			switch (key) {
			case 'e': case 'E':
				to_edit_mode();
				break;
			case 'v': case 'V':
				stop();
				start_visualizing();
				mode = VISUALIZE_MODE;
				break;
			case 's': case 'S':
				save_image();
				break;
			case '+': case '=':
				stop();
				increase_area_light_sample_count();
				break;
			case '-': case '_':
				stop();
				decrease_area_light_sample_count();
				break;
			case '[': case ']':
				key_press(key);
				break;
			}
		}
		break;
	case VISUALIZE_MODE:
		if (event == EVENT_PRESS) {
			switch (key) {
			case 'e': case 'E':
				to_edit_mode();
				break;
			case 'r': case 'R':
				stop();

				mode = RENDER_MODE;
				break;
			case ' ':
				reset_camera(); // do we need this ? TODO
				break;
			default:
				key_press(key);
			}
		}
		break;
	case EDIT_MODE:
		if (event == EVENT_PRESS) {
			switch (key) {
			case 'r': case 'R':
				set_up_pathtracer();
				mode = RENDER_MODE;
				break;
			case 'v': case 'V':
				set_up_pathtracer();
				start_visualizing();
				mode = VISUALIZE_MODE;
				break;
			case ' ':
				reset_camera(); // do we need this ? TODO
				break;
			case 'h': case 'H':
				show_hud = !show_hud;
				break;
			case 'u': case 'U':
				appScene->upsample_selected_mesh();
				break;
			case 'd': case 'D':
				appScene->downsample_selected_mesh();
				break;
			case 'i': case 'I':
				// i for isotropic.
				appScene->resample_selected_mesh();
				break;
			case 'f': case 'F':
				appScene->flip_selected_edge();
				break;
			case 's': case 'S':
				appScene->split_selected_edge();
				break;
			case 'c': case 'C':
				appScene->collapse_selected_edge();
				break;
			default:
				break;
			}
		}
		break;
	}
}

void PathTracer::mouse_pressed(e_mouse_button b) {
	switch (b) {
	case LEFT:
		if (mode == EDIT_MODE) {
			if (appScene->has_hover()) {
				appScene->confirm_selection();
			}
			else {
				appScene->invalidate_selection();
			}
		}
		leftDown = true;
		break;
	case RIGHT:
		rightDown = true;
		break;
	case MIDDLE:
		middleDown = true;
		break;
	}
}

void PathTracer::mouse_released(e_mouse_button b) {
	switch (b) {
	case LEFT:
		leftDown = false;	
		break;
	case RIGHT:
		rightDown = false;
		break;
	case MIDDLE:
		middleDown = false;
		break;
	}
}

/*
When in edit mode and there is a selection, move the selection.
When in visualization mode, rotate.
*/
void PathTracer::mouse1_dragged(float x, float y) {
	if (mode == RENDER_MODE) {
		return;
	}
	float dx = (x - mouseX);
	float dy = (y - mouseY);

	if (mode == EDIT_MODE && appScene->has_selection()) {
		appScene->drag_selection(2 * dx / screenW, 2 * -dy / screenH,
			get_world_to_3DH());
	}
	else {
		/* invert cam*/
		appCamera.rotate_by(-dy * (PI / screenH), -dx * (PI / screenW));
	}
}

/*
When the mouse is dragged with the right button held down, translate.
*/
void PathTracer::mouse2_dragged(float x, float y) {
	if (mode == RENDER_MODE) return;
	float dx = (x - mouseX);
	float dy = (y - mouseY);

	// don't negate y because up is down.
	appCamera.move_by(-dx, dy, canonical_view_distance);
}

void PathTracer::mouse_moved(float x, float y) {
	if (mode != EDIT_MODE) return;
	y = screenH - y; // Because up is down.
					 // Converts x from [0, w] to [-1, 1], and similarly for y.
	Vector2D p(x * 2 / screenW - 1, y * 2 / screenH - 1);
	appScene->update_selection(p, get_world_to_3DH());
}

void PathTracer::to_edit_mode() {
	if (mode == EDIT_MODE)
		return;
	stop();
	clear();
	mode = EDIT_MODE;
	mouse_moved(mouseX, mouseY);
}

void PathTracer::set_up_pathtracer() {
	if (mode != EDIT_MODE) 
		return;
	// scene in changed here; a new staticScene
	// is created from the dynamicscene;
	// we change it only one time
	if (!set_up) { set_up = true;
		set_scene(appScene->get_static_scene(), _cdb2Ptr,
			_bvhPtr, other_file_type);	
	}
	set_camera(&appCamera);
	set_frame_size(screenW, screenH);
	build_accel();
}

Matrix4x4 PathTracer::get_world_to_3DH() {
	Matrix4x4 P, M;
	glGetDoublev(GL_PROJECTION_MATRIX, &P(0, 0));
	glGetDoublev(GL_MODELVIEW_MATRIX, &M(0, 0));
	return P * M;
}

inline void PathTracer::draw_string(OSDText &txt_mgr, 
	float x, float y, string str, size_t size, 
	const Color& c) {
	int line_index = txt_mgr.add_line((x * 2 / screenW) - 1.0,
		(-y * 2 / screenH) + 1.0,
		str, size, c);
	messages.push_back(line_index);
}

void PathTracer::draw_coordinates() {

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);
	glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
	glVertex3i(0, 0, 0);
	glVertex3i(1, 0, 0);

	glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
	glVertex3i(0, 0, 0);
	glVertex3i(0, 1, 0);

	glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
	glVertex3i(0, 0, 0);
	glVertex3i(0, 0, 1);

	glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
	for (int x = 0; x <= 8; ++x) {
		glVertex3i(x - 4, 0, -4);
		glVertex3i(x - 4, 0, 4);
	}
	for (int z = 0; z <= 8; ++z) {
		glVertex3i(-4, 0, z - 4);
		glVertex3i(4, 0, z - 4);
	}
	glEnd();

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

}

void PathTracer::draw_hud() {
	textManager.clear();
	messages.clear();

	const size_t size = 16;
	const float x0 = use_hdpi ? screenW - 300 * 2 : screenW - 300;
	const float y0 = use_hdpi ? 128 : 64;
	const int inc = use_hdpi ? 48 : 24;
	float y = y0 + inc - size;

	// No selection --> no messages.
	if (!appScene->has_selection()) {
		//draw_string(x0, y, "No mesh feature is selected", size, text_color);
		y += inc;
	}
	else {
		DynamicScene::SelectionInfo *selectionInfo = appScene->get_selection_info();
		for (const string& s : selectionInfo->info) {
			size_t split = s.find_first_of(":");
			if (split != string::npos) {
				split++;
				string s1 = s.substr(0, split);
				string s2 = s.substr(split);
				draw_string(textManager, x0, y, s1, size, TO_C_COLOR(ctext));
				draw_string(textManager, x0 + (use_hdpi ? 150 : 75), y, s2, size, TO_C_COLOR(ctext));
			}
			else {
				draw_string(textManager, x0, y, s, size, TO_C_COLOR(ctext));
			}
			y += inc;
		}
	}

	// -- First draw a lovely black rectangle.
	/* no lovely rectangle.
	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, screenW, screenH);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, screenW, screenH, 0, 0, 1);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(0, 0, -1);

	// -- Black with opacity .8;

	glColor4f(0.0, 0.0, 0.0, 0.8);

	float min_x = x0 - 32;
	float min_y = y0 - 32;
	float max_x = screenW;
	float max_y = y;

	float z = 0.0;

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	glBegin(GL_QUADS);

	glVertex3f(min_x, min_y, z);
	glVertex3f(min_x, max_y, z);
	glVertex3f(max_x, max_y, z);
	glVertex3f(max_x, min_y, z);
	glEnd();*/
	// end lovely Rec

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glPopAttrib();

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

	textManager.render();
} // class Pathtracer

/**
 *  Draws the node_info string
 */
void PathTracer::draw_pt_info() {
	if (node_info == "")
	{
		if (other_fType == FO2_BVH_FILETYPE) {
			set_node_nmb_bvh();
		} else if (other_fType == FO2_CDB2_FILETYPE)
		{
			set_node_nmb_cdb2();
		}
	}
		

	pt_info.clear();
	const size_t size = 16;
	const float x0 = use_hdpi ? screenW - 300 * 2 : screenW - 300;
	const float y0 = use_hdpi ? 4 : 2;
	float y = screenH- y0 - size;

	draw_string(pt_info, x0, y, node_info, size, TO_C_COLOR(ctext));

	pt_info.render();
}


}  // namespace CMU462
