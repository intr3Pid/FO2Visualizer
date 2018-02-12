#include "CMU462/CMU462.h"

#define WITH_ANTTWEAKBAR
#include "CMU462/viewer.h"

#define TINYEXR_IMPLEMENTATION
#include "CMU462/tinyexr.h"
#include "pathtracer.h"
#include <iostream>
#include <unistd.h>


using namespace std;
using namespace CMU462;

#define msg(s) cerr << "[PathTracer] " << s << endl;

void usage(const char* binaryName) {
  printf("Usage: %s [options] <scenefile>\n", binaryName);
  printf("Program Options:\n");
  printf("  -s  <INT>        Number of camera rays per pixel\n");
  printf("  -l  <INT>        Number of samples per area light\n");
  printf("  -t  <INT>        Number of render threads\n");
  printf("  -m  <INT>        Maximum ray depth\n");
  printf("  -e  <PATH>       Path to environment map\n");
  printf("  -h               Print this help message\n");
  printf("\n");
}

 // I put some error here, I get linker error, maybe chek why if you need EXR
 
HDRImageBuffer* load_exr(const char* file_path) {

	const char* err;

	EXRImage exr;
	InitEXRImage(&exr);

	int ret = ParseMultiChannelEXRHeaderFromFile(&exr, file_path, &err);
	if (ret != 0) {
		msg("Error parsing OpenEXR file: " << err);
		return NULL;
	}

	for (int i = 0; i < exr.num_channels; i++) {
		if (exr.pixel_types[i] == TINYEXR_PIXELTYPE_HALF) {
			exr.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
		}
	}

	ret = LoadMultiChannelEXRFromFile(&exr, file_path, &err);
	if (ret != 0) {
		msg("Error loading OpenEXR file: " << err);
		exit(EXIT_FAILURE);
	}

	HDRImageBuffer* envmap = new HDRImageBuffer();
	envmap->resize(exr.width, exr.height);
	float* channel_r = (float*)exr.images[2];
	float* channel_g = (float*)exr.images[1];
	float* channel_b = (float*)exr.images[0];
	for (size_t i = 0; i < exr.width * exr.height; i++) {
		envmap->data[i] = Spectrum(channel_r[i],
			channel_g[i],
			channel_b[i]);
	}

	return envmap;
}

int main(int argc, char** argv) {

	// get the options
	AppConfig config; int opt;
	while ((opt = getopt(argc, argv, "s:l:t:m:e:h")) != -1) {  // for each option...
		switch (opt) {
		case 's':
			config.pathtracer_ns_aa = atoi(optarg);
			break;
		case 'l':
			config.pathtracer_ns_area_light = atoi(optarg);
			break;
		case 't':
			config.pathtracer_num_threads = atoi(optarg);
			break;
		case 'm':
			config.pathtracer_max_ray_depth = atoi(optarg);
			break;
		case 'e':
			//config.pathtracer_envmap = load_exr(optarg);
			break;
		default:
			usage(argv[0]);
			return 1;
		}
	}

	// print usage if no argument given
	if (optind >= argc) {
		usage(argv[0]);
		return 1;
	}

	string sceneFilePath = argv[optind];
	msg("Input scene file: " << sceneFilePath);


	/* try to load .gen with same name */
	size_t dot_pos = sceneFilePath.find_last_of(".");
	string cdb2_path = "", bvh_path = "";
	int loaded = -1;
	cTrackCollision * fcdb2 = nullptr;
	cTrackBVH *BVH_Ob = nullptr;
	do {
		if (dot_pos != string::npos) {
			cdb2_path = bvh_path = sceneFilePath.substr(0, dot_pos);
			cdb2_path.append(".cdb2");
			// bvh format 
			bvh_path.append(".bvh");
		}
		else {
			break;
		}

		ifstream cdb2s(cdb2_path, ios::binary);
		ifstream bvhI(bvh_path, ios::binary);
		
		if (cdb2s.is_open()) {
			// check the ID of the file
			unsigned int cdb2_ID = 0;
			cdb2s.read((char*)&cdb2_ID, sizeof(unsigned int));
			cdb2s.close();
			if (cdb2_ID == CDB2_ID) { /* read cdb2_gen file*/
				fcdb2 = new cTrackCollision(cdb2_path);
				if(fcdb2 != nullptr)
				{
					loaded = FO2_CDB2_FILETYPE;
				}
				
			}
		}
		else if (bvhI.is_open())
		{
			// check the ID of the file
			unsigned int bvh_ID = 0;
			bvhI.read((char*)&bvh_ID, sizeof(unsigned int));
			bvhI.close();
			if (bvh_ID == FO2_BVH_ID)
			{
				BVH_Ob = new cTrackBVH(bvh_path);
				if (BVH_Ob != nullptr) {
					loaded = FO2_BVH_FILETYPE;
				}
			}	

		}
		else {
			break;// collada has it's own error handler 
		}

	} while (false);

	if (loaded == FO2_CDB2_FILETYPE) {
		msg("- Loaded FlatOut 2 cdb2 file -");
	}
	else if (loaded == FO2_BVH_FILETYPE) {
		msg("- Loaded FlatOut 2 bvh file -");
	}
	else {
		msg("- No FlatOut 2 file loaded -");
	}



	// parse scene
	Collada::SceneInfo *sceneInfo = new Collada::SceneInfo();
	if (Collada::ColladaParser::load(sceneFilePath.c_str(), sceneInfo, FO2_SCALE) < 0) {
		delete sceneInfo;
		exit(0);
	}

	// create viewer
	Viewer viewer = Viewer();

	// create application
	PathTracer pt(config);

	// set renderer
	viewer.set_renderer(&pt);

	// set pointer to FO2 file objects
	pt.setCDB2ptr(fcdb2);
	pt.set_bvhPtr(BVH_Ob);
	// set loaded FO2 file type
	pt.other_file_type = loaded;


	// init viewer
	if (viewer.init() == 1) {

		// load scene
		pt.load(sceneInfo);

	
		// NOTE (sky): are we copying everything to dynamic scene? If so:
		// TODO (sky): check and make sure the destructor is freeing everything

		// start viewer
		viewer.start();
	}

	delete fcdb2;
	delete BVH_Ob;


	delete sceneInfo;
	// TODO:
	// apparently the meshEdit renderer instance was not destroyed properly
	// not sure if this is due to the recent refractor but if anyone got some
	// free time, check the destructor for Application.
	exit(EXIT_SUCCESS); // shamelessly faking it

	return 0;

}
