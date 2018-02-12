#include "CMU462/CMU462.h"
#include "CMU462/viewer.h"

#define TINYEXR_IMPLEMENTATION
#include "CMU462/tinyexr.h"

#include "application.h"
#include "image.h"

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
  float* channel_r = (float*) exr.images[2];
  float* channel_g = (float*) exr.images[1];
  float* channel_b = (float*) exr.images[0];
  for (size_t i = 0; i < exr.width * exr.height; i++) {
    envmap->data[i] = Spectrum(channel_r[i], 
                               channel_g[i], 
                               channel_b[i]);
  }

  return envmap;
}

int main( int argc, char** argv ) {

  // get the options
  AppConfig config; int opt;
  while ( (opt = getopt(argc, argv, "s:l:t:m:e:h")) != -1 ) {  // for each option...
    switch ( opt ) {
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
        config.pathtracer_envmap = load_exr(optarg);
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

#ifdef WITH_FO2_COLLISION
  /* try to load .gen with same name */
  size_t fnd1 = sceneFilePath.find_last_of(".");
  //size_t fnd2 = sceneFilePath.find_last_of("/\\");
  string nameWo = "";
  bool loaded = false;
  cTrackCollision * fcdb2 = NULL;
  do {
  if (fnd1 != std::string::npos /*&& fnd2 != std::string::npos*/) {
	  nameWo = sceneFilePath.substr(0, fnd1);
	  nameWo.append(".gen");
  } else {
       break;
  }

  ifstream cdb2s(nameWo, ios::binary );
  unsigned int cdb2_ID = 0;
  if (cdb2s.is_open()) {
	  cdb2s.read((char*)&cdb2_ID, sizeof(unsigned int));
	  cdb2s.close();
	  if (cdb2_ID == CDB2_ID) { /* read cdb2_gen file*/
		  fcdb2 = new cTrackCollision(nameWo);
		  loaded = true;
	  }
  } else {
	  break;// ? collada has it's own error handler ... 
  }

  } while (0);
	if(loaded) {
		msg("- Loaded FlatOut 2 Collision file -");
	} else {
		msg("- No FlatOut 2 Collision file loaded -");
	}
#endif // #ifdef WITH_FO2_COLLISION


  // parse scene
  Collada::SceneInfo *sceneInfo = new Collada::SceneInfo();
  if (Collada::ColladaParser::load(sceneFilePath.c_str(), sceneInfo) < 0) {
    delete sceneInfo;
    exit(0);
  }

  // create viewer
  Viewer viewer = Viewer();

  // create application
  Application app (config);

  // set renderer
  viewer.set_renderer(&app);

  // init viewer
  viewer.init();

  // load scene
  app.load(sceneInfo);

#ifdef WITH_FO2_COLLISION
  // set ptr to cdb2
  app.setCDB2ptr(fcdb2);
#endif // #ifdef WITH_FO2_COLLISION


  delete sceneInfo;
  
  // NOTE (sky): are we copying everything to dynamic scene? If so:
  // TODO (sky): check and make sure the destructor is freeing everything

  // start viewer
  viewer.start();

#ifdef WITH_FO2_COLLISION
  delete fcdb2;
#endif // #ifdef WITH_FO2_COLLISION

  // TODO:
  // apparently the meshEdit renderer instance was not destroyed properly
  // not sure if this is due to the recent refractor but if anyone got some
  // free time, check the destructor for Application.
  exit(EXIT_SUCCESS); // shamelessly faking it

  return 0;

}
