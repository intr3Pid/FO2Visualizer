#ifndef CMU462_RAYTRACER_H
#define CMU462_RAYTRACER_H

// using AntTweakbar
#define WITH_ANTTWEAKBAR

// STL
#include <stack>
#include <random>
#include <string>
#include <iostream>
#include <thread>
#include <sstream>
#include <atomic>
#include <vector>
#include <algorithm>

// libCMU462
#include "CMU462/CMU462.h"
#include "CMU462/viewer.h" //  
#include "CMU462/osdtext.h"
#include "CMU462/timer.h"

// COLLADA
#include "collada/collada.h"
#include "collada/light_info.h"
#include "collada/sphere_info.h"
#include "collada/polymesh_info.h"
#include "collada/material_info.h"

#include "bvh.h"
#include "camera.h"
#include "sampler.h"
#include "image.h"
#include "work_queue.h"

// MeshEdit
#include "static_scene/scene.h"
#include "static_scene/sphere.h"
#include "static_scene/triangle.h"
#include "static_scene/light.h"

#include "static_scene/environment_light.h"
using CMU462::StaticScene::EnvironmentLight;

#include "dynamic_scene/scene.h"
#include "dynamic_scene/ambient_light.h"
#include "dynamic_scene/environment_light.h"
#include "dynamic_scene/directional_light.h"
#include "dynamic_scene/area_light.h"
#include "dynamic_scene/point_light.h"
#include "dynamic_scene/spot_light.h"
#include "dynamic_scene/sphere.h"
#include "dynamic_scene/mesh.h"
#include "halfEdgeMesh.h"
#include "meshEdit.h"


using CMU462::StaticScene::Scene;

#include "image.h"

// win stuff
#include <Shobjidl.h>

// Flatout 2 
#include "FO2_collision.h"
#include "FO2_bvh.h"

// AntTweakBar
#ifdef WITH_ANTTWEAKBAR
#include "../AntTweakBar/include/for_GLFW3/AntTweakBar.h"
#endif

#define BAR_NUM 4
#define ID_MAIN_BAR 0
#define ID_INFO_BAR 1
#define ID_FILEIO_BAR 2
#define ID_TWEAK_BAR 3

using namespace std;

// we want to use arrays as representation of the colors to be able 
// to tweak them with AntTweakBar, no we must convert them
#define TO_C_COLOR(c) \
        Color(c[0], c[1], c[2], c[3])

#define ID_CNODE 0
#define ID_CNODEHL 1
#define ID_CNODEHLCHILD 2
#define ID_CSBLUE 3
#define ID_CHLLEAVES 4
#define ID_CHLLEFT 5
#define ID_CHLRIGHT 6
#define ID_CHLEDGES 7

#define ID_CTEXT 8

#define FO2_CDB2_FILETYPE 1
#define FO2_BVH_FILETYPE 2


namespace CMU462 {

    struct WorkItem {

        // Default constructor.
        WorkItem() : WorkItem(0, 0, 0, 0) { }

        WorkItem(int x, int y, int w, int h)
            : tile_x(x), tile_y(y), tile_w(w), tile_h(h) {}

        int tile_x;
        int tile_y;
        int tile_w;
        int tile_h;

    };

    struct AppConfig {

        AppConfig() {

            pathtracer_ns_aa = 1;
            pathtracer_max_ray_depth = 1;
            pathtracer_ns_area_light = 4;

            pathtracer_ns_diff = 1;
            pathtracer_ns_glsy = 1;
            pathtracer_ns_refr = 1;

            pathtracer_num_threads = 1;
            pathtracer_envmap = NULL;

        }

        size_t pathtracer_ns_aa;
        size_t pathtracer_max_ray_depth;
        size_t pathtracer_ns_area_light;
        size_t pathtracer_ns_diff;
        size_t pathtracer_ns_glsy;
        size_t pathtracer_ns_refr;
        size_t pathtracer_num_threads;
        HDRImageBuffer* pathtracer_envmap;

    };

    /**
     * A pathtracer with BVH accelerator and BVH visualization capabilities.
     * It is always in exactly one of the following states:
     * -> INIT: is missing some data needed to be usable, like a camera or scene.
     * -> READY: fully configured, but not rendering.
     * -> VISUALIZE: visualizatiNG BVH aggregate.
     * -> RENDERING: rendering a scene.
     * -> DONE: completed rendering a scene.
     */
    class PathTracer : public Renderer {
    public:

        /**
         * Constructor with config as parameter
         */
        PathTracer(AppConfig config);

        /**
         * Destructor.
         * Frees all the internal resources used by the pathtracer.
         */
        ~PathTracer();


        /**
         * Sets up the tracer
         */
        void init();

		/**
		 * Initilizes the bars
		 */
		void init_bars();

        /*
         * Renders
         */
        void render();


        /**
         * If in the INIT state, configures the pathtracer to use the given scene. If
         * configuration is done, transitions to the READY state.
         * This DOES take ownership of the scene, and therefore deletes it if a new
         * scene is later passed in.
         * \param scene pointer to the new scene to be rendered
         */
        void set_scene(Scene* scene
#ifdef WITH_FO2_COLLISION 
            , cTrackCollision * pcdb2, cTrackBVH * pbvh, int other_fType
#endif
        );

        /**
         * If in the INIT state, configures the pathtracer to use the given camera. If
         * configuration is done, transitions to the READY state.
         * This DOES NOT take ownership of the camera, and doesn't delete it ever.
         * \param camera the camera to use in rendering
         */
        void set_camera(Camera* camera);

        /**
         * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
         * RENDERING, or DONE), transitions to READY b/c a changing window size
         * would invalidate the output. If in INIT and configuration is done,
         * transitions to READY.
         * \param width width of the frame
         * \param height height of the frame
         */
        void set_frame_size(size_t width, size_t height);


        /*
         *
         */

        void resize(size_t w, size_t h);



        /**
         * Update result on screen.
         * If the pathtracer is in RENDERING or DONE, it will display the result in
         * its frame buffer. If the pathtracer is in VISUALIZE mode, it will draw
         * the BVH visualization with OpenGL.
         */
        void update_screen();

        /**
         * Transitions from any running state to READY.
         */
        void stop();

        /**
         * If the pathtracer is in READY, delete all internal data, transition to INIT.
         */
        void clear();

        /**
         * If the pathtracer is in READY, transition to VISUALIZE.
         */
        void start_visualizing();

        /**
         * If the pathtracer is in READY, transition to RENDERING.
         */
        //void start_raytracing();

        /**
         * If the pathtracer is in VISUALIZE, handle key presses to traverse the bvh.
         */
        void key_press(int key);

        /**
         * Increase the pathtracer's area light sample count parameter by 2X
         */
        void increase_area_light_sample_count();

        /**
         * Decrease the pathtracer's area light sample count parameter by 2X
         */
        void decrease_area_light_sample_count();

        /**
         * Save rendered result to png file.
         */
        void save_image();


        /**
        * get colors by ID
        */
        float * get_color(unsigned int ID);


		/**
		 * sets Viewer. ugly.
		 */
		void set_viewer(Viewer *v);
        
        /**
        * Callbacks
        */
        void cursor_event(float x, float y, bool update_pos_only);
        void scroll_event(float offset_x, float offset_y);
        void mouse_key_event(int key, int event, unsigned char mods, bool update_key_only);
        void keyboard_event(int key, int event, unsigned char mods);
        void load(Collada::SceneInfo* sceneInfo);



        std::string name();
        std::string info();


		// Bars callbacks
		static void TW_CALL GetMyStdStringCB(void *value, void * clientData);
		static void TW_CALL SetMyStdStringCB(const void *value, void *clientData);
		void set_node_nmb_bvh();
		void set_node_nmb_cdb2();

        int other_file_type;

        void setCDB2ptr(cTrackCollision *ptr) {
            if (ptr) _cdb2Ptr = ptr;
        }
        void set_bvhPtr(cTrackBVH *ptr) {
            if (ptr) _bvhPtr = ptr;
        }

        enum Mode {
            EDIT_MODE,
            RENDER_MODE,
            VISUALIZE_MODE
        };
        Mode mode;
        Mode get_mode();


		// update stack.
		void update_stack();

    private:

        /**
         * Used in initialization.
         */
        bool has_valid_configuration();

        /**
         * Build acceleration structures.
         */
        void build_accel();

        /**
         * Visualize acceleration structures.
         */
        void visualize_accel() const;

        /**
         * Trace an ray in the scene.
         */
        //Spectrum trace_ray(const Ray& ray);

        /**
         * Trace a camera ray given by the pixel coordinate.
         */
        //Spectrum raytrace_pixel(size_t x, size_t y);

        /**
         * Raytrace a tile of the scene and update the frame buffer. Is run
         * in a worker thread.
         */
        //void raytrace_tile(int tile_x, int tile_y, int tile_w, int tile_h);

        /**
         * Implementation of a ray tracer worker thread
         */
        //void worker_thread();

        /**
         * Log a ray miss.
         */
        void log_ray_miss(const Ray& r);

        /**
         * Log a ray hit.
         */
        void log_ray_hit(const Ray& r, double hit_t);

        enum State {
            INIT,               ///< to be initialized
            READY,              ///< initialized ready to do stuff
            VISUALIZE,          ///< visualizing BVH accelerator aggregate
            RENDERING,          ///< started but not completed raytracing
            DONE                ///< started and completed raytracing
        };

        // Configurables //

        State state;          ///< current state
        Scene* scene;         ///< current scene
        Camera* camera;       ///< current camera

        // colors 
        float cnode[4], cnode_hl[4], cnode_hl_child[4], s_blue[4],
            cprim_hl_leaves[4], cprim_hl_left[4],
            cprim_hl_right[4], cprim_hl_edges[4], ctext[4];

        // Integrator sampling settings //

        size_t max_ray_depth; ///< maximum allowed ray depth (applies to all rays)
        size_t ns_aa;         ///< number of camera rays in one pixel (along one axis)
        size_t ns_area_light; ///< number samples per area light source
        size_t ns_diff;       ///< number of samples - diffuse surfaces
        size_t ns_glsy;       ///< number of samples - glossy surfaces
        size_t ns_refr;       ///< number of samples - refractive surfaces

        // Integration state //

        vector<int> tile_samples; ///< current sample rate for tile
        size_t num_tiles_w;       ///< number of tiles along width of the image
        size_t num_tiles_h;       ///< number of tiles along height of the image

        // Components //
        cTrackCollision * cdb2Ptr;
        cTrackBVH * bvhPtr;
        int other_fType; // cdb2 oder bvh
		bool draw_mesh;
		bool draw_non_hl_aabbs;

    public:
		bool tree_mode; // flat (1) or normal binary tree (0)
	private:

        EnvironmentLight *envLight;    ///< environment map
        Sampler2D* gridSampler;        ///< samples unit grid
        Sampler3D* hemisphereSampler;  ///< samples unit hemisphere
        HDRImageBuffer sampleBuffer;   ///< sample buffer
        ImageBuffer frameBuffer;       ///< frame buffer
        Timer timer;                   ///< performance test timer

        // Internals //

        size_t numWorkerThreads;
        size_t imageTileSize;

		// pathtracer already set up
		bool set_up;
        bool continueRaytracing;                  ///< rendering should continue
        std::vector<std::thread*> workerThreads;  ///< pool of worker threads
        std::atomic<int> workerDoneCount;         ///< worker threads management
        WorkQueue<WorkItem> workQueue;            ///< queue of work for the workers

        // Tonemapping Controls //

        float tm_gamma;                           ///< gamma
        float tm_level;                           ///< exposure level
        float tm_key;                             ///< key value
        float tm_wht;                             ///< white point



        void to_edit_mode();
        void set_up_pathtracer();

        DynamicScene::Scene *appScene;

		// primitives of all staticScenes
		vector<Primitive*> primitives; // must not be freed 

        // View Frustrum Variables.
        // On resize, the aspect ratio is changed. On reset_camera, the position and
        // orientation are reset but NOT the aspect ratio.
        Camera appCamera;
        Camera canonicalCamera;

        size_t screenW;
        size_t screenH;

        // Length of diagonal of bounding box for the mesh.
        // Guranteed to not have the camera occlude with the mes.
        double canonical_view_distance;

        // Rate of translation on scrolling.
        double scroll_rate;


        cTrackCollision * _cdb2Ptr;
        cTrackBVH * _bvhPtr;


        /*
        Called whenever the camera fov or screenW/screenH changes.
        */
        void set_projection_matrix();

        /**
        * Fills the DrawStyle structs.
        */
        void initialize_style();

        /**
        * Update draw styles properly given the current view distance.
        */
        void update_style();

        /**
        * Reads and combines the current modelview and projection matrices.
        */
        Matrix4x4 get_world_to_3DH();

        // Initialization functions to get the opengl cooking with oil.
        void init_camera(Collada::CameraInfo& camera, const Matrix4x4& transform);
        DynamicScene::SceneLight *init_light(Collada::LightInfo& light, const Matrix4x4& transform);
        DynamicScene::SceneObject *init_sphere(Collada::SphereInfo& polymesh, const Matrix4x4& transform);
        DynamicScene::SceneObject *init_polymesh(Collada::PolymeshInfo& polymesh, const Matrix4x4& transform);
        void init_material(Collada::MaterialInfo& material);

        void set_scroll_rate();

        // Resets the camera to the canonical initial view position.
        void reset_camera();

        // Rendering functions.
        void update_gl_camera();

        // style for elements that are neither hovered nor selected
        DynamicScene::DrawStyle defaultStyle;
        DynamicScene::DrawStyle hoverStyle;
        DynamicScene::DrawStyle selectStyle;

        // Internal event system //

        float mouseX, mouseY;
        enum e_mouse_button {
            LEFT = MOUSE_LEFT,
            RIGHT = MOUSE_RIGHT,
            MIDDLE = MOUSE_MIDDLE
        };

        bool leftDown;
        bool rightDown;
        bool middleDown;

        // Event handling //

        void mouse_pressed(e_mouse_button b);   // Mouse pressed.
        void mouse_released(e_mouse_button b);  // Mouse Released.
        void mouse1_dragged(float x, float y);  // Left Mouse Dragged.
        void mouse2_dragged(float x, float y);  // Right Mouse Dragged.
        void mouse_moved(float x, float y);     // Mouse Moved.
        void reset_keys();                      // if mouse event is handled by other handler, keys should be resetted

        // OSD for edit mode //
        OSDText textManager;
        //Color text_color;
        vector<int> messages;


		// OSD for pathtrace
		OSDText pt_info;
		bool show_pt_info;
		void draw_pt_info();

        // Coordinate System //
        bool show_coordinates;
        void draw_coordinates();

        // HUD //
        bool show_hud;
        void draw_hud();
        inline void draw_string(OSDText &txt_mgr, float x,
			float y, string str, size_t size, const Color& c);


		// Bars Stuff
		string node_info;

        // Visualizer Controls //
        std::stack<tree<cCDB2_node>::iterator> cdb2selectionHistory;  ///< node selection history

        std::vector<LoggedRay> rayLog;          ///< ray tracing log
        bool show_rays;                         ///< show rays from raylog


    };// class PathTracer


}  // namespace CMU462

#endif  // CMU462_RAYTRACER_H
