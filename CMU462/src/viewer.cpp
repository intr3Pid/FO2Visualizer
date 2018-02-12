#include "viewer.h"

#include <vector>
#include <iostream>

#include "GL/glew.h"

#include "console.h"

using namespace std;
using namespace chrono;

#define DEFAULT_W 960
#define DEFAULT_H 640

namespace CMU462 {

// HDPI display
bool Viewer::HDPI;

// framecount & related timeers
int Viewer::framecount;
time_point<system_clock> Viewer::sys_last; 
time_point<system_clock> Viewer::sys_curr; 

// draw toggles
bool Viewer::showInfo = true;

// window properties
GLFWwindow* Viewer::window;
size_t Viewer::buffer_w;
size_t Viewer::buffer_h;

// user space renderer
Renderer* Viewer::renderer; 

// on-screen display
OSDText* Viewer::osd_text;
int Viewer::line_id_renderer;
int Viewer::line_id_framerate;

Viewer::Viewer() {

}

Viewer::~Viewer() {

  glfwDestroyWindow(window);

#ifdef WITH_ANTTWEAKBAR
  // Terminate AntTweakBar and GLFW
  TwTerminate();
#endif // WITH_ANTTWEAKBAR

  glfwTerminate();
  
  // free resources
  delete renderer;
  delete osd_text;
}

int Viewer::init() {

	int ret = -1;

	do {


		// initialize glfw
		glfwSetErrorCallback(err_callback);
		if (!glfwInit()) {
			out_err("Error: could not initialize GLFW!");
			break;
		}

		// create window
		string title = renderer ? "CMU462: " + renderer->name() : "CMU462";
		window = glfwCreateWindow(DEFAULT_W, DEFAULT_H, title.c_str(), NULL, NULL);
		if (!window) {
			out_err("Error: could not create window!");
			break;
		}

		// set context
		glfwMakeContextCurrent(window);
		glfwSwapInterval(1);

		// framebuffer event callbacks
		glfwSetFramebufferSizeCallback(window, resize_callback);

		// key event callbacks
		glfwSetKeyCallback(window, key_callback);

		// cursor event callbacks
		glfwSetCursorPosCallback(window, cursor_callback);

		// wheel event callbacks
		glfwSetScrollCallback(window, scroll_callback);

		// mouse button callbacks
		glfwSetInputMode(window, GLFW_STICKY_MOUSE_BUTTONS, 1);
		glfwSetMouseButtonCallback(window, mouse_button_callback);


#ifdef WITH_ANTTWEAKBAR
		// some more callbacks
		glfwSetCharCallback(window, on_char_callback);

		TwInit(TW_OPENGL, nullptr);
		// Change the font size, and add a global message to the Help bar.
		TwDefine(" GLOBAL fontSize=3 help='Help.' ");

#endif

			// initialize glew
		if (glewInit() != GLEW_OK) {
			out_err("Error: could not initialize GLEW!");
			break;
		}

		// enable alpha blending
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		// resize components to current window size, get DPI
		glfwGetFramebufferSize(window, (int*)&buffer_w, (int*)&buffer_h);
		if (buffer_w > DEFAULT_W) HDPI = true;

		// initialize renderer if already set
		if (renderer) {
			if (HDPI) 
				renderer->use_hdpi_reneder_target();
			renderer->init();
		}

		// initialize status OSD
		osd_text = new OSDText();
		if (osd_text->init(HDPI) < 0) {
			out_err("Error: could not initialize on-screen display!");
			break;
		}

		// add lines for renderer and fps
		line_id_renderer = osd_text->add_line(-0.95, 0.90, "Renderer",
			18, Color(0.15f, 0.5f, 0.15f));
		line_id_framerate = osd_text->add_line(-0.98, -0.96, "Framerate",
			14, Color(0.15f, 0.5f, 0.15f));

		// resize elements to current size
		resize_callback(window, buffer_w, buffer_h);

		ret = 1;

	} while (0);
	return ret;
}

void Viewer::start() {
  // start timer
  sys_last = system_clock::now();

  // run update loop
  while( !glfwWindowShouldClose( window )) {
	update();
  }
}

void Viewer::set_renderer(Renderer *renderer) {
  this->renderer = renderer;
}


TwBar *Viewer::get_bar(unsigned int ID, int *status) {
	status[0] = -1;
	if( ID  > bars.size() - 1 ) 
		return nullptr;
	status[0] = 1;
	return bars[ID];
}


TwBar *Viewer::create_new_bar(string name, int *status) {
	TwBar * new_bar = TwNewBar(name.c_str());
	if (new_bar == nullptr)
		status[0] = -1;
	else 
		status[0] = -1;
	bars.push_back(new_bar);
	return new_bar;
}

void Viewer::update() {
  
  // clear frame
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // run user renderer
  if (renderer) {
    renderer->render();
  }

  // draw info
  if( showInfo ) {
    drawInfo();        
  } 

#ifdef WITH_ANTTWEAKBAR
  // Draw tweak bars
  TwDraw();
#endif


  // swap buffers
  glfwSwapBuffers(window); 

  // poll events
  //glfwPollEvents(); 
  /* sleep ... */
  //if (((Application*)renderer)->get_mode() != Application::EDIT_MODE  )
    glfwWaitEvents();

}


void Viewer::drawInfo() {

  // compute timers - fps is update every second
  sys_curr = system_clock::now();
  double elapsed = ((duration<double>) (sys_curr - sys_last)).count();
  if (elapsed >= 1.0f) {

    // update framecount OSD
    Color c = framecount < 20 ? Color(1.0, 0.35, 0.35) : Color(0.15, 0.5, 0.15);
    osd_text->set_color(line_id_framerate, c);
    string framerate_info = "Framerate: " + to_string(framecount) + " fps";
    osd_text->set_text(line_id_framerate, framerate_info);

    // reset timer and counter
    framecount = 0;
    sys_last = sys_curr; 

  } else {

    // increment framecount
    framecount++;
  
  }

  // udpate renderer OSD
  // TODO: This is done on every update and it shouldn't be!
  // The viewer should only update when the renderer needs to
  // update the info text. 
  if (renderer) {
    string renderer_info = renderer->info();
    osd_text->set_text(line_id_renderer, renderer_info);
  } else {
    string renderer_info = "No input renderer";
    osd_text->set_text(line_id_renderer, renderer_info);
  }

  // render OSD
  osd_text->render();

}

void Viewer::err_callback( int error, const char* description ) {
    out_err( "GLFW Error: error code: " << error << ", discription: " << description );
}

void Viewer::resize_callback( GLFWwindow* window, int width, int height ) {

  // get framebuffer size
  int w, h; 
  glfwGetFramebufferSize(window, &w, &h );
    
  // update buffer size
  buffer_w = w; buffer_h = h;
  glViewport( 0, 0, buffer_w, buffer_h );

  // resize on-screen display
  osd_text->resize(buffer_w, buffer_h);

  // resize render if there is a user space renderer
  if (renderer) renderer->resize( buffer_w, buffer_h );


#ifdef WITH_ANTTWEAKBAR
  TwWindowSize(width, height);
#endif

}

void Viewer::cursor_callback( GLFWwindow* window, double xpos, double ypos ) {
	bool handled = false;

#ifdef WITH_ANTTWEAKBAR
	// if anttweakbar didn't hadled the cursor event, handle by pathtracer
	// but actualize pos of mouse always 
	handled = TwEventMousePosGLFW3(window, xpos, ypos) != 0;
#endif

	// forward pan event to renderer
	if (HDPI) {
		float cursor_x = 2 * xpos;
		float cursor_y = 2 * ypos;
		renderer->cursor_event(cursor_x, cursor_y, handled);
	}
	else {
		float cursor_x = xpos;
		float cursor_y = ypos;
		renderer->cursor_event(cursor_x, cursor_y, handled);
	}

}

void Viewer::scroll_callback( GLFWwindow* window, double xoffset, double yoffset) {

#ifdef WITH_ANTTWEAKBAR
	if (!TwEventMouseWheelGLFW3(window, xoffset, yoffset)) {
		renderer->scroll_event((float)xoffset, (float)yoffset);
	}
#else
  renderer->scroll_event(xoffset, yoffset );
#endif
}


void Viewer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods ) {
	bool handled = false;
#ifdef WITH_ANTTWEAKBAR
	handled = TwEventMouseButtonGLFW3(window, button, action, mods) != 0;	 
#endif //  WITH_ANTTWEAKBAR
	// this is called unconditional, as it should update the keys atleast
  renderer->mouse_key_event(button, action, (unsigned char)mods, handled);
}

void Viewer::key_callback( GLFWwindow* window, 
                           int key, int scancode, int action, int mods ) {

	// let AntTweakBar handle the events first 
#ifdef WITH_ANTTWEAKBAR
	TwEventKeyGLFW3(window, key, scancode, action, mods);
#endif

  if (action == GLFW_PRESS) {
    if( key == GLFW_KEY_ESCAPE ) { 
      glfwSetWindowShouldClose( window, true ); 
    } else if( key == GLFW_KEY_GRAVE_ACCENT ){
      showInfo = !showInfo;
    } 
  }
  
  renderer->keyboard_event( key, action, (unsigned char)mods );
}

// Callback function called by GLFW on char event, maybe need this for AntTweakBar
void Viewer::on_char_callback(GLFWwindow* window, unsigned int codepoint) {
	// using custom AntTweakBar which has updated funtions for GLFW3
	TwEventCharGLFW3(window, codepoint);

}



} // namespace CMU462

