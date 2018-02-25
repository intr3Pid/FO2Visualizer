#ifndef CMU462_H
#define CMU462_H

#include "misc.h"
/* for M_PI */
#ifdef _MSC_VER
#define _USE_MATH_DEFINES 
#include <math.h>
#endif

// CMU462 Forward Declarations //

namespace CMU462 {

class Vector2D;
class Vector3D;
class Vector4D;

class Matrix3x3;
class Matrix4x4;

class Quaternion;
class Complex;

class Color;
class Spectrum;

class Renderer;
class OSDText;
class Viewer;
class Timer;

} // namespace CMU462

#endif // CMU462_H
