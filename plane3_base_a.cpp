//|___________________________________________________________________
//!
//! \file plane3_base.cpp
//!
//! \brief Base source code for the third plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard inputs for plane and propeller (subpart):
//! ------------------------------------------------------> Plane 2 飞机2
//!   i   = moves the plane forward    i 移动飞机向前
//!   k   = moves the plane backward    k 移动飞机向后
//!   q,e = rolls the plane     q,e 旋转飞机
//!   a,d = yaws the plane     a,d 飞机偏航
//!   x,s = pitches the plane     x,s 飞机俯角
//! ------------------------------------------------------> Plane 1 飞机1
//!   I   = moves the plane forward    I 移动飞机向前
//!   K   = moves the plane backward    K 移动飞机向后
//!   Q,E = rolls the plane     Q,E 旋转飞机
//!   A,D = yaws the plane     A,D 飞机偏航
//!   X,S = pitches the plane     X,S 飞机俯角
//! ------------------------------------------------------> Plane Propeller 飞机配件
//!   r   = Rotates right propeller    r 旋转右边螺旋桨
//!   y   = Rotates left propeller    y 旋转左边螺旋桨
//!   t   = Rotates Front propeller    t 旋转前边螺旋桨
//!   g   = Rotates Subsubpropeller1    g 旋转螺旋桨配件1
//!   h   = Rotates Subsubpropeller2    h 旋转螺旋桨配件2
//!   H   = Rotates Subsubpropeller3    H 旋转螺旋桨配件3
//! ------------------------------------------------------> Select Camera 选择相机
//!   v   = Select camera to view     v 选择相机视角
//!   b   = Select camera to control    b 选择相机控制
//! ------------------------------------------------------> Light Control 光源控制
//!		1,3 = translates light up/down (+y/-y)    1,3  调整光源上下
//!		2,4 = translates light (+z/-z)     2,4  调整光源前后
//!		5,6 = translates light (+x/-x)     2,4  调整光源左右
//!		9   = toggles diffuse light on/off      9  打开/关闭漫射光
//!		8   = toggles specular light on/off      8  打开/关闭镜面反射光
//!		7   = toggles ambient light on/off      7打开/关闭环境光
//! ------------------------------------------------------> Mouse Control 鼠标控制
//!    Mouse inputs for world-relative camera:
//!    Hold left button and drag  = controls azimuth and elevation   
//!    按住鼠标左键控制相机的方位角和仰角
//! 
//!   (Press CTRL (and hold) before left button to restrict to azimuth control only,
//!    Press SHIFT (and hold) before left button to restrict to elevation control only)  
//!   (按住 CTRL 并 拖动鼠标左键 可以实现相机方位角的固定旋转
//!    按住 SHIFT 并 拖动鼠标左键 可以实现相机仰角的固定旋转)
//!  
//!    Hold right button and drag = controls distance                
//!    按住鼠标右键控制相机的缩放距离
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

#define _CRT_SECURE_NO_WARNINGS

//|___________________
//|
//| Includes
//|___________________

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>
#include <gmtl/gmtl.h>
#include <GL/glut.h>

//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH = 3.0f;
const float P_LENGTH = 3.0f;
const float P_HEIGHT = 1.5f;

// Plane transforms
const gmtl::Vec3f PLANE_FORWARD(0, 0, 1.0f);            // Plane's forward translation vector (w.r.t. local frame)
const float PLANE_ROTATION = 5.0f;                      // Plane rotated by 5 degs per input

// Propeller dimensions (subpart)
const float PP_WIDTH = 0.25f;
const float PP_LENGTH = 1.5f;

// Propeller transforms
const gmtl::Point3f PROPELLER_POS(5, 0.25, 5.5);     // Propeller0 position on the plane (w.r.t. plane's frame) 
const gmtl::Point3f PROPELLER_POS1(-5, 0.25, 5.5);     // Propeller1 position on the plane (w.r.t. plane's frame) 
const gmtl::Point3f PROPELLER_POS2(0, 0, 0);         // Propeller2 position on the plane (w.r.t. plane's frame)
//const gmtl::Point3f PROPELLER_POS3(P_WIDTH / 2, 0, 0);         // Propeller2 position on the plane (w.r.t. plane's frame)
const float PROPELLER_ROTATION = 5.0f;               // Propeller rotated by 5 degs per input

// Camera's view frustum
const float CAM_FOV = 90.0f;                     // Field of view in degs

// Keyboard modifiers
enum KeyModifier { KM_SHIFT = 0, KM_CTRL, KM_ALT };

// Textures
enum TextureID { TID_SKYBACK = 0, TID_SKYLEFT, TID_SKYBOTTOM, TID_SKYTOP, TID_SKYFRONT, TID_SKYRIGHT, TEXTURE_NB };  // Texture IDs, with the last ID indicating the total number of textures

// Skybox
const float SB_SIZE = 1000.0f;                     // Skybox dimension

// Lighting 可以调整光的大小
const GLfloat NO_LIGHT[] = { 0.0, 0.0, 0.0, 1.0 };
const GLfloat AMBIENT_LIGHT[] = { 0.2, 0.2, 0.2, 1.0 };
const GLfloat DIFFUSE_LIGHT[] = { 0.7, 0.7, 0.7, 1.0 };
const GLfloat SPECULAR_LIGHT[] = { 0.7, 0.7, 0.7, 1.0 };

/* test1
const GLfloat NO_LIGHT[] = { 0.0, 0.0, 0.0, 1.0 };
const GLfloat AMBIENT_LIGHT[] = { 0.1, 0.1, 0.1, 1.0 };
const GLfloat DIFFUSE_LIGHT[] = { 0.5, 0.5, 0.5, 1.0 };
const GLfloat SPECULAR_LIGHT[] = { 0.5, 0.5, 0.5, 1.0 };
*/

// Materials 使用到的颜色 都在这里，在后面可以直接使用
const GLfloat DARKRED_COL[] = { 0.1, 0.0, 0.0, 1.0 };
const GLfloat BRIGHTRED_COL[] = { 0.7, 0.0, 0.0, 1.0 };
const GLfloat DARKBLUE_COL[] = { 0.0, 0.0, 0.1, 1.0 };
const GLfloat BRIGHTBLUE_COL[] = { 0.0, 0.0, 0.7, 1.0 };
const GLfloat BLUE_COL[] = { 0.0, 0.3, 0.1, 1.0 };
const GLfloat DARK_COL[] = { 0.1, 0.1, 0.1, 1.0 };
const GLfloat MEDIUMWHITE_COL[] = { 0.7, 0.7, 0.7, 1.0 };
const GLfloat SPECULAR_COL[] = { 0.7, 0.7, 0.7, 1.0 };
const GLfloat GREEN_COL[] = { 0.55,0.95,0.64,1.0 };
const GLfloat DARKGREEN_COL[] = { 0.0, 0.1, 0.0, 1.0 };
const GLfloat DARKGRAY_COL[] = { 0.26, 0.26, 0.26, 1.0 };
const GLfloat GRAY_COL[] = { 0.75, 0.75, 0.75, 1.0 };
const GLfloat WHITE_COL[] = { 1.0, 1.0, 1.0, 1.0 };
const GLfloat PASTEL_COL[] = { 1.0, 0.69, 0.94, 1.0 };
const GLfloat PASTELDARKER_COL[] = { 0.90, 0.59, 0.69, 1.0 };
const GLfloat RED_COL[] = { 0.90, 0.00, 0.00, 1.0 };
/*
// Sphere Constant
const int space = 10;
const int VertexCount = (90 / space) * (360 / space) * 4;
*/

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width = 800;
int w_height = 600;

// Plane 1 pose (position-quaternion pair)  //飞机1
gmtl::Point4f plane_p;      // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q;        // Quaternion

// Plane 2 pose  (position-quaternion pair) //飞机2
gmtl::Point4f plane1_p;		// Position
gmtl::Quatf plane1_q;		// Quaternion

// Quaternions to rotate plane
gmtl::Quatf zrotp_q;        // Positive and negative Z rotations
gmtl::Quatf zrotn_q;

gmtl::Quatf xrotp_q;		// Positive and negative X rotations
gmtl::Quatf xrotn_q;

gmtl::Quatf yrotp_q;		// Positive and negative Y rotations
gmtl::Quatf yrotn_q;


// Propeller rotation (subpart)
float pp_angle = 0;         // Rotation angle 
float pp_angle_l = 0;         // Rotation front angle
float pp_angle_t = 0;         // Rotation left angle
float pp_angle_ss = 0;         // Rotation subpropeller1 angle
float pp_angle_ss2 = 0;         // Rotation subpropeller2 angle
float pp_angle_ss3 = 0;         // Rotation subpropeller3 angle

// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3] = { false, false, false };
bool kmodifiers[3] = { false, false, false };

// Cameras
int cam_id = 0;                                // Selects which camera to view
int camctrl_id = 0;                                // Selects which camera to control

float distance[3] = { 5.0f, 5.0f, 5.0f };     // Distance of the camera from world's origin. 浮动距离
float elevation[3] = { -45.0f, 0.0f, -15.0f }; // Elevation of the camera. (in degs) 浮动高程(海拔高度)
float azimuth[3] = { 15.0,  -45.0f, -10.0f }; // Azimuth of the camera. (in degs) 浮动方位角

/*
float distance[3] = { 20.0f,  20.0f,  20.0f };                 // Distance of the camera from world's origin.
float elevation[3] = { -55.0f, -45.0f, -45.0f };                 // Elevation of the camera. (in degs)
float azimuth[3] = { 10.0f,  15.0f,  20.0f };                 // Azimuth of the camera. (in degs)
*/


// Lighting
gmtl::Point4f light_pos(0.0, 20.0, 20.0, 1.0);
bool is_diffuse_on = true;
bool is_ambient_on = true;
bool is_specular_on = true;

// Textures  //需要添加的材质名字
GLuint textures[TEXTURE_NB];                           // Textures
GLuint satellite_wing;
GLuint cloud_texture;
GLuint flower_texture;
GLuint plane_wing2;

/*//Sphere Vertices
typedef struct
{
	int X;
	int Y;
	int Z;
	double U;
	double V;
}VERTICES;
VERTICES SphereVertices[VertexCount];
*/

//|___________________
//|
//| Function Prototypes
//|___________________

gmtl::Vec3f FindNormal(const gmtl::Point3f& p1, const gmtl::Point3f& p2, const gmtl::Point3f& p3);
void InitTransforms();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void ReshapeFunc(int w, int h);

void DrawCoordinateFrame(const float l);

void DrawPlaneBody(const float width, const float length, const float height);
void DrawPlaneBody2(const float width, const float length, const float height);

void DrawPropeller(const float width, const float length);
void DrawPropeller2(const float width, const float length);

void DrawSubPropeller(const float width, const float length);
void DrawSubPropeller2(const float width, const float length);

void drawSatellite();
void DrawSkybox(const float s);
void SetLight(const gmtl::Point4f& pos, const bool is_ambient, const bool is_diffuse, const bool is_specular);
void drawCamera(const float radius, const float height);
void drawLight(const float radius);

void LoadPPM(const char* fname, unsigned int* w, unsigned int* h, unsigned char** data, const int mallocflag);

//|____________________________________________________________________
//|
//| Function: FindNormal
//|
//! \param p1	[in] Point 1.
//! \param p2	[in] Point 2.
//! \param p3	[in] Point 3.
//! \return Normalized surface normal.
//!
//! Finds the surface normal of a triangle. The input must be in CCW order.
//|____________________________________________________________________

gmtl::Vec3f FindNormal(const gmtl::Point3f& p1,
	const gmtl::Point3f& p2,
	const gmtl::Point3f& p3)
{
	gmtl::Vec3f v12 = p2 - p1;
	gmtl::Vec3f v13 = p3 - p1;

	gmtl::Vec3f normal;
	gmtl::cross(normal, v12, v13);
	gmtl::normalize(normal);

	return normal;
}

//|____________________________________________________________________
//|
//| Function: InitTransforms
//|
//! \param None.
//! \return None.
//!
//! Initializes all the transforms
//|____________________________________________________________________

void InitTransforms()
{
	const float COSTHETA_D2 = cos(gmtl::Math::deg2Rad(PLANE_ROTATION / 2));  // cos() and sin() expect radians
	const float SINTHETA_D2 = sin(gmtl::Math::deg2Rad(PLANE_ROTATION / 2));

	// Inits plane2 pose  //飞机2的初始位置
	plane_p.set(1.0f, 0.0f, 4.0f, 1.0f);
	plane_q.set(0, 0, 0, 1);

	// Inits plane1 pose  //飞机1的初始位置
	plane1_p.set(20.0f, 15, -40.0f, 1.0f);
	plane1_q.set(0, 0, 0, 1);

	/*// Inits plane2 pose
    plane_p2.set(0.0f, 5, 20.0f, 1.0f);
	plane_q2.set(0, 0, 0, 1);
	*/

	// Z rotations (roll)
	zrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
	zrotn_q = gmtl::makeConj(zrotp_q);                // -Z

	// X rotation (pitch)
	xrotp_q.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X
	xrotn_q = gmtl::makeConj(xrotp_q);				  // -X

	// Y rotation (yaw)
	yrotp_q.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
	yrotn_q = gmtl::makeConj(yrotp_q);

}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
	unsigned char* img_data;               // Texture image data
	unsigned int  width;                   // Texture width
	unsigned int  height;                  // Texture height

	glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	//|___________________________________________________________________
	//|
	//| Setup lighting
	//|___________________________________________________________________

	  // Disable global ambient
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, NO_LIGHT);

	// NOTE: for specular reflections, the "local viewer" model produces better
	// results than the default, but is slower. The default would not use the correct
	// vertex-to-eyepoint vector, treating it as always parallel to Z.
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

	// Disable two sided lighting because we cannot see the inside of the most object in the scene (except satellite)
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

	// Enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	//|___________________________________________________________________
	//|
	//| Setup texturing
	//|___________________________________________________________________

	  // Describe how data will be stored in memory
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	// Select the method for combining texture color with the lighting equation
	  // (look up the third parameter)
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// Generate and setup texture objects
	glGenTextures(TEXTURE_NB, textures);

	// Skybox back wall 天空盒子后方
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBACK]);
	LoadPPM("skybox_back.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox left wall 天空盒子左方
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYLEFT]);
	LoadPPM("skybox_left.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox buttom wall 天空盒子下方
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBOTTOM]);
	LoadPPM("skybox_bottom.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// TODO: Initializes the remaining textures

	// Skybox front wall 天空盒子前方
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYFRONT]);
	LoadPPM("skybox_front.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox right wall 天空盒子右方
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYRIGHT]);
	LoadPPM("skybox_right.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox top wall 天空盒子上方
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYTOP]);
	LoadPPM("skybox_top.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Wings Texture1 机翼贴图
	glGenTextures(1, &cloud_texture);
	glBindTexture(GL_TEXTURE_2D, cloud_texture);
	LoadPPM("cloud_texture.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Wings Texture1 机翼贴图1
	glGenTextures(1, &flower_texture);
	glBindTexture(GL_TEXTURE_2D, flower_texture);
	LoadPPM("flower_texture.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Wings Texture2 机翼贴图2
	glBindTexture(GL_TEXTURE_2D, plane_wing2);
	LoadPPM("wings_texture2.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Satellite Wing Texture  卫星贴图
	glGenTextures(1, &satellite_wing);
	glBindTexture(GL_TEXTURE_2D, satellite_wing);
	LoadPPM("satellite_wing.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	/*
	glGenTextures          是用来生成纹理的函数
	glBindTexture          允许建立一个绑定到目标纹理的有名称的纹理
	glTexImage2D          功能是根据指定的参数，生成一个2D纹理
	glTexParameteri          (纹理过滤函数),图象从纹理图象空间映射到帧缓冲图象空间(映射需要重新构造纹理图像,
	这样就会造成应用到多边形上的图像失真),这时就可用glTexParmeteri()函数来确定如何把纹理象素映射成像素.
	GL_UNSIGNED_BYTE          是一个整数常量，常用于指示您将在指针中传递GLubyte。sizeof(GLubyte)根据定义总是1
	GL_LINEAR         也叫线性过滤,(Bi)linear Filtering)它会基于纹理坐标附近的纹理像素,计算出一个插值,近似出这些纹理像素之间的颜色, 
	一个纹理像素的中心距离纹理坐标越近,那么这个纹理像素的颜色对最终的样本颜色的贡献越大
	*/
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
	gmtl::AxisAnglef aa;    // Converts plane2's quaternion to axis-angle form to be used by glRotatef()
	gmtl::AxisAnglef aa2;    // Converts plane1's quaternion to axis-angle form to be used by glRotatef()
	gmtl::Vec3f axis;       // Plane2 Axis component of axis-angle representation
	gmtl::Vec3f axis2;       // Plane1 Axis component of axis-angle representation
	float angle;            // Plane2 Angle component of axis-angle representation
	float angle2;            // Plane1 Angle2 component of axis-angle representation

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(CAM_FOV, (float)w_width / w_height, 0.1f, 1000.0f);     // Check MSDN: google "gluPerspective msdn"

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//|____________________________________________________________________
	//|
	//| Setting up view transform by:
	//| "move up to the world frame by composing all of the (inverse) transforms from the camera up to the world node"
	//|____________________________________________________________________

	switch (cam_id) {
	case 0:
		// For the world-relative camera 坐标轴的摄像机
		glTranslatef(0, 0, -distance[0]);
		glRotatef(-elevation[0], 1, 0, 0);
		glRotatef(-azimuth[0], 0, 1, 0);
		break;

	case 1:
		// For plane2's camera 飞机2的摄像机
		glTranslatef(0, 0, -distance[1]);
		glRotatef(-elevation[1], 1, 0, 0);
		glRotatef(-azimuth[1], 0, 1, 0);

		gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
		axis = aa.getAxis();
		angle = aa.getAngle();
		glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
		glTranslatef(-plane_p[0], -plane_p[1], -plane_p[2]);
		break;

		// TODO: Add case for the plane1's camera

	case 2:
		// For plane1's camera 飞机1的摄像机
		glTranslatef(0, 0, -distance[2]);
		glRotatef(-elevation[2], 1, 0, 0);
		glRotatef(-azimuth[2], 0, 1, 0);

		gmtl::set(aa2, plane1_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
		axis2 = aa2.getAxis();
		angle2 = aa2.getAngle();
		glRotatef(-gmtl::Math::rad2Deg(angle2), axis2[0], axis2[1], axis2[2]);
		glTranslatef(-plane1_p[0], -plane1_p[1], -plane1_p[2]);
		break;
	}

	//|____________________________________________________________________
	//|
	//| Draw traversal begins, start from world (root) node
	//|____________________________________________________________________

	  // Set light position wrt world
	SetLight(light_pos, is_ambient_on, is_diffuse_on, is_specular_on);
	// DrawLight
	glPushMatrix();
	glTranslatef(light_pos[0], light_pos[1], light_pos[2]);
	drawLight(0.5);
	glPopMatrix();

	// World node: draws world coordinate frame
	DrawCoordinateFrame(10);
	DrawSkybox(SB_SIZE);

	// World-relative camera:
	//摄像机1
	if (cam_id != 0) {
		glPushMatrix();
		glRotatef(azimuth[0], 0, 1, 0);
		glRotatef(elevation[0], 1, 0, 0);
		glTranslatef(0, 0, distance[0]);
		drawCamera(0.5, 1);
		DrawCoordinateFrame(1);
		glPopMatrix();
	}
	// Plane 2 body:
	glPushMatrix();
	gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	axis = aa.getAxis();
	angle = aa.getAngle();
	glTranslatef(plane_p[0], plane_p[1], plane_p[2]);
	glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
	DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(3);

	// Plane 2's camera:
	//摄像机2
	glPushMatrix();
	glRotatef(azimuth[1], 0, 1, 0);
	glRotatef(elevation[1], 1, 0, 0);
	glTranslatef(0, 0, distance[1]);
	DrawCoordinateFrame(3);
	glPopMatrix();

	//right Propeller (subpart):
	glPushMatrix();
	glTranslatef(PROPELLER_POS1[0], PROPELLER_POS1[1], PROPELLER_POS1[2]);     // Positions right propeller on the plane
	glRotatef(pp_angle, 0, 1, 0);                                           // Rotates propeller
	DrawPropeller(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(2);
	glPopMatrix();

	//left Propeller (subpart):
	glPushMatrix();
	glTranslatef(PROPELLER_POS[0], PROPELLER_POS[1], PROPELLER_POS[2]);     // Positions left propeller on the plane
	glRotatef(-pp_angle_t, 0, 1, 0);                                           // Rotates propeller
	DrawPropeller(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(2);
	glPopMatrix();

	//Front Propeller (subpart):
	glPushMatrix();
	glTranslatef(PROPELLER_POS2[0], PROPELLER_POS2[1], PROPELLER_POS2[2]);     // Positions front propeller on the plane
	glRotatef(pp_angle_l, 0, 0, 1);                                           // Rotates propeller
	DrawPropeller2(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(2);
	glPopMatrix();

	// Subsubpart1:
	glPushMatrix();
	glTranslatef(PROPELLER_POS2[0], PROPELLER_POS2[1], PROPELLER_POS2[2]);                                        // Positions subpropeller on the plane
	glRotatef(pp_angle_ss, 0, 0, 1);
	DrawSubPropeller(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(1);

	// Subsubpart2:
	glPushMatrix();
	glTranslatef(PROPELLER_POS2[0], PROPELLER_POS2[1], PROPELLER_POS2[2]);                                        // Positions subpropeller on the plane
	glRotatef(pp_angle_ss2, 0, 1, 0);
	DrawSubPropeller2(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(1);
    glPopMatrix();
	glPopMatrix();
	glPopMatrix();

	// Plane 1 body:
	glPushMatrix();
	gmtl::set(aa, plane1_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	axis = aa.getAxis();
	angle = aa.getAngle();
	glTranslatef(plane1_p[0], plane1_p[1], plane1_p[2]);
	glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
	DrawPlaneBody2(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(3);

	// Plane 1's camera:
	//摄像机3
	glPushMatrix();
	glRotatef(azimuth[2], 0, 1, 0);
	glRotatef(elevation[2], 1, 0, 0);
	glTranslatef(0, 0, distance[2]);
	DrawCoordinateFrame(1);
	glPopMatrix();

	//Satellite1
	glPushMatrix();
	glTranslatef(20, 20, 40);
	glRotatef(pp_angle_ss3, 0, 1, 0);
	drawSatellite();
	glPopMatrix();

	glutSwapBuffers();                          // Replaces glFlush() to use double buffering
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param key    [in] Key code.
//! \param x      [in] X-coordinate of mouse when key is pressed.
//! \param y      [in] Y-coordinate of mouse when key is pressed.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
	switch (key) {
		//|____________________________________________________________________
		//|
		//| Camera switch
		//|____________________________________________________________________

	case 'v': // Select camera to view
		cam_id = (cam_id + 1) % 3;
		printf("View camera = %d\n", cam_id);
		break;
	case 'b': // Select camera to control
		camctrl_id = (camctrl_id + 1) % 3;
		printf("Control camera = %d\n", camctrl_id);
		break;

		//|____________________________________________________________________
		//|
		//| Plane controls
		//|____________________________________________________________________

	case 'I': { // Forward translation of the plane (+Z translation)  
		gmtl::Quatf v_q = plane1_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane1_q);
		plane1_p = plane1_p + v_q.mData;
	} break;
	case 'K': { // Backward translation of the plane (-Z translation)
		gmtl::Quatf v_q = plane1_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane1_q);
		plane1_p = plane1_p + v_q.mData;
	} break;

	case 'E': // Rolls the plane (+Z rot) //逆时针旋转飞机
		plane1_q = plane1_q * zrotp_q;
		break;
	case 'Q': // Rolls the plane (-Z rot) //顺时针旋转飞机
		plane1_q = plane1_q * zrotn_q;
		break;

	case 'X': // Pitches the plane (+X rot) //飞机仰角
		plane1_q = plane1_q * xrotp_q;
		break;
	case 'S': // Pitches the plane (-X rot) //飞机俯角
		plane1_q = plane1_q * xrotn_q;
		break;

	case 'D': // Yaws the plane (+Y rot)  // 左转飞机
		plane1_q = plane1_q * yrotp_q;
		break;
	case 'A': // Yaws the plane (-Y rot)  // 右转飞机
		plane1_q = plane1_q * yrotn_q;
		break;


		//飞机2控制部分
	case 'i': { // Forward translation of the plane (+Z translation)  
		gmtl::Quatf v_q = plane_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
		plane_p = plane_p + v_q.mData;
	} break;
	case 'k': { // Backward translation of the plane (-Z translation)
		gmtl::Quatf v_q = plane_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
		plane_p = plane_p + v_q.mData;
	} break;

	case 'e': // Rolls the plane (+Z rot) //逆时针旋转飞机
		plane_q = plane_q * zrotp_q;
		break;
	case 'q': // Rolls the plane (-Z rot) //顺时针旋转飞机
		plane_q = plane_q * zrotn_q;
		break;

	case 'x': // Pitches the plane (+X rot) //飞机仰角
		plane_q = plane_q * xrotp_q;
		break;
	case 's': // Pitches the plane (-X rot) //飞机俯角
		plane_q = plane_q * xrotn_q;
		break;

	case 'd': // Yaws the plane (+Y rot)  // 左转飞机
		plane_q = plane_q * yrotp_q;
		break;
	case 'a': // Yaws the plane (-Y rot)  // 右转飞机
		plane_q = plane_q * yrotn_q;
		break;

			//|____________________________________________________________________
			//|
			//| Propeller controls (subpart)
			//|____________________________________________________________________

	case 'r': // Rotates right propeller 
		pp_angle += PROPELLER_ROTATION;
		break;

	case 't': // Rotates front propeller 
		pp_angle_l += PROPELLER_ROTATION;
		break;

	case 'y': // Rotates left propeller 
		pp_angle_t += PROPELLER_ROTATION;
		break;

	case 'g': // Rotates subsubpart1 (propeller)
		pp_angle_ss += -PROPELLER_ROTATION;
		break;

	case 'h': // Rotates subsubpart2 (propeller)
		pp_angle_ss2 += -PROPELLER_ROTATION;
		break;

	case 'H': // Rotates subsubpart3 (propeller)
		pp_angle_ss3 += -PROPELLER_ROTATION;
		break;

		//|____________________________________________________________________
		//|
		//| Lighting controls
		//|____________________________________________________________________

	case '1': // Light up (+Y translation)
		light_pos[1]++;
		printf("Light-Y = %.2f\n", light_pos[1]);
		break;
	case '3': // Light down (-Y translation)
		light_pos[1]--;
		printf("Light-Y = %.2f\n", light_pos[1]);
		break;
	case '4': // Light down (+Z translation)
		light_pos[2]++;
		break;
	case '2': // Light down (-Z translation)
		light_pos[2]--;
		break;
	case '5': // Light down (+X translation)
		light_pos[0]--;
		break;
	case '6': // Light down (-X translation)
		light_pos[0]++;
		break;
	case '9': // Toggles diffuse light ON/OFF
		is_diffuse_on = !is_diffuse_on;
		printf("Light-diffuse = %s\n", is_diffuse_on ? "ON" : "OFF");
		break;
	case '8': // Toggles Specular light ON/OFF
		is_specular_on = !is_specular_on;
		printf("Light-Specular = %s\n", is_specular_on ? "ON" : "OFF");
		break;
	case '7':// Toggles Ambient light ON/OFF
		is_ambient_on = !is_ambient_on;
		printf("Light-Ambient = %s\n", is_ambient_on ? "ON" : "OFF");
		break;
	}

	glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MouseFunc
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void MouseFunc(int button, int state, int x, int y)
{
	int km_state;

	// Updates button's sate and mouse coordinates
	if (state == GLUT_DOWN) {
		mbuttons[button] = true;
		mx_prev = x;
		my_prev = y;
	}
	else {
		mbuttons[button] = false;
	}

	// Updates keyboard modifiers
	km_state = glutGetModifiers();
	kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
	kmodifiers[KM_CTRL] = km_state & GLUT_ACTIVE_CTRL ? true : false;
	kmodifiers[KM_ALT] = km_state & GLUT_ACTIVE_ALT ? true : false;

	//glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MotionFunc
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void MotionFunc(int x, int y)
{
	int dx, dy, d;

	if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON]) {
		// Computes distances the mouse has moved
		dx = x - mx_prev;
		dy = y - my_prev;

		// Updates mouse coordinates
		mx_prev = x;
		my_prev = y;

		// Hold left button to rotate camera
		if (mbuttons[GLUT_LEFT_BUTTON]) {
			if (!kmodifiers[KM_CTRL]) {
				elevation[camctrl_id] += dy;            // Elevation update
			}
			if (!kmodifiers[KM_SHIFT]) {
				azimuth[camctrl_id] += dx;             // Azimuth update
			}
		}

		// Hold right button to zoom
		if (mbuttons[GLUT_RIGHT_BUTTON]) {
			if (abs(dx) >= abs(dy)) {
				d = dx;
			}
			else {
				d = -dy;
			}
			distance[camctrl_id] += d;
		}

		glutPostRedisplay();      // Asks GLUT to redraw the screen
	}
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
	// Track the current window dimensions
	w_width = w;
	w_height = h;
	glViewport(0, 0, (GLsizei)w_width, (GLsizei)w_height);
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);
	// X axis is red
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(l, 0.0f, 0.0f);

	// Y axis is green
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, l, 0.0f);

	// Z axis is blue
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, l);
	glEnd();

	glEnable(GL_LIGHTING);
}

//|____________________________________________________________________
//|
//| Function: DrawPlaneBody
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws a plane body.
//|____________________________________________________________________


	/*绘制飞机
	1,飞机尾部,上下两个部分 蓝白
	2,飞机尾翼,左右两个部分 白
	3,飞机机头,红黑
	4,飞机机舱,蓝色
	5,飞机机身,红黑
	6,飞机机翼,红色
	7,飞机后翼.黄色

	glBegin(GL_POLYGON); 多边形
	glBegin(GL_QUADS); 四边形
	glBegin(GL_TRIANGLES); 三角形
	glEnd(); 结束

	GL_SHININESS 该属性只有一个值,称为“镜面指数”,取值范围是0到128。该值越小,表示材质越粗糙,
	点光源发射的光线照射到上面,也可以产生较大的亮点。该值越大,表示材质越类似于镜面,光源照射到上面后,产生较小的亮点。
	GL_FRONT_AND_BACK 表示剔除正向面和背向面
	GL_AMBIENT 表示各种光线照射到该材质上,经过很多次反射后最终遗留在环境中的光线强度(颜色)
	GL_DIFFUSE 表示光线照射到该材质上,经过漫反射后形成的光线强度(颜色)
	GL_SPECULAR 表示光线照射到该材质上,经过镜面反射后形成的光线强度(颜色)

	Color I used：
	DARKRED_COL[] = { 0.1, 0.0, 0.0, 1.0 };
    BRIGHTRED_COL[] = { 0.7, 0.0, 0.0, 1.0 };
	BLUE_COL[] = { 0.0, 0.3, 0.1, 1.0 };
    DARKBLUE_COL[] = { 0.0, 0.0, 0.1, 1.0 };
    BRIGHTBLUE_COL[] = { 0.0, 0.0, 0.7, 1.0 };
    DARK_COL[] = { 0.1, 0.1, 0.1, 1.0 };
    MEDIUMWHITE_COL[] = { 0.7, 0.7, 0.7, 1.0 };
    SPECULAR_COL[] = { 0.7, 0.7, 0.7, 1.0 };
    GREEN_COL[] = { 0.55,0.95,0.64,1.0 };
    DARKGREEN_COL[] = { 0.0, 0.1, 0.0, 1.0 };
    DARKGRAY_COL[] = { 0.26, 0.26, 0.26, 1.0 };
    GRAY_COL[] = { 0.75, 0.75, 0.75, 1.0 };
    WHITE_COL[] = { 1.0, 1.0, 1.0, 1.0 };
    PASTEL_COL[] = { 1.0, 0.69, 0.94, 1.0 };
    PASTELDARKER_COL[] = { 0.90, 0.59, 0.69, 1.0 };
    RED_COL[] = { 0.90, 0.00, 0.00, 1.0 };

	// Turn on texture mapping and disable lighting
	//glEnable(GL_TEXTURE_2D);
	//glDisable(GL_LIGHTING); 可以去掉光

	How to add texture如何加贴图
	glBindTexture(GL_TEXTURE_2D, plane_wing );
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(xf, yf, zf);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(xf, yf, zf);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(xf, yf, zf);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(xf, yf, zf);

	Draw circular cone (圆锥)
    N---> Circular cone smoothness N = integer 平滑度(取整数)
    H---> Hight 高
    R---> Radius 半径
    PI----> π
    底面积(Lateral area) =π*r^2=π*1=π
    侧面积(Bottom area) =π*r*l=π*1*2=2π
	*/
	

//飞机1
void DrawPlaneBody(const float width, const float length, const float height)
{
	float w = width / 2;
	float l = length / 2;


	glBegin(GL_POLYGON);
	//1
	//竖着的下部分
	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 15.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, WHITE_COL); 
	glColor3f(1.0f, 1.0f, 1.0f);//颜色是白色white
	glVertex3f(0.0f, 1.0f, 0.5f);//
	glVertex3f(0.0f, 0.25f, 1.0f);//
	glVertex3f(0.0f, 0.5f, 4.0f);//
	glVertex3f(0.0f, 1.4f, 2.0f);//
	glColor3f(1.0f, 1.0f, 1.0f);//颜色是白色white

	//竖着的上部分
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTBLUE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, WHITE_COL);
	glColor3f(0.0f, 3.0f, 1.0f);//颜色是蓝色blue
	glVertex3f(0.0f, 1.4f, 2.0f);//
	glVertex3f(0.0f, 3.0f, 1.0f);//
	glVertex3f(0.0f, 3.0f, 0.0f);//
	glVertex3f(0.0f, 1.0f, 0.5f);//
	glColor3f(0.0f, 3.0f, 1.0f);//颜色是蓝色blue
	glEnd();

	//2
	glBegin(GL_POLYGON);
	//横着的右半部分
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, WHITE_COL);
	glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
	glVertex3f(0.0f, 2.0f, 0.25f);//
	glVertex3f(1.0f, 2.0f, -0.5f);//
	glVertex3f(2.0f, 2.0f, -0.5f);//
	glVertex3f(0.0f, 2.0f, 1.40f);//

	//横着的左半部分
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, WHITE_COL);
	glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
	glVertex3f(0.0f, 2.0f, 0.25f);//
	glVertex3f(-1.0f, 2.0f, -0.5f);//
	glVertex3f(-2.0f, 2.0f, -0.5f);//
	glVertex3f(0.0f, 2.0f, 1.40f);//
	glEnd();

	//3
	//机头部分
	glBegin(GL_TRIANGLES);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, RED_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
	glVertex3f(-1.0f, 0.5f, 10.0f);//
	glVertex3f(1.0f, 0.5f, 10.0f);//
	glVertex3f(0.0f, 0.0f, 13.5f);//
	glEnd();
	glBegin(GL_TRIANGLES);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, GREEN_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
	glVertex3f(1.0f, 0.5f, 10.0f);//
	glVertex3f(2.0f, -0.2f, 10.0f);//
	glVertex3f(0.0f, 0.0f, 13.5f);//
	glEnd();
	glBegin(GL_TRIANGLES);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, RED_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
	glVertex3f(2.0f, -0.2f, 10.0f);//
	glVertex3f(-2.0f, -0.2f, 10.0f);//
	glVertex3f(0.0f, 0.0f, 13.5f);//
	glEnd();
	glBegin(GL_TRIANGLES);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
	glVertex3f(-2.0f, -0.2f, 10.0f);//
	glVertex3f(-1.0f, 0.5f, 10.0f);//
	glVertex3f(0.0f, 0.0f, 13.5f);//
	glEnd();

    //4
	//飞机驾驶舱
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 110.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(0.0f, 0.5f, 10.0f);//
	glVertex3f(0.25f, 1.5f, 9.25f);//
	glVertex3f(-0.25f, 1.5f, 9.25f);//
	glEnd();
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 110.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(0.0f, 0.5f, 10.0f);//
	glVertex3f(0.25f, 1.5f, 9.25f);//
	glVertex3f(1.0f, 0.5f, 9.0f);//
	glEnd();
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 110.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(0.0f, 0.5f, 10.0f);//
	glVertex3f(-0.25f, 1.5f, 9.25f);//
	glVertex3f(-1.0f, 0.5f, 9.0f);//
	glEnd();

	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 110.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_QUADS);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(-1.0f, 0.5f, 9.0f);//
	glVertex3f(-0.25f, 1.5f, 9.25f);//
	glVertex3f(-0.25f, 1.5f, 7.75f);//
	glVertex3f(-1.0f, 0.5f, 8.0f);//
	glEnd();
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_QUADS);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(0.25f, 1.5f, 9.25f);//
	glVertex3f(-0.25f, 1.5f, 9.25f);//
	glVertex3f(-0.25f, 1.5f, 7.75f);//
	glVertex3f(0.25f, 1.5f, 7.75f);//
	glEnd();
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_QUADS);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(1.0f, 0.5f, 9.0f);//
	glVertex3f(0.25f, 1.5f, 9.25f);//
	glVertex3f(0.25f, 1.5f, 7.75f);//
	glVertex3f(1.0f, 0.5f, 8.0f);//
	glEnd();

	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 120.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(0.0f, 0.5f, 7.0f);//
	glVertex3f(0.25f, 1.5f, 7.75f);//
	glVertex3f(-0.25f, 1.5f, 7.75f);//
	glEnd();
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 120.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(0.0f, 0.5f, 7.0f);//
	glVertex3f(0.25f, 1.5f, 7.75f);//
	glVertex3f(1.0f, 0.5f, 8.0f);//
	glEnd();
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 120.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, BLUE_COL);
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
	glVertex3f(0.0f, 0.5f, 7.0f);//
	glVertex3f(-0.25f, 1.5f, 7.75f);//
	glVertex3f(-1.0f, 0.5f, 8.0f);//
	glEnd();

	//5
	//机身
	glBegin(GL_QUADS);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
	glVertex3f(-0.25f, 0.25f, 1.0f);//
	glVertex3f(0.25f, 0.25f, 1.0f);//
	glVertex3f(0.6f, 0.5f, 4.0f);//
	glVertex3f(-0.6f, 0.5f, 4.0f);//
	glEnd();
	glBegin(GL_QUADS);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
	glVertex3f(0.6f, 0.5f, 4.0f);//
	glVertex3f(-0.6f, 0.5f, 4.0f);//
	glVertex3f(-1.0f, 0.5f, 10.0f);//
	glVertex3f(1.0f, 0.5f, 10.0f);//
	glEnd();

	glBegin(GL_QUADS);

	glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
	glVertex3f(-0.5f, -0.2f, -1.0f);//
	glVertex3f(-0.25f, 0.2f, 1.0f);//
	glVertex3f(0.25f, 0.2f, 1.0f);//
	glVertex3f(0.5f, -0.2f, -1.0f);//
	glEnd();
	glBegin(GL_POLYGON);

	glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
	glVertex3f(2.0f, -0.2f, 10.0f);//
	glVertex3f(1.0f, 0.5f, 10.0f);//
	glVertex3f(0.6f, 0.5f, 4.0f);//
	glVertex3f(0.25f, 0.2f, 1.0f);//
	glVertex3f(0.5f, -0.2f, -1.0f);//
	glVertex3f(0.5f, -0.2f, -1.0f);//
	glVertex3f(1.2f, -0.2f, 4.0f);//
	glEnd();
	glBegin(GL_POLYGON);

	glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
	glVertex3f(0.0f, 0.0f, 13.5f);//
	glVertex3f(-2.0f, -0.2f, 10.0f);//
	glVertex3f(-0.5f, -0.2f, -1.0f);//
	glVertex3f(0.5f, -0.2f, -1.0f);//
	glVertex3f(2.0f, -0.2f, 10.0f);//
	glEnd();

	glBegin(GL_POLYGON);

	glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
	glVertex3f(-2.0f, -0.2f, 10.0f);//
	glVertex3f(-1.0f, 0.5f, 10.0f);//
	glVertex3f(-0.6f, 0.5f, 4.0f);//
	glVertex3f(-0.25f, 0.2f, 1.0f);//
	glVertex3f(-0.5f, -0.2f, -1.0f);//
	glVertex3f(-0.5f, -0.2f, -1.0f);//
	glVertex3f(-1.2f, -0.2f, 4.0f);//
	glEnd();

	//6
	//飞机机翼
	glBegin(GL_POLYGON);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, GREEN_COL);
	glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red 
	glVertex3f(-1.75f, 0.0f, 10.0f);//
	glVertex3f(-5.0f, 0.0f, 7.5f);//
	glVertex3f(-5.0f, 0.25f, 4.1f);//
	glVertex3f(-1.0f, 0.25f, 4.0f);//
	glVertex3f(-1.75f, 0.0f, 10.0f);//
	glEnd();
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, cloud_texture);
	glBegin(GL_POLYGON);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, GREEN_COL);
	glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-5.0f, 0.0f, 7.5f);//
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-5.0f, 0.25f, 4.1f);//
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-9.0f, 0.45f, 2.5f);//
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-10.0f, 0.2f, 5.5f);//
	glEnd();
	glBegin(GL_POLYGON);

	glBegin(GL_POLYGON);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, GREEN_COL);
	glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red 
	glVertex3f(1.75f, 0.0f, 10.0f);//
	glVertex3f(5.0f, 0.0f, 7.5f);//
	glVertex3f(5.0f, 0.25f, 4.1f);//
	glVertex3f(1.0f, 0.25f, 4.0f);//
	glVertex3f(1.75f, 0.0f, 10.0f);//
	glEnd();
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, cloud_texture);
	glBegin(GL_POLYGON);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, GREEN_COL);
	glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(5.0f, 0.0f, 7.5f);//
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(5.0f, 0.25f, 4.1f);//
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(9.0f, 0.45f, 2.5f);//
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(10.0f, 0.2f, 5.5f);//
	glEnd();

	//7
	//飞机后翼
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, cloud_texture);
	glBegin(GL_POLYGON);
	glColor3f(1.0f, 1.0f, 0.0f);//颜色是黄色yellow
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-0.5f, -0.25f, -1.0f);//
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-0.55f, 0.1f, 1.0f);//
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-2.75f, 0.1f, -0.25f);//
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-2.5f, -0.25f, -1.1f);//
	glEnd();
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, cloud_texture);
	glBegin(GL_POLYGON);
	glColor3f(1.0f, 1.0f, 0.0f);//颜色是黄色yellow
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(0.5f, -0.25f, -1.0f);//
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(0.55f, 0.1f, 1.0f);//
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(2.75f, 0.1f, -0.25f);//
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(2.5f, -0.25f, -1.1f);//
	glEnd();

}

void drawCylinder(const float radius, const float height) {
	int a;
	float x, y, xp, yp, mag;
	bool bright_col = true;

	xp = radius;
	yp = 0;
	glBegin(GL_QUADS);
	for (a = 1; a <= 360; a++) {
		x = radius * cos(gmtl::Math::deg2Rad((float)a));
		y = radius * sin(gmtl::Math::deg2Rad((float)a));

		//mag = sqrt(pow(x, 2) + pow(y, 2) + pow(height / 2, 2));
		gmtl::Vec3f normal(x, y, 0.0);
		gmtl::normalize(normal);
		glNormal3f(normal[0], normal[1], normal[2]);
		glVertex3f(xp, yp, height);
		glVertex3f(xp, yp, 0);
		glVertex3f(x, y, 0);
		glVertex3f(x, y, height);

		xp = x;
		yp = y;
	}
	glEnd();
}

//飞机2
void DrawPlaneBody2(const float width, const float length, const float height)
{
	float w = width / 2;
	float l = length / 2;

	// Sets materials 材质
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 30.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	//Plane Core 飞机2
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKGRAY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, GRAY_COL);
	glColor3ub(126, 198, 204);
	GLUquadric* sphere = gluNewQuadric();
	drawCylinder(0.5f, (height * 2) / 3);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARKGRAY_COL);
	gluSphere(sphere, 0.5f, 360, 360);
	glPushMatrix();
	glTranslatef(0, 0, (height * 2) / 3);
	gluSphere(sphere, 0.5f, 360, 360);
	glPopMatrix();

	//Plane Wings 机翼左
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glBindTexture(GL_TEXTURE_2D, flower_texture);
	glBegin(GL_QUADS);
	glColor3ub(35, 80, 119);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-4.0f, 0.0f, 0.0f);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-4.0f, 0.0f, 4.0f);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.4f);
	glEnd();
	//Plane Wings 机翼右
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glBindTexture(GL_TEXTURE_2D, flower_texture);
	glBegin(GL_QUADS);
	glColor3ub(35, 80, 119);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(4.0f, 0.0f, 0.0f);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(4.0f, 0.0f, 4.0f);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.4f);
	glEnd();


	//Plane Wings 机翼上
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glBindTexture(GL_TEXTURE_2D, plane_wing2);
	glBegin(GL_QUADS);
	glColor3ub(119, 80, 35);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(0.0f, 4.0f, 0.0f);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(0.0f, 4.0f, 4.0f);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.4f);
	glEnd();
	//Plane Wings 机翼下
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glBindTexture(GL_TEXTURE_2D, plane_wing2);
	glBegin(GL_QUADS);
	glColor3ub(119, 80, 35);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(0.0f, -4.0f, 0.0f);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(0.0f, -4.0f, 4.0f);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.4f);
	glEnd();

}


//|____________________________________________________________________
//|
//| Function: DrawPropeller
//|
//! \param width       [in] Width  of the propeller.
//! \param length      [in] Length of the propeller.
//! \return None.
//!
//! Draws a propeller.
//|____________________________________________________________________

//组件
void DrawPropeller(const float width, const float length)  //左右边方螺旋桨
{
	float w = width / 2;
	float l = length / 2;

	/*螺旋桨部分
	可以旋转
	配合按键：
	r,Rotates right propeller
	y,Rotates left propeller
	t,Rotates top propeller
	*/

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	// Propeller is blue
	glColor3f(-0.2f, 1.0f, 1.0f);
	glVertex3f(0.0f, 0.25f, -2.0f);
	glVertex3f(0.5f, 0.25f, -2.0f);
	glVertex3f(0.0f, 0.25f, 0.0f);
	glEnd();

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	// Propeller is blue
	glColor3f(-0.2f, 1.0f, 1.0f);
	glVertex3f(0.0f, 0.25f, 2.0f);
	glVertex3f(-0.5f, 0.25f, 2.0f);
	glVertex3f(0.0f, 0.25f, 0.0f);
	glEnd();

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	// Propeller is white
	glColor3f(1.0f, 1.0f, 1.0f);
	glVertex3f(2.0f, 0.25f, 0.0f);
	glVertex3f(2.0f, 0.25f, 0.5f);
	glVertex3f(0.0f, 0.25f, 0.0f);
	glEnd();

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	// Propeller is white
	glColor3f(1.0f, 1.0f, 1.0f);
	glVertex3f(-2.0f, 0.25f, 0.0f);
	glVertex3f(-2.0f, 0.25f, -0.5f);
	glVertex3f(0.0f, 0.25f, 0.0f);
	glEnd();
}

void DrawPropeller2(const float width, const float length)  //前方螺旋桨 Front Propeller
{
	float w = width / 2;
	float l = length / 2;

	/*螺旋桨部分
	可以旋转
	配合按键：
	r,Rotates right propeller
	y,Rotates left propeller
	t,Rotates top propeller
	*/

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	// Propeller is blue
	glColor3f(-0.2f, 1.0f, 1.0f);
	glVertex3f(0.0f, 3.0f, 13.5f);
	glVertex3f(1.0f, 3.0f, 13.5f);
	glVertex3f(0.0f, 0.0f, 13.5f);
	glEnd();

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	// Propeller is blue
	glColor3f(-0.2f, 1.0f, 1.0f);
	glVertex3f(-1.0f, -3.0f, 13.5f);
	glVertex3f(0.0f, -3.0f, 13.5f);
	glVertex3f(0.0f, 0.0f, 13.5f);
	glEnd();

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	// Propeller is white
	glColor3f(0.8f, 1.0f, 1.0f);
	glVertex3f(3.0f, 0.0f, 13.5f);
	glVertex3f(3.0f, -1.0f, 13.5f);
	glVertex3f(0.0f, 0.0f, 13.5f);
	glEnd();

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	// Propeller is white
	glColor3f(0.8f, 1.0f, 1.0f);
	glVertex3f(-3.0f, 0.0f, 13.5f);
	glVertex3f(-3.0f, 1.0f, 13.5f);
	glVertex3f(0.0f, 0.0f, 13.5f);
	glEnd();
}

/*draw circular cone
N---> Circular cone smoothness N = integer
H---> Hight
R---> Radius
PI----> π 
底面积(Lateral area) =π*r^2=π*1=π
侧面积(Bottom area) =π*r*l=π*1*2=2π
*/ 

void DrawSubPropeller(const float width, const float length)  //SubPropeller
{
	//绘制圆锥
	int N = 66; // 决定圆锥的平滑程度，
	float R = 1.0f; // 半径
	float H = 1.0f; // 高
	float PI = 3.141593f;
	glBegin(GL_TRIANGLE_FAN);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKGRAY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARKGREEN_COL);
	glVertex3f(10.0f, H, 11.0f);
	for (int i = 0; i < N; i++)
	{
		glVertex3f(10.0f + R * cos(i * 2 * PI / N), 3.0f, 11.0f + R * sin(i * 2 * PI / N));
		glVertex3f(10.0f + R * cos((i + 1) * 2 * PI / N), 3.0f, 11.0f + R * sin(i * 2 * PI / N));
	}
	glEnd();

	glBegin(GL_TRIANGLE_FAN);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, GREEN_COL);
	glVertex3f(10.0f, 4.0f, 11.0f);
	for (int i = 0; i < N; i++)
	{
		glVertex3f(10.0f + R * cos(i * 2 * PI / N), 3.0f, 11.0f + R * sin(i * 2 * PI / N));
		glVertex3f(10.0f + R * cos((i + 1) * 2 * PI / N), 3.0f, 11.0f + R * sin(i * 2 * PI / N));
	}
	glEnd();
}

void DrawSubPropeller2(const float width, const float length)  //SubPropeller2
{
	//绘制圆锥2 
	int N = 36; // 决定圆锥的平滑程度，
	float R = 1.0f; // 半径
	float H = 1.0f; // 高
	float PI = 3.141593f;
	glBegin(GL_TRIANGLE_FAN);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, WHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BLUE_COL);
	glVertex3f(20.0f, H, 10.0f);
	for (int i = 0; i < N; i++)
	{
		glVertex3f(20.0f + R * cos(i * 2 * PI / N), 5.0f, 10.0f + R * sin(i * 2 * PI / N));
		glVertex3f(20.0f + R * cos((i + 1) * 2 * PI / N), 5.0f, 10.0f + R * sin(i * 2 * PI / N));
	}
	glEnd();

	glBegin(GL_TRIANGLE_FAN);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, RED_COL);
	glVertex3f(20.0f, 5.0f, 10.0f);
	for (int i = 0; i < N; i++)
	{
		glVertex3f(20.0f + R * cos(i * 2 * PI / N), 5.0f, 10.0f + R * sin(i * 2 * PI / N));
		glVertex3f(20.0f + R * cos((i + 1) * 2 * PI / N), 5.0f, 10.0f + R * sin(i * 2 * PI / N));
	}
	glEnd();
}

/*
// ///----------------------------- unused part------------------------///
//	//// ------------------------五角形----------------------------////
//                             Pentagram
// 设五角星的五个顶点分布位置关系如下：
//        A
// E ++++++++++++ B
//      ++++++
//    D        C
// 首先，根据余弦定理列方程，计算五角星的中心到顶点的距离a
// （假设五角星对应正五边形的边长为.0）
// a = 1 / (2-2*cos(72*Pi/180));
// 然后，根据正弦和余弦的定义，计算B的x坐标bx和y坐标by，以及C的y坐标
// （假设五角星的中心在坐标原点）
// bx = a * cos(18 * Pi/180);
// by = a * sin(18 * Pi/180);
// cy = -a * cos(18 * Pi/180);
// 五个点的坐标就可以通过以上四个量和一些常数简单的表示出来
	const GLfloat Pi = 3.1415926536f;
	GLfloat a = 1 / (2 - 2 * cos(72 * Pi / 180));
	GLfloat bx = a * cos(18 * Pi / 180);
	GLfloat by = a * sin(18 * Pi / 180);
	GLfloat cy = -a * cos(18 * Pi / 180);
	GLfloat
		PointA[2] = { 0, a },
		PointB[2] = { bx, by},
		PointC[2] = { 0.5, cy},
		PointD[2] = { -0.5, cy},
		PointE[2] = { -bx, by};
	glClear(GL_COLOR_BUFFER_BIT);
	// 按照A->C->E->B->D->A的顺序，可以一笔将五角星画出
	glBegin(GL_LINE_LOOP);
	glVertex2fv(PointA);
	glVertex2fv(PointC);
	glVertex2fv(PointE);
	glVertex2fv(PointB);
	glVertex2fv(PointD);
	glEnd();
	glFlush();
	*/

//卫星
void drawSatelliteLeftWing(const float width, const float height, const float offset)
{
	//Enable the texture of satellite wing
	glEnable(GL_TEXTURE_2D);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE_COL);

	glBindTexture(GL_TEXTURE_2D, satellite_wing);
	glBegin(GL_QUADS);

	glNormal3f(0, 1, 0);
	glTexCoord2f(1, 1);
	glVertex3f(1.5f, 0, height + offset);
	glTexCoord2f(0, 1);
	glVertex3f(width, 0, height + offset);
	glTexCoord2f(0, 0);
	glVertex3f(width, 0, offset);
	glTexCoord2f(1, 0);
	glVertex3f(1.5f, 0, offset);

	glEnd();
	glDisable(GL_TEXTURE_2D);
}
void drawSatelliteRightWing(const float width, const float height, const float offset)
{
	glEnable(GL_TEXTURE_2D);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE_COL);
	glBindTexture(GL_TEXTURE_2D, satellite_wing);
	glBegin(GL_QUADS);

	glNormal3f(0, 1, 0);
	glTexCoord2f(1, 1);
	glVertex3f(-1.5f, 0, offset);
	glTexCoord2f(0, 1);
	glVertex3f(-width, 0, offset);
	glTexCoord2f(0, 0);
	glVertex3f(-width, 0, height + offset);
	glTexCoord2f(1, 0);
	glVertex3f(-1.5f, 0, height + offset);

	glEnd();
	glDisable(GL_TEXTURE_2D);
}
void drawSatellite() {
	//卫星身体部分
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, PASTELDARKER_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, PASTEL_COL);
	drawCylinder(1.5f, 10.0f);
	drawSatelliteLeftWing(12.0f, 5.0f, 2.5f);
	drawSatelliteRightWing(12.0f, 5.0f, 2.5f);
	DrawCoordinateFrame(2);
	glPopMatrix();

	//Disable Two-Side Lighting
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
}

//相机
void drawCamera(const float radius, const float height) {
	glDisable(GL_LIGHTING);
	glColor3ub(237, 176, 102);
	glutSolidCone(radius, height, 10, 10);
	glEnable(GL_LIGHTING);
}
//光
void drawLight(const float radius) {
	glDisable(GL_LIGHTING);
	glColor3f(1.0f, 1.0f, 1.0f);
	glutSolidSphere(radius, 20, 20);
	glEnable(GL_LIGHTING);
}

//|____________________________________________________________________
//|
//| Function: DrawSkybox
//|
//! \param s      [in] Skybox size.
//! \return None.
//!
//! Draws a skybox.
//|____________________________________________________________________

// 天空盒子
void DrawSkybox(const float s)
{
	float s2 = s / 2;

	// Turn on texture mapping and disable lighting
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);

	// Back wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBACK]);  // Specify which texture will be used
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(-s2, -s2, -s2);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(s2, s2, -s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(-s2, s2, -s2);
	glEnd();

	// Left wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYLEFT]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-s2, -s2, -s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-s2, s2, -s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(-s2, s2, s2);
	glEnd();

	// Bottom wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBOTTOM]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-s2, -s2, -s2);
	glEnd();

	// Top Wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYTOP]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-s2, s2, s2);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(s2, s2, s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, s2, -s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-s2, s2, -s2);
	glEnd();

	// Front Wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYFRONT]);  // Specify which texture will be used
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, s2, s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-s2, s2, s2);
	glEnd();

	// Right Wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYRIGHT]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, s2, -s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(s2, s2, s2);
	glEnd();

	// Turn off texture mapping and enable lighting
	glEnable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
}

//|____________________________________________________________________
//|
//| Function: SetLight
//|
//! \param pos          [in] Light position.
//! \param is_ambient   [in] Is ambient enabled?
//! \param is_diffuse   [in] Is diffuse enabled?
//! \param is_specular  [in] Is specular enabled?
//! \return None.
//!
//! Set light properties.
//|____________________________________________________________________

void SetLight(const gmtl::Point4f& pos, const bool is_ambient, const bool is_diffuse, const bool is_specular)
{
	glLightfv(GL_LIGHT0, GL_POSITION, pos.mData);
	if (is_ambient) {
		glLightfv(GL_LIGHT0, GL_AMBIENT, AMBIENT_LIGHT);
	}else {
		glLightfv(GL_LIGHT0, GL_AMBIENT, NO_LIGHT);
	}if (is_diffuse) {
		glLightfv(GL_LIGHT0, GL_DIFFUSE, DIFFUSE_LIGHT);
	}else {
		glLightfv(GL_LIGHT0, GL_DIFFUSE, NO_LIGHT);
	}if (is_specular) {
		glLightfv(GL_LIGHT0, GL_SPECULAR, SPECULAR_LIGHT);
	}else {
		glLightfv(GL_LIGHT0, GL_SPECULAR, NO_LIGHT);
	}
}

//|____________________________________________________________________
//|
//| Function: LoadPPM
//|
//! \param fname       [in]  Name of file to load.
//! \param w           [out] Width of loaded image in pixels.
//! \param h           [out] Height of loaded image in pixels.
//! \param data        [in/out] Image data address (input or output depending on mallocflag).
//! \param mallocflag  [in] 1 if memory not pre-allocated, 0 if data already points to allocated memory that can hold the image.
//! \return None.
//!
//! A very minimal Portable Pixelformat image file loader. Note that if new memory is allocated, free() should be used
//! to deallocate when it is no longer needed.
//|____________________________________________________________________

void LoadPPM(const char* fname, unsigned int* w, unsigned int* h, unsigned char** data, const int mallocflag)
{
	FILE* fp;
	char P, num;
	int max;
	char s[1000];

	if (!(fp = fopen(fname, "rb")))
	{
		perror("cannot open image file\n");
		exit(0);
	}

	fscanf(fp, "%c%c\n", &P, &num);
	if ((P != 'P') || (num != '6'))
	{
		perror("unknown file format for image\n");
		exit(0);
	}

	do
	{
		fgets(s, 999, fp);
	} while (s[0] == '#');

	sscanf(s, "%d%d", w, h);
	fgets(s, 999, fp);
	sscanf(s, "%d", &max);

	printf("%s -> w: %d h: %d\n", fname, *w, *h);

	if (mallocflag)
		if (!(*data = (unsigned char*)malloc(*w * *h * 3)))
		{
			perror("cannot allocate memory for image data\n");
			exit(0);
		}

	fread(*data, 3, *w * *h, fp);

	fclose(fp);
}

//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char** argv)
{
	InitTransforms();

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);     // Uses GLUT_DOUBLE to enable double buffering
	glutInitWindowSize(w_width, w_height);

	glutCreateWindow("Plane Episode 3");

	glutDisplayFunc(DisplayFunc);
	glutKeyboardFunc(KeyboardFunc);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MotionFunc);
	glutReshapeFunc(ReshapeFunc);

	InitGL();

	glutMainLoop();

	return 0;
}