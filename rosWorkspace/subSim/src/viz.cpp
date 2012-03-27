//
// This code was created by Jeff Molofee '99 (ported to Linux/GLUT by 
// Richard Campbell '00)
//
// If you've found this code useful, please let me know.
//
// Visit Jeff Molofee at www.demonews.com/hosted/nehe 
// (email Richard Campbell at ulmont@bellsouth.net)
//
#include <GL/glut.h>    // Header File For The GLUT Library 
#include <GL/gl.h>	// Header File For The OpenGL32 Library
#include <GL/glu.h>	// Header File For The GLu32 Library
#include <unistd.h>     // Header file for sleeping.
#include <stdio.h>      // Header file for standard file i/o.
#include <stdlib.h>     // Header file for malloc/free.
#include <math.h>       // Header file for trigonometric functions.
#include "objLoader.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"

/* ascii codes for various special keys */
#define ESCAPE 27
#define PAGE_UP 73
#define PAGE_DOWN 81
#define UP_ARROW 72
#define DOWN_ARROW 80
#define LEFT_ARROW 75
#define RIGHT_ARROW 77

/* The number of our GLUT window */
int window; 

int gWidth, gHeight;
GLuint loop;             // general loop variable
GLuint texture[3];       // storage for 3 textures;

int light = 0;           // lighting on/off
int blend = 0;        // blending on/off
int fog = 0;

int oldx;
int oldy;

float Orange[] = {0.8, .45, .2};
float seaGreen[] = {.1, .5, .3};
float fogColor[] = {.01, .12, .10, 1};

//Models 
int pool;
int path;
int gate;
int obsCourse;

//ros subscribers
ros::Subscriber mPosX;
ros::Subscriber mPosY;
ros::Subscriber mPosZ;
ros::Subscriber mRotYaw;

char keyDown[256];

float xpos, ypos, zpos;
float rot, vrot;

GLfloat camx = 0, camy = 0, camz = 0; // camera location.
GLfloat therotate;

GLfloat z=0.0f;                       // depth into the screen.

GLfloat LightAmbient[]  = {0.5f, 0.5f, 0.5f, 1.0f}; 
GLfloat LightDiffuse[]  = {1.0f, 1.0f, 1.0f, 1.0f}; 
GLfloat LightSpecular[]  = {0.0f, 0.0f, 0.0f, 0.2f}; 
GLfloat LightPosition[] = {0.4f, 0.4f, 1.0f, 0.0f};
GLfloat AltPosition[] = {0.0f, 0.0f, -1.0f, 0.0f};

GLuint filter = 0;       // texture filtering method to use (nearest, linear, linear + mipmaps)

typedef struct {         // vertex coordinates - 3d and texture
	GLfloat x, y, z;     // 3d coords.
	GLfloat u, v;        // texture coords.
} VERTEX;

typedef struct {         // triangle
	VERTEX vertex[3];    // 3 vertices array
} TRIANGLE;

typedef struct {         // sector of a 3d environment
	int numtriangles;    // number of triangles in the sector
	TRIANGLE* triangle;  // pointer to array of triangles.
} SECTOR;

SECTOR sector1;

/* Image type - contains height, width, and data */
typedef struct {
	unsigned long sizeX;
	unsigned long sizeY;
	char *data;
} Image;

void mPosXCallback(const std_msgs::Float32::ConstPtr& msg) {
	xpos = msg->data * 12;
	printf("setting xpos = %f\n", xpos);
	glutPostRedisplay();
}

void mPosYCallback(const std_msgs::Float32::ConstPtr& msg) {
	ypos = msg->data * 12;
	printf("setting ypos = %f\n", ypos);
	glutPostRedisplay();
}

void mPosZCallback(const std_msgs::Float32::ConstPtr& msg) {
	zpos = msg->data * 12;
	printf("setting zpos = %f\n", zpos);
	glutPostRedisplay();
}

void mRotYawCallback(const std_msgs::Float32::ConstPtr& msg) {
	rot = msg->data;
	printf("setting yaw = %f\n", rot);
	glutPostRedisplay();
}

int makePath() {
	int num = glGenLists(1);
	glNewList(num, GL_COMPILE);
	glPushMatrix();
	glColor3fv(Orange);
	glTranslatef(0,0,-174);
	glScalef(48, 6, 1);
	glutSolidCube(1);
	glPopMatrix();
	glEndList();
	return num;
}

int makeObsCourse() {
	int num = glGenLists(1);
	glNewList(num, GL_COMPILE);
	GLUquadric* quad = gluNewQuadric();

	glColor3f(0,.7,0);
	glPushMatrix();
	glTranslatef(36, 0, 0);
	gluCylinder(quad, 1, 1, 24, 20, 2);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-36, 0, 0);
	gluCylinder(quad, 1, 1, 24, 20, 2);
	glPopMatrix();

	glPushMatrix();
	glRotatef(90, 0, 1, 0);
	glTranslatef(24, 0, -36);
	gluCylinder(quad, 1, 1, 72, 20, 2);
	glPopMatrix();
	glEndList();
	return num;
}

int makeGate() {
	int num = glGenLists(1);
	glNewList(num, GL_COMPILE);
	GLUquadric* quad = gluNewQuadric();

	glColor3fv(Orange);
	glPushMatrix();
	glTranslatef(60, 0, 0);
	gluCylinder(quad, 2, 2, 72, 20, 2);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-60, 0, 0);
	gluCylinder(quad, 2, 2, 72, 20, 2);
	glPopMatrix();

	glColor3f(.9, .9, .9);
	glPushMatrix();
	glRotatef(90, 0, 1, 0);
	glTranslatef(-72, 0, -60);
	gluCylinder(quad, 2, 2, 120, 20, 2);
	glPopMatrix();
	glEndList();
	return num;
}

// helper for SetupWorld.  reads a file into a string until a nonblank, non-comment line
// is found ("/" at the start indicating a comment); assumes lines < 255 characters long.
void readstr(FILE *f, char *string)
{
	do {
		fgets(string, 255, f); // read the line
	} while ((string[0] == '/') || (string[0] == '\n'));
	return;
}

// loads the world from a text file.
void SetupWorld() 
{
	float x, y, z, u, v;
	int vert;
	int numtriangles;
	FILE *filein;        // file to load the world from
	char oneline[255];
	int loop;

	filein = fopen("Data/lesson10/world.txt", "rt");

	readstr(filein, oneline);
	sscanf(oneline, "NUMPOLLIES %d\n", &numtriangles);

	sector1.numtriangles = numtriangles;
	sector1.triangle = (TRIANGLE *) malloc(sizeof(TRIANGLE)*numtriangles);

	for (loop = 0; loop < numtriangles; loop++) {
		for (vert = 0; vert < 3; vert++) {
			readstr(filein,oneline);
			sscanf(oneline, "%f %f %f %f %f", &x, &y, &z, &u, &v);
			sector1.triangle[loop].vertex[vert].x = x;
			sector1.triangle[loop].vertex[vert].y = y;
			sector1.triangle[loop].vertex[vert].z = z;
			sector1.triangle[loop].vertex[vert].u = u;
			sector1.triangle[loop].vertex[vert].v = v;
		}
	}

	fclose(filein);
	return;
}

/*
 * getint and getshort are help functions to load the bitmap byte by byte on 
 * SPARC platform (actually, just makes the thing work on platforms of either
 * endianness, not just Intel's little endian)
 */
static unsigned int getint(FILE* fp)
{
	int c, c1, c2, c3;

	// get 4 bytes
	c = getc(fp);  
	c1 = getc(fp);  
	c2 = getc(fp);  
	c3 = getc(fp);

	return ((unsigned int) c) +   
		(((unsigned int) c1) << 8) + 
		(((unsigned int) c2) << 16) +
		(((unsigned int) c3) << 24);
}

static unsigned int getshort(FILE* fp)
{
	int c, c1;

	//get 2 bytes
	c = getc(fp);  
	c1 = getc(fp);

	return ((unsigned int) c) + (((unsigned int) c1) << 8);
}

// quick and dirty bitmap loader...for 24 bit bitmaps with 1 plane only.  
// See http://www.dcs.ed.ac.uk/~mxr/gfx/2d/BMP.txt for more info.
int ImageLoad(const char *filename, Image *image) 
{
	FILE *file;
	unsigned long size;                 // size of the image in bytes.
	unsigned long i;                    // standard counter.
	unsigned short int planes;          // number of planes in image (must be 1) 
	unsigned short int bpp;             // number of bits per pixel (must be 24)
	char temp;                          // used to convert bgr to rgb color.

	// make sure the file is there.
	if ((file = fopen(filename, "rb"))==NULL) {
		printf("File Not Found : %s\n",filename);
		return 0;
	}

	// seek through the bmp header, up to the width/height:
	fseek(file, 18, SEEK_CUR);

	// No 100% errorchecking anymore!!!

	// read the width
	image->sizeX = getint (file);
	printf("Width of %s: %lu\n", filename, image->sizeX);

	// read the height 
	image->sizeY = getint (file);
	printf("Height of %s: %lu\n", filename, image->sizeY);

	// calculate the size (assuming 24 bits or 3 bytes per pixel).
	size = image->sizeX * image->sizeY * 3;

	// read the planes
	planes = getshort(file);
	if (planes != 1) {
		printf("Planes from %s is not 1: %u\n", filename, planes);
		return 0;
	}

	// read the bpp
	bpp = getshort(file);
	if (bpp != 24) {
		printf("Bpp from %s is not 24: %u\n", filename, bpp);
		return 0;
	}

	// seek past the rest of the bitmap header.
	fseek(file, 24, SEEK_CUR);

	// read the data. 
	image->data = (char *) malloc(size);
	if (image->data == NULL) {
		printf("Error allocating memory for color-corrected image data");
		return 0;	
	}

	if ((i = fread(image->data, size, 1, file)) != 1) {
		printf("Error reading image data from %s.\n", filename);
		return 0;
	}

	for (i=0;i<size;i+=3) { // reverse all of the colors. (bgr -> rgb)
		temp = image->data[i];
		image->data[i] = image->data[i+2];
		image->data[i+2] = temp;
	}

	// we're done.
	return 1;
}

// Load Bitmaps And Convert To Textures
void LoadGLTextures(void) 
{	
	// Load Texture
	Image *image1;

	// allocate space for texture
	image1 = (Image *) malloc(sizeof(Image));
	if (image1 == NULL) {
		printf("Error allocating space for image");
		exit(0);
	}

	if (!ImageLoad("Data/lesson10/mud.bmp", image1)) {
		exit(1);
	}        

	// Create Textures	
	glGenTextures(3, &texture[0]);

	// nearest filtered texture
	glBindTexture(GL_TEXTURE_2D, texture[0]);   // 2d texture (x and y size)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST); // scale cheaply when image bigger than texture
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST); // scale cheaply when image smalled than texture
	glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);

	// linear filtered texture
	glBindTexture(GL_TEXTURE_2D, texture[1]);   // 2d texture (x and y size)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture
	glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);

	// mipmapped texture
	glBindTexture(GL_TEXTURE_2D, texture[2]);   // 2d texture (x and y size)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST); // scale mipmap when image smalled than texture
	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, image1->sizeX, image1->sizeY, GL_RGB, GL_UNSIGNED_BYTE, image1->data);
};

/* A general OpenGL initialization function.  Sets all of the initial parameters. */
void InitGL(GLsizei Width, GLsizei Height)	// We call this right after our OpenGL window is created.
{
	pool = loadObject("pool.obj");
	gate = makeGate();
	obsCourse = makeObsCourse();
	path = makePath();
	xpos = -18.3;
	ypos =  18.5;
	zpos = - 0.6;
	vrot = 0;
	LoadGLTextures();                           // load the textures.
	glEnable(GL_TEXTURE_2D);                    // Enable texture mapping.

	glEnable( GL_NORMALIZE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);          // Set the blending function for translucency (note off at init time)
	glClearColor(0.01f, 0.12f, 0.1f, 1.0f);	// This Will Clear The Background Color To Black
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogfv(GL_FOG_COLOR, fogColor);
	glFogf(GL_FOG_DENSITY, 0.45f);
	glFogf(GL_FOG_START, 0.2);
	glFogf(GL_FOG_END, 10.0);
	//glEnable(GL_FOG);
	glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LESS);                       // type of depth test to do.
	glEnable(GL_DEPTH_TEST);                    // enables depth testing.
	glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading
	glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();				// Reset The Projection Matrix

	gluPerspective(30.0f,(GLfloat)Width/(GLfloat)Height,1.0f,1000.0f);	// Calculate The Aspect Ratio Of The Window

	glMatrixMode(GL_MODELVIEW);

	glutSetCursor(GLUT_CURSOR_NONE);
	glutIgnoreKeyRepeat(1);
}

void InitROS(int argc, char** argv) {
	ros::init(argc, argv, "subViz");

	ros::NodeHandle n;

	mPosX = n.subscribe("/sim/pos/x", 100, mPosXCallback);
	mPosY = n.subscribe("/sim/pos/y", 100, mPosYCallback);
	mPosZ = n.subscribe("/sim/pos/z", 100, mPosZCallback);
	mRotYaw = n.subscribe("/sim/rotation/yaw", 100, mRotYawCallback);
}
/* The function called when our window is resized (which shouldn't happen, because we're fullscreen) */
void ReSizeGLScene(GLsizei Width, GLsizei Height)
{
	if (Height==0)				// Prevent A Divide By Zero If The Window Is Too Small
		Height=1;

	glViewport(0, 0, Width, Height);		// Reset The Current Viewport And Perspective Transformation

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(30.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);
	glMatrixMode(GL_MODELVIEW);
	gWidth = Width;
	gHeight = Height;
}

void MouseClick(int button, int state, int x, int y) {
	oldx = x;
	oldy = y;
}

/* The idle function for the program */
void Idle(void) {
//	if(keyDown['s']) {
//		xpos -= (float)sin(rot) * 0.008f;
//		ypos -= (float)cos(rot) * 0.008f;	
//	}
//	if(keyDown['d']) {
//		xpos += (float)sin(90+rot) * 0.008f;
//		ypos += (float)cos(90+rot) * 0.008f;	
//	}
//	if(keyDown['a']) {
//		xpos += (float)sin(-90+rot) * 0.008f;
//		ypos += (float)cos(-90+rot) * 0.008f;	
//	}
//	if(keyDown['w']) {
//		xpos += (float)sin(rot) * 0.008f;
//		ypos += (float)cos(rot) * 0.008f;	
//	}

	ros::spinOnce();
	glutPostRedisplay();
}

/* The main drawing function. */
void DrawGLScene(void)
{
	float xat = xpos + sin(rot);
	float yat = ypos + cos(rot);
	float zat = zpos + tan(vrot);
	// calculate translations and rotations.

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// Clear The Screen And The Depth Buffer

	// set up lights.
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT1);

	glLoadIdentity();

	gluLookAt(xpos/100, ypos/100, zpos/100,
			  //0.0, 0.0, 0.0,
			  xat/100 , yat/100 , zat/100,
			  0.0, 0.0, 1.0);
	glScalef(.01,.01,.01);

	glBindTexture(GL_TEXTURE_2D, texture[filter]);    // pick the texture.

	glColor3fv(seaGreen);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, seaGreen);
	glPushMatrix();
	glRotatef(-90, 1,0,0);
	glCallList(pool);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-1700.0, 1500.0, -70.0);
	glRotatef(30.0, 0.0, 0.0, 1.0);
	glCallList(gate);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-1700, 1500, 0);
	glRotatef(110, 0, 0, 1);
	glCallList(path);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-1600.0, 1250.0, -75.0);
	glColor3f(.7,.15,.06);
	glutSolidSphere(4.5, 20, 20);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-1550.0, 1250.0, -100.0);
	glColor3f(0,.7,0);
	glutSolidSphere(4.5, 20, 20);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-1650.0, 1250.0, -88.0);
	glColor3f(.6,.7,0);
	glutSolidSphere(4.5, 20, 20);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-1600, 1250, 0);
	glRotatef(160, 0, 0, 1);
	glCallList(path);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-1400.0, 1230.0, -84.0);
	glRotatef(75.0, 0.0, 0.0, 1.0);
	glCallList(obsCourse);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-1100.0, 1130.0, -84.0);
	glRotatef(75.0, 0.0, 0.0, 1.0);
	glCallList(obsCourse);
	glPopMatrix();

	// since this is double buffered, swap the buffers to display what just got drawn.
	glutSwapBuffers();
}


void keyUp(unsigned char key, int x, int y) {
	switch(key) {
		case 'w':
		case 's':
		case 'd':
		case 'a':
			keyDown[key]=0;
			break;
	}
}

/* The function called whenever a normal key is pressed. */
void keyPressed(unsigned char key, int x, int y) 
{
	/* avoid thrashing this procedure */
	usleep(100);

	switch (key) {    
		case ESCAPE:
			exit(1);                   	
			break; // redundant.
		case 'a':
		case 's':
		case 'w':
		case 'd':
			keyDown[key] = 1;
			break;
		case 'A':
		case 'W':
		case 'S':
		case 'D':
			keyDown[key + 'a' - 'A'] = 1;
			break;

		case 'b': 
		case 'B': // switch the blending
			printf("B/b pressed; blending is: %d\n", blend);
			blend = blend ? 0 : 1;              // switch the current value of blend, between 0 and 1.
			if (blend) {
				glEnable(GL_BLEND);
				glDisable(GL_DEPTH_TEST);
			} else {
				glDisable(GL_BLEND);
				glEnable(GL_DEPTH_TEST);
			}
			printf("Blending is now: %d\n", blend);
			break;

		case 'o':
		case 'O':
			if(fog) {
				glDisable(GL_FOG);
			} else {
				glEnable(GL_FOG);
			}
			fog = !fog;
			break;

		case 'f': 
		case 'F': // switch the filter
			printf("F/f pressed; filter is: %d\n", filter);
			filter++;                           // switch the current value of filter, between 0/1/2;
			if (filter > 2) {
				filter = 0;
			}
			printf("Filter is now: %d\n", filter);
			break;

		case 'l': 
		case 'L': // switch the lighting
			printf("L/l pressed; lighting is: %d\n", light);
			light = light ? 0 : 1;              // switch the current value of light, between 0 and 1.
			if (light) {
				glEnable(GL_LIGHTING);
				glEnable(GL_COLOR_MATERIAL);
			} else {
				glDisable(GL_LIGHTING);
				glDisable(GL_COLOR_MATERIAL);
			}
			printf("Lighting is now: %d\n", light);
			break;

		default:
			printf ("Key %d pressed. No action there yet.\n", key);
			break;
	}	
}

int main(int argc, char **argv) 
{  
	/* load our world from disk */
	SetupWorld();

	/* Initialize GLUT state - glut will take any command line arguments that pertain to it or 
	   X Windows - look at its documentation at http://reality.sgi.com/mjk/spec3/spec3.html */  
	glutInit(&argc, argv);  

	/* Select type of Display mode:   
	   Double buffer 
	   RGBA color
	   Depth buffer 
	   Alpha blending */  
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_ALPHA);  

	/* get a 640 x 480 window */
	glutInitWindowSize(640, 480);  

	/* the window starts at the upper left corner of the screen */
	glutInitWindowPosition(0, 0);  

	/* Open a window */  
	window = glutCreateWindow("Robosub 2012 Simulation");

	/* Register the function to do all our OpenGL drawing. */
	glutDisplayFunc(&DrawGLScene);  

	/* Go fullscreen.  This is as soon as possible. */
//	glutFullScreen();

	/* Even if there are no events, redraw our gl scene. */
	glutIdleFunc(&Idle); 

	/* Register the function called when our window is resized. */
	glutReshapeFunc(&ReSizeGLScene);

	/* Register the function called when the keyboard is pressed. */
	glutKeyboardFunc(&keyPressed);
	glutKeyboardUpFunc(&keyUp);

	/* Mouse Motion function */
//	glutMotionFunc(Mouse);
//	glutPassiveMotionFunc(Mouse);

	glutMouseFunc(MouseClick);

	/* Initialize our window. */
	InitGL(640, 480);

	InitROS(argc, argv);
	/* Start Event Processing Engine */  
	glutMainLoop();  

	return 1;
}
