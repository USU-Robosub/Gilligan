#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdlib.h>
#include <math.h>
#include <list>
#include <time.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#define FEET *12

struct Vector {
	double x;
	double y;
	double z;
};

struct Object {
	Vector Pos;
	Vector Rot;
};

GLfloat fogDenisty = .3;
GLfloat fogColor[] = {.1,.65,.75,0.0};

const int reduction = 1;
const int screenX = 640/reduction;
const int screenY = 480/reduction;

Object Sub;


struct Cylinder {
	Cylinder() {}
	Cylinder(double x1, double y1, double z1,
			 double x2, double y2, double z2,
			 double red,  double green,  double blue,
			 double Radius) {
		p1.x = x1; p1.y = y1; p1.z = z1;
		p2.x = x2; p2.y = y2; p2.z = z2;
		r = red; b = blue; g = green;
		radius = Radius;
	}
	Vector p1;
	Vector p2;
	double radius;
	float r,g,b;
};

void mPosXCallback(const std_msgs::Float32::ConstPtr& msg) {
	printf("x %f -> %f\n", Sub.Pos.x, msg->data FEET);
	Sub.Pos.x = msg->data FEET;
	glutPostRedisplay();
}
void mPosYCallback(const std_msgs::Float32::ConstPtr& msg) {
	Sub.Pos.y = msg->data FEET;
	glutPostRedisplay();
}
void mPosZCallback(const std_msgs::Float32::ConstPtr& msg) {
	Sub.Pos.z = msg->data FEET;
	glutPostRedisplay();
}

std::list<Cylinder> rC;

void AddLLane(Vector p) {
	rC.push_back(Cylinder(p.x-20,p.y,p.z, p.x-20,p.y,p.z+48, 0,0,0, 1));
	rC.push_back(Cylinder(p.x+20,p.y,p.z, p.x+20,p.y,p.z+48, 0,0,0, 1));
	rC.push_back(Cylinder(p.x-36,p.y,p.z+48, p.x+36,p.y,p.z+48, 0,.6,.1, 1));
	rC.push_back(Cylinder(p.x-36,p.y,p.z+48, p.x-36,p.y,p.z+96, 0,.6,.1, 1));
}

void AddBouy(Vector p, float r, float g, float b) {
	rC.push_back(Cylinder(p.x, p.y, p.z+1,  p.x, p.y, p.z-1, r,g,b, 4.5));
}

ros::Subscriber mPosX;
ros::Subscriber mPosY;
ros::Subscriber mPosZ;


void init(void) 
{
	srand(time(0));
	glClearColor (0.1, 0.65, 0.75, 0.1);
	glShadeModel (GL_FLAT);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);

	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogfv(GL_FOG_COLOR, fogColor);
	glFogf(GL_FOG_DENSITY, fogDenisty);
	glFogf(GL_FOG_START, 120.0);
	glFogf(GL_FOG_END, 360.0);

	GLfloat LightPos[] = {0.0, 0.0, 2000.0, 0.0};
	GLfloat Ambient[] = {.5,.5,.5,1.0}; 

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, Ambient);

	Vector p; 
	p.x = -50 FEET;
	p.y = 100 FEET;
	p.z = 0;
	AddLLane(p);
	p.x = 25 FEET;
	p.y = 85 FEET;
	p.z = 0;
	AddLLane(p);
	p.x = 45 FEET;
	p.y = 65 FEET;
	p.z = 0;
	AddLLane(p);
	p.x = -4 FEET;
	p.y = 0;
	p.z = 7 FEET;
	AddBouy(p, .2, .81, .5); //Green
	p.x = 0;
	p.y = 2 FEET;
	p.z = 9 FEET;
	AddBouy(p, .86, .39, .29); //red
	p.x = 4 FEET;
	p.y = 1 FEET;
	p.z = 8 FEET;
	AddBouy(p, .63, .95, .30); //yellow
	Sub.Pos.x = 0 FEET;
	Sub.Pos.y = -20 FEET;
	Sub.Pos.z = 4 FEET;

	ros::NodeHandle n;

	mPosX = n.subscribe("/sim/pos/x", 100, mPosXCallback);
	mPosY = n.subscribe("/sim/pos/y", 100, mPosYCallback);
	mPosZ = n.subscribe("/sim/pos/z", 100, mPosZCallback);
}

void renderCylinder(float x1, float y1, float z1, 
		            float x2, float y2, float z2, 
					float radius, int subdivisions,
					GLUquadricObj *quadric) {
	float vx = x2-x1;
	float vy = y2-y1;
	float vz = z2-z1;

	//handle the degenerate case of z1 == z2 with an approximation
	if(vz == 0)
		vz = .0001;

	float v = sqrt( vx*vx + vy*vy + vz*vz );
	float ax = 57.2957795*acos( vz/v );
	if ( vz < 0.0 )
		ax = -ax;
	float rx = -vy*vz;
	float ry = vx*vz;
	glPushMatrix();

	//draw the cylinder body
	glTranslatef( x1,y1,z1 );
	glRotatef(ax, rx, ry, 0.0);
	gluQuadricOrientation(quadric,GLU_OUTSIDE);
	gluCylinder(quadric, radius, radius, v, subdivisions, 1);

	//draw the first cap
//	gluQuadricOrientation(quadric,GLU_INSIDE);
//	gluDisk( quadric, 0.0, radius, subdivisions, 1);
	glutSolidSphere(radius, subdivisions, subdivisions/2);
	glTranslatef( 0,0,v );

	//draw the second cap
//	gluQuadricOrientation(quadric,GLU_OUTSIDE);
//	gluDisk( quadric, 0.0, radius, subdivisions, 1);
	glutSolidSphere(radius, subdivisions, subdivisions/2);
	glPopMatrix();
}

void renderCylinder_convenient(float x1, float y1, float z1, 
		                       float x2, float y2, float z2, 
							   float radius,int subdivisions) {
	//the same quadric can be re-used for drawing many cylinders
	GLUquadricObj *quadric=gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);
	renderCylinder(x1,y1,z1,x2,y2,z2,radius,subdivisions,quadric);
	gluDeleteQuadric(quadric);
}

void DrawCylinder(Cylinder c) {
	float colorv[4] = {c.r, c.g, c.b, 0};
	glMaterialfv(GL_FRONT, GL_DIFFUSE, colorv);
	renderCylinder_convenient(c.p1.x, c.p1.y, c.p1.z,
			                  c.p2.x, c.p2.y, c.p2.z,
							  c.radius, c.radius*10+4);
}

void DrawSub(Object o) {
	const int length = 30;
	const int radius = 4.5;
	float colorv[4] = {0, 0, .75, 0};
	glMaterialfv(GL_FRONT, GL_DIFFUSE, colorv);
	renderCylinder_convenient(o.Pos.x+sin(o.Rot.z)*length*.5, o.Pos.y+cos(o.Rot.z)*length*.5, o.Pos.z,
	                          o.Pos.x-sin(o.Rot.z)*length*.5, o.Pos.y-cos(o.Rot.z)*length*.5, o.Pos.z,
							  radius, 20);
}

void DrawCylinderChain(std::list<Cylinder> cylinders) {
	for(std::list<Cylinder>::iterator it = cylinders.begin();
			it != cylinders.end(); it++) {
		DrawCylinder(*it);
	}
}

bool inline inRange(int val, int low, int high) {
	bool ret = (val < high && val > low);
	return ret;
}

void idle(void) {
	ros::Rate fps(30);
	ros::spinOnce();
//	fps.sleep();
}

void display(void) {
	
	glLoadIdentity();
	gluLookAt( Sub.Pos.x + 24*sin(Sub.Rot.z), Sub.Pos.y + 24*cos(Sub.Rot.z), Sub.Pos.z, 
			   Sub.Pos.x + 36*sin(Sub.Rot.z), Sub.Pos.y + 36*cos(Sub.Rot.z), Sub.Pos.z, 
			   0                            , 0                            , 1.0       );
//	gluLookAt(100,200,200, Sub.Pos.x, Sub.Pos.y, Sub.Pos.z, 0.0, 0, 1.0);


	glPushMatrix();
	float zerov[4] = {0,0,0,0};
	glMaterialfv(GL_FRONT, GL_SPECULAR, zerov);
	glColor3f(0.0, 0.5, 1.0);
	DrawCylinderChain(rC);
	DrawSub(Sub);
	glPopMatrix();
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport (0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
//	glOrtho(-50.0, 50.0, -50.0, 50.0, -100.0, 1000.0);

	gluPerspective(60, 1.3, 0.1, 360);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/* 
 *  Request double buffer display mode.
 *  Register mouse input callback functions
 */
int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	ros::init(argc, argv, "subViz");
	glutInitDisplayMode (GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (screenX, screenY); 
	glutInitWindowPosition (100, 100);
	glutCreateWindow (argv[0]);
	init ();
	glutIdleFunc(idle);
	glutDisplayFunc(display); 
	glutReshapeFunc(reshape); 
	glutPostRedisplay();
	glutMainLoop();
	return 0;
}
