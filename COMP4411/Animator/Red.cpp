// The sample model.  You should build a file
// very similar to this for when you make your model.
#pragma warning (disable : 4305)
#pragma warning (disable : 4244)
#pragma warning(disable : 4786)
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "modelerui.h"
#include "particleSystem.h"
#include <FL/gl.h>
#include <iostream>
#include <vector>
#include <stdlib.h>

#include "modelerglobals.h"
#include <list>
#include "CUBE_GRID.h"
#include "METABALL.h"
#define PI 3.1415926
#include "bitmap.h"
#include "mat.h"
#include "vec.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
using namespace Eigen;
using namespace std;

void _setupOpenGl();
Mat4d getModelViewMatrix();
void SpawnParticles(Mat4d cameraTransform, Vec4d velocity);
void SpawnSpringParticles(Mat4d cameraTransform, bool isStartPt = false, bool heavier = false);
ClothParticle* SpawnClothParticles(Mat4d cameraTransform, bool isStartPt = false);
HairParticle* SpawnHairParticles(Mat4d cameraTransform, bool isStartPt = false);
void SpawnFlockParticles(Mat4d cameraTransform);

// To make a Unicycle, we inherit off of ModelerView
class Unicycle : public ModelerView
{
public:
	Unicycle(int x, int y, int w, int h, char* label)
		: ModelerView(x, y, w, h, label) {
		BMPheight = 64;
		BMPwidth = 64;
		textureBMP = readBMP("./Red_Label.bmp", BMPwidth, BMPheight);
		// For IK
		IK_related[AXLE_DIR] = true;
		IK_related[CRANK_DIR] = true;
		IK_related[TUBE_DIR] = true;
		IK_related[BEND] = true;
		IK_related[SADDLE_DIR] = true;
	}

	virtual void draw();

private:
	unsigned char* textureBMP = nullptr;
	int BMPheight = 0;
	int BMPwidth = 0;
	double dDegree = 10;
	double prev_crank_dir = 0;

	bool animated = false;
	int seed = 0;

	void drawLabelTexture();

	// For IK
	bool IK_Mode_Prev = false;
	double IK_init[NUMCONTROLS];
	double IK_target[NUMCONTROLS];
	bool IK_related[NUMCONTROLS] = { false };
	void IK_Compute();
	Matrix<double, 3, 1> Diff(Matrix<double, 5, 1> input_vector_of_angles);
	Matrix<double, 3, 1> End(Matrix<double,5,1> input_vector_of_angles, Matrix<double,3,1> starting_point);
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSampleModel(int x, int y, int w, int h, char* label)
{
	return new Unicycle(x, y, w, h, label);
}

GLfloat point_cloud[130][60] = {};
GLfloat control_point[5] = {0 , 50, -20, 25, 0};

GLfloat bezier(GLfloat l)
{
	GLfloat t = (l / SADDLE_LENGTH);
	GLfloat b = 0;
	int coef[5] = { 1, 4, 6, 4, 1 };
	for (int i = 0; i < 5; i++)
	{
		b += control_point[i] * coef[i] * pow(t, i) * pow(1-t, 4-i);
	}
	return b;
}

void cal_points()
{

	// initialize the point_cloud
	for (int i = 0; i < 130; ++i)
	{
		for (int j = 0; j < 60; ++j)
		{
			point_cloud[i][j] = SADDLE_HEIGHT;
		}
	}

	for (int l = 0; l < SADDLE_LENGTH; l++)
	{
		GLfloat b = bezier(l);
		GLfloat w = (SADDLE_LENGTH - l) / SADDLE_LENGTH * SADDLE_WIDTH;
		GLfloat r = w / 2;
		GLfloat h = (sqrt(2) - 1) * r;
		for (int w_ = -w / 2; w_ < w / 2; w_++)
		{
			GLfloat h_ = sqrt(2 * r * r - w_ * w_) - r;
			GLfloat b_ = (h_ / h) * b;
			point_cloud[l][int(SADDLE_WIDTH) / 2 + w_] = b_ + SADDLE_HEIGHT;
		}
		//point_cloud[l][int(SADDLE_WIDTH / 2) + int(-w/2) - 1] = SADDLE_HEIGHT;
		//point_cloud[l][int(SADDLE_WIDTH / 2) + int(w / 2)] = SADDLE_HEIGHT;
	}
}

bool calculated_pad = 0;

void draw_pad()
{
	cal_points();
	for (int l = 0; l < SADDLE_LENGTH-1; l++)
	{
		GLfloat w = (SADDLE_LENGTH - l) / SADDLE_LENGTH * SADDLE_WIDTH;
		for (int w_ = -w / 2 - 1; w_ < w / 2; w_++)
		{
			if (l == 0 && w_ == -w / 2 - 1)	continue;
			double x_1 = SADDLE_WIDTH / 2 + w_; double y_1 = l; double z_1 = point_cloud[int(y_1)][int(x_1)];
			double x_2 = SADDLE_WIDTH / 2 + w_ + 1; double y_2 = l; double z_2 = point_cloud[int(y_2)][int(x_2)];
			double x_3 = SADDLE_WIDTH / 2 + w_; double y_3 = l + 1; double z_3 = point_cloud[int(y_3)][int(x_3)];
			double x_4 = SADDLE_WIDTH / 2 + w_ + 1; double y_4 = l + 1; double z_4 = point_cloud[int(y_4)][int(x_4)];

			drawTriangle(x_1, y_1, z_1, 
						x_2, y_2, z_2, 
						x_4, y_4, z_4 ); //draw points 1,2,4 
			drawTriangle(x_1, y_1, z_1,
						x_4, y_4, z_4,
						x_3, y_3, z_3); //draw points 1,3,4 

		}
	};
}
/*
void draw_spring()
{
	float r = 2;
	float R = 8;
	glPushMatrix();
	for (float theta = 0; theta < 360 * 5; theta++)
	{
		double rad = theta * PI / 180;
		glPushMatrix();
		glTranslated(R * cos(rad) + 20, R * sin(rad) + 40, -(1 + 2.5 * theta / 360) * r);
		drawSphere(r);
		glPopMatrix();

		glPushMatrix();		
		glTranslated(R * cos(rad) + 40, R * sin(rad) + 40, -(1 + 2.5 * theta / 360) * r);
		drawSphere(r);
		glPopMatrix();
		// drawSphere(15 * cos(rad)+15, 15*sin(rad)+15, -(1+2*theta/360)*r);
	}
	glPopMatrix();
}
*/
// By default, normal is k_hat = (0,0,1).
// centerLine is of size (lineSize+1,2). edgeLine is of size (lineSize,2).
void drawTube(double** centerLine, double** edgeLine, int lineSize, double dDegree) {
	_setupOpenGl();

	//Beware of segmentation error when using this function.
	int SPPL = (int)(360 / dDegree); //"Sample Points per Layer"

	double*** pts = new double** [lineSize];
	for (int i = 0; i < lineSize; i++) {
		pts[i] = new double* [SPPL+1]; //首尾相连
		for (int j = 0; j < SPPL+1; j++) {
			pts[i][j] = new double[3];
		}
	}

	// Construct 'pts'
	for (int i = 0; i < lineSize; i++) {
		double a[2] = { edgeLine[i][0] - centerLine[i][0],edgeLine[i][1] - centerLine[i][1] };
		double b[2] = { centerLine[i + 1][0] - centerLine[i][0],centerLine[i + 1][1] - centerLine[i][1] };
		double n[2] = { a[0] * (a[0] * b[0] + a[1] * b[1]) - b[0] * (a[0] * a[0] + a[1] * a[1]), 
						a[1] * (a[0] * b[0] + a[1] * b[1]) - b[1] * (a[0] * a[0] + a[1] * a[1]) };
		for (int k = 0; k < 3; k++) {
			if (k != 2) pts[i][0][k] = edgeLine[i][k];
			else pts[i][0][k] = 0;
			pts[i][SPPL][k] = pts[i][0][k]; //首尾相连
		}
		for (int j = 1; j < SPPL; j++) {
			double p[3] = { pts[i][j - 1][0] - centerLine[i][0],pts[i][j - 1][1] - centerLine[i][1],pts[i][j - 1][2] };
			Mat3d rot = Mat3d::createRotation(dDegree / 180 * PI, n[0], n[1]);
			double q[3] = { p[0] * rot[0] + p[1] * rot[1] + p[2] * rot[2],p[0] * rot[3] + p[1] * rot[4] + p[2] * rot[5],p[0] * rot[6] + p[1] * rot[7] + p[2] * rot[8] };
			for (int k = 0; k < 3; k++) {
				if (k != 2) pts[i][j][k] = q[k] + centerLine[i][k];
				else pts[i][j][k] = q[k];
			}
		}
	}

	// draw side
	for (int i = 0; i < lineSize-1; i++) {
		for (int j = 0; j < SPPL; j++) {
			glBegin(GL_POLYGON);
				glVertex3d(pts[i][j][0], pts[i][j][1], pts[i][j][2]);
				glVertex3d(pts[i + 1][j][0], pts[i + 1][j][1], pts[i + 1][j][2]);
				glVertex3d(pts[i + 1][j + 1][0], pts[i + 1][j + 1][1], pts[i + 1][j + 1][2]);
				glVertex3d(pts[i][j + 1][0], pts[i][j + 1][1], pts[i][j + 1][2]);
			glEnd();
		}
	}

	//draw bottom & top
	glBegin(GL_TRIANGLE_FAN);
		glVertex3d(centerLine[0][0], centerLine[0][1], centerLine[0][2]);
		for (int j = 0; j < SPPL + 1; j++) {
			glVertex3d(pts[0][j][0], pts[0][j][1], pts[0][j][2]);
		}
	glEnd();
	glBegin(GL_TRIANGLE_FAN);
	glVertex3d(centerLine[lineSize - 1][0], centerLine[lineSize - 1][1], centerLine[lineSize - 1][2]);
	for (int j = SPPL; j >= 0; j--) { // Beware of Orientation
		glVertex3d(pts[lineSize - 1][j][0], pts[lineSize - 1][j][1], pts[lineSize - 1][j][2]);
	}
	glEnd();

	for (int i = 0; i < lineSize; i++) {
		for (int j = 0; j < SPPL+1; j++) {
			delete[] pts[i][j];
		}
		delete[] pts[i];
	}
	delete[] pts;
}

void drawTorus(double R, double r, double dDegree) {
	if (R < r) exit(-1);

	int SPPL = (int)(360 / dDegree); //"Sample Points per Layer"

	double** centerLine = new double* [SPPL + 2];
	for (int j = 0; j < SPPL + 2; j++) {
		centerLine[j] = new double[2];
		centerLine[j][0] = R * cos(j * dDegree * PI / 180);
		centerLine[j][1] = R * sin(j * dDegree * PI / 180);
	}
	for (int k = 0; k < 2; k++) {
		centerLine[SPPL][k] = centerLine[0][k];
		centerLine[SPPL + 1][k] = centerLine[1][k];
	}

	double** edgeLine = new double* [SPPL + 1];
	for (int j = 0; j < SPPL + 1; j++) {
		edgeLine[j] = new double[2];
		edgeLine[j][0] = (R-r) * cos(j * dDegree * PI / 180);
		edgeLine[j][1] = (R-r) * sin(j * dDegree * PI / 180);
	}
	for (int k = 0; k < 2; k++) {
		edgeLine[SPPL][k] = edgeLine[0][k];
	}

	drawTube(centerLine, edgeLine, SPPL + 1, dDegree);

	for (int j = 0; j < SPPL + 2; j++) {
		delete[] centerLine[j];
	}
	delete[] centerLine;
	for (int j = 0; j < SPPL + 1; j++) {
		delete[] edgeLine[j];
	}
	delete[] edgeLine;
}

void drawPartialTorus(double l, double r, double t, double dDegree) { // length, radius, theta (in rad). Drawn in x-y plane
	int NoSP = floor(180*t / PI + 0.1) + 1; //"Number of Sample Points" (in edgeLine)

	double** centerLine = new double* [NoSP + 1];
	for (int j = 0; j < NoSP + 1; j++) {
		centerLine[j] = new double[2];
		double this_t = t / (NoSP-1) * j;
		centerLine[j][0] = l / t * (1 - cos(this_t));
		centerLine[j][1] = l / t * sin(this_t);
	}
	centerLine[NoSP-1][0] = l / t * (1 - cos(t));
	centerLine[NoSP-1][1] = l / t * sin(t);

	double** edgeLine = new double* [NoSP];
	for (int j = 0; j < NoSP; j++) {
		edgeLine[j] = new double[2];
		double this_t = t / (NoSP-1) * j;
		edgeLine[j][0] = r + (l / t - r) * (1 - cos(this_t));
		edgeLine[j][1] = (l / t - r) * sin(this_t);
	}
	edgeLine[NoSP-1][0] = r + (l / t - r) * (1 - cos(t));
	edgeLine[NoSP-1][1] = (l / t - r) * sin(t);

	drawTube(centerLine, edgeLine, NoSP, dDegree);

	for (int j = 0; j < NoSP + 1; j++) {
		delete[] centerLine[j];
	}
	delete[] centerLine;
	for (int j = 0; j < NoSP; j++) {
		delete[] edgeLine[j];
	}
	delete[] edgeLine;
}

list<char> koch_curve[10];
const char rule[] = { 'F', '+', 'F', '-', 'F', '-', 'F', '+', 'F' };
bool calculated = 0;

void calculateLSystem()
{
	list<char>::iterator it;
	koch_curve[0].emplace_back('F');

	for (int i = 1; i < 10; ++i)
	{
		for (it = koch_curve[i - 1].begin(); it != koch_curve[i - 1].end(); ++it)
		{
			koch_curve[i].emplace_back(*it);
		}
		for (it = koch_curve[i].begin(); it != koch_curve[i].end(); ++it)
		{
			if (*it == 'F')
			{
				for (int j = 0; j < 8; ++j)
				{
					koch_curve[i].insert(it, rule[j]);
				}
			}
		}
	}
}

void drawLSystem()
{
	int num_iter = VAL(L_SYSTEM) - 1;
	list<char>::iterator it;
	setDiffuseColor(0.2, 1, 1);
	if (!calculated)
	{
		calculateLSystem();
		calculated = 1;
	}
	if (num_iter < 0)
		return;
	glPushMatrix();
	glRotated(90, 1, 0, 0);
	glTranslated(1, 0, 0);
	for (it = koch_curve[num_iter].begin(); it != koch_curve[num_iter].end(); ++it)
	{
		if (*it == 'F')
		{
			drawBox(0.2, 0.01, 0.01);
			glTranslated(0.2, 0, 0);
		}
		else if (*it == '+')
		{
			glRotated(90, 0, 1, 0);
		}
		else
		{
			glRotated(-90, 0, 1, 0);
		}
	}
	//drawBox(100, 100, 100);
	glPopMatrix();

}

void drawMetaball()
{
	float ball_parameters[4][4] = { {-5, 0, 5, VAL(META_BALL1_RADIUS)}, {5, 0, 5, VAL(META_BALL2_RADIUS)}, {5, 0, -5, VAL(META_BALL3_RADIUS)}, {-5, 0, -5, VAL(META_BALL4_RADIUS)} };
	int num_metaballs = 4;
	int grid_size = 20;
	float threshold = 0.7 * VAL(META_BALL_SCALE);
	CUBE_GRID cube_grid;
	VECTOR3D ball_center;
	VECTOR3D center_to_position;
	float radius_square = 0;
	float scale = 0;

	// intialize the cubes we want
	cube_grid.CreateMemory();
	cube_grid.Init(grid_size);

	// define and intialize metaballs
	METABALL* metaball = new METABALL[num_metaballs];

	metaball[0].Init(VECTOR3D(ball_parameters[0][0], ball_parameters[0][1], ball_parameters[0][2]), pow(ball_parameters[0][3], 2));
	metaball[1].Init(VECTOR3D(ball_parameters[1][0], ball_parameters[1][1], ball_parameters[1][2]), pow(ball_parameters[1][3], 2));
	metaball[2].Init(VECTOR3D(ball_parameters[2][0], ball_parameters[2][1], ball_parameters[2][2]), pow(ball_parameters[2][3], 2));
	metaball[3].Init(VECTOR3D(ball_parameters[3][0], ball_parameters[3][1], ball_parameters[3][2]), pow(ball_parameters[3][3], 2));

	for (int i = 0; i < num_metaballs; ++i)
	{
		radius_square = metaball[i].squaredRadius;
		ball_center = metaball[i].position;

		for (int j = 0; j < cube_grid.numVertices; j++)
		{
			center_to_position.x = cube_grid.vertices[j].position.x - ball_center.x;
			center_to_position.y = cube_grid.vertices[j].position.y - ball_center.y;
			center_to_position.z = cube_grid.vertices[j].position.z - ball_center.z;

			// square distance from the center to the point
			float distance_square = pow(center_to_position.x, 2) + pow(center_to_position.y, 2) + pow(center_to_position.z, 2);
			distance_square = (distance_square == 0) ? 0.00001 : distance_square;

			cube_grid.vertices[j].value += radius_square / distance_square;
			scale = radius_square / pow(distance_square, 2);
			// get the normal vector
			cube_grid.vertices[j].normal.x += center_to_position.x * scale;
			cube_grid.vertices[j].normal.y += center_to_position.y * scale;
			cube_grid.vertices[j].normal.z += center_to_position.z * scale;
		}
	}

	cube_grid.DrawSurface(threshold);
	cube_grid.FreeMemory();
	delete[] metaball;

}

bool initialized = false;
void drawSpring() {
	if (ModelerApplication::Instance()->GetUI()->simulate()) {
		Mat4d CameraMatrix = getModelViewMatrix();
		glPushMatrix();
		glScaled(1, -1, 1);
		glRotated(90, 1, 0, 0);
		glScaled(SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR);

		glTranslated(0, 0, 180);
		drawCylinder(40, 2, 2);
		glPushMatrix();
		setDiffuseColor(0, 0, 1);
		glTranslated(-70, -70, 40);
		drawBox(140, 140, 3);
		glPopMatrix();
		setDiffuseColor(1, 1, 0);
		drawSphere(10);
		if (!initialized) {
			// add particles
			SpawnSpringParticles(CameraMatrix,true);
			int numNodes = 3;
			for (int i = 0; i < numNodes; ++i) {
				glTranslated(20, 20, -20);
				if (i == numNodes - 1) {
					SpawnSpringParticles(CameraMatrix,false,true);
					break;
				}
				SpawnSpringParticles(CameraMatrix);
			}
			initialized = true;
			// add neighbors for the particles
			SpringParticleSystem* ps = dynamic_cast<SpringParticleSystem*>(ModelerApplication::Instance()->GetParticleSystem());
			vector<Particle*> particles = ps->getCurParticles();
			vector<Particle*>::iterator iter;
			for (iter = particles.begin(); iter != particles.end(); ++iter) {
				if (iter == particles.begin()) continue;
				vector<Particle*>::iterator i = iter;
				--i;
				dynamic_cast<SpringParticle*>(*iter)->addNeighbor(dynamic_cast<SpringParticle*>(*i));
				++i; ++i;
				if (i != particles.end()) {
					dynamic_cast<SpringParticle*>(*iter)->addNeighbor(dynamic_cast<SpringParticle*>(*i));
				}
			}
		}

		glPopMatrix();
	}
	else {
		glPushMatrix();
		glScaled(1, -1, 1);
		glRotated(90, 1, 0, 0);
		glScaled(SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR);

		glTranslated(0, 0, 180);
		drawCylinder(40, 2, 2);
		glPushMatrix();
		setDiffuseColor(0, 0, 1);
		glTranslated(-70, -70, 40);
		drawBox(140, 140, 3);
		glPopMatrix();
		setDiffuseColor(1, 1, 0);
		drawSphere(10);
		glPopMatrix();
	}
}

#ifndef cpcount
#define cpcount 13
#endif
void drawCloth() {
	if (ModelerApplication::Instance()->GetUI()->simulate()) {
		Mat4d CameraMatrix = getModelViewMatrix();
		glPushMatrix();
		glScaled(1, -1, 1);
		glRotated(90, 1, 0, 0);
		glScaled(SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR);

		glPushMatrix();
		glTranslated(-180, 0, 180);
		setDiffuseColor(1, 1, 0);
		drawSphere(10);
		glTranslated(360, 0, 0);
		drawSphere(10);
		glPopMatrix();

		if (!initialized) {
			initialized = true;
			double gap = (double)360 / (cpcount - 1);
			ClothParticle* grid[cpcount][cpcount] = { nullptr };
			glTranslated(-180, 0, 180);
			for (int j = 0; j < cpcount; ++j) {
				glPushMatrix();
				for (int i = 0; i < cpcount; ++i) {
					if (j == 0 && (i == 0 || i == cpcount-1)) {
						grid[j][i] = SpawnClothParticles(CameraMatrix, true);
					}
					else {
						grid[j][i] = SpawnClothParticles(CameraMatrix);
					}
					glTranslated(gap, 0, 0);
				}
				glPopMatrix();
				glRotated(double(180)/cpcount, 1, 0, 0);
				glTranslated(0, 0, -gap);
			}
			// add neighbors
			for (int j = 0; j < cpcount; ++j) {
				for (int i = 0; i < cpcount; ++i) {
					// type0
					if (j - 1 >= 0) grid[j][i]->addNeighbor(grid[j - 1][i],0);
					if (j + 1 < cpcount) grid[j][i]->addNeighbor(grid[j + 1][i],0);
					if (i - 1 >= 0) grid[j][i]->addNeighbor(grid[j][i-1], 0);
					if (i + 1 < cpcount) grid[j][i]->addNeighbor(grid[j][i+1], 0);
					// type1
					if (j - 1 >= 0 && i - 1 >= 0) grid[j][i]->addNeighbor(grid[j - 1][i - 1], 1);
					if (j - 1 >= 0 && i + 1 < cpcount) grid[j][i]->addNeighbor(grid[j - 1][i + 1], 1);
					if (j + 1 < cpcount && i - 1 >= 0) grid[j][i]->addNeighbor(grid[j + 1][i - 1], 1);
					if (j + 1 < cpcount && i + 1 < cpcount) grid[j][i]->addNeighbor(grid[j + 1][i + 1], 1);
					// type2
					if (j - 2 >= 0) grid[j][i]->addNeighbor(grid[j - 2][i], 2);
					if (j + 2 < cpcount) grid[j][i]->addNeighbor(grid[j + 2][i], 2);
					if (i - 2 >= 0) grid[j][i]->addNeighbor(grid[j][i - 2], 2);
					if (i + 2 < cpcount) grid[j][i]->addNeighbor(grid[j][i + 2], 2);
				}
			}
		}

		glPopMatrix();
	}
	else {
		glPushMatrix();
		glScaled(1, -1, 1);
		glRotated(90, 1, 0, 0);
		glScaled(SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR);

		glTranslated(-180, 0, 180);
		setDiffuseColor(1, 1, 0);
		drawSphere(10);
		glTranslated(360, 0, 0);
		drawSphere(10);
		glPopMatrix();
	}
}

#ifndef hcount
#define hcount 12
#endif
void drawHair() {
	if (ModelerApplication::Instance()->GetUI()->simulate()) {
		Mat4d CameraMatrix = getModelViewMatrix();
		glPushMatrix();
		glScaled(1, -1, 1);
		glRotated(90, 1, 0, 0);
		glScaled(SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR);

		glPushMatrix();

		if (VAL(WIND)==1) {
			glPushMatrix();
			setDiffuseColor(0, 1, 1);
			glTranslated(0, -25, -20);
			glTranslated(-500, 0, -500);
			drawBox(1000, 0.01, 1000);
			glPopMatrix();
		}

		glRotated(90, 1, 0, 0);
		glTranslated(-20, -20, 0);
		setDiffuseColor(1, 1, 0);
		drawBox(40, 40/3, -0.1);

		if (!initialized) {
			initialized = true;
			const int n = hcount; double gap = double(40) / (n - 1);
			HairParticle* grid[n/3][n][18] = { nullptr };
			for (int j = 0; j < n/3; ++j) {
				glPushMatrix();
				for (int i = 0; i < n; ++i) {
					glPushMatrix();
					grid[j][i][0] = SpawnHairParticles(CameraMatrix, true);
					for (int k = 1; k < 18; ++k) {
						glTranslated(0, 0, 3);
						if (k == 1) {
							grid[j][i][k] = SpawnHairParticles(CameraMatrix, true);
							continue;
						}
						grid[j][i][k] = SpawnHairParticles(CameraMatrix);
					}
					glPopMatrix();
					glTranslated(gap, 0, 0);
				}
				glPopMatrix();
				glTranslated(0, gap, 0);
			}

			// add neighbors
			for (int j = 0; j < n/3; ++j) {
				for (int i = 0; i < n; ++i) {
					for (int k = 0; k < 18; ++k) {
						// type0
						if (k - 1 >= 0) grid[j][i][k]->addNeighbor(grid[j][i][k - 1], 0);
						if (k + 1 < 18) grid[j][i][k]->addNeighbor(grid[j][i][k + 1], 0);
						// type1
						if (k - 2 >= 0) grid[j][i][k]->addNeighbor(grid[j][i][k - 2], 1);
						if (k + 2 < 18) grid[j][i][k]->addNeighbor(grid[j][i][k + 2], 1);
					}
				}
			}
		}
		glPopMatrix();
		glPopMatrix();
	}
	else {
		glPushMatrix();
		glScaled(1, -1, 1);
		glRotated(90, 1, 0, 0);
		glScaled(SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR);

		glRotated(90, 1, 0, 0);
		glTranslated(-20, -20, 0);
		setDiffuseColor(1, 1, 0);
		drawBox(40, 40/3, -0.1);

		glPopMatrix();
	}
}

void drawFlock() {
	Mat4d CameraMatrix = getModelViewMatrix();
	glPushMatrix();
	glScaled(1, -1, 1);
	glRotated(90, 1, 0, 0);
	glScaled(SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR);
	
	setDiffuseColor(1, 1, 0);
	drawSphere(10);

	glPushMatrix();
	glRotated(VAL(FLOCKTARGET), 0, 0, 1);
	glTranslated(300, 0, 0);
	setDiffuseColor(1, 0, 0);
	drawSphere(20);
	glPopMatrix();

	if (ModelerApplication::Instance()->GetUI()->simulate() && VAL(FLOCKSPAWN)) {
		SpawnFlockParticles(CameraMatrix);
	}

	glPopMatrix();
}

Vec3f surface_points(double u, double v, const vector<Vec3f>& control_points)
{
	Vec4f M_1(u * u * u, u * u, u, 1);
	Vec4f M_2(v * v * v, v * v, v, 1);
	Mat4f M_3(-1, 3, -3, 1, 3, -6, 3, 0, -3, 0, 3, 0, 1, 4, 1, 0);
	
	Mat4f g_y(control_points[0][1], control_points[1][1], control_points[2][1], control_points[3][1],
		control_points[4][1], control_points[5][1], control_points[6][1], control_points[7][1],
		control_points[8][1], control_points[9][1], control_points[10][1], control_points[11][1],
		control_points[12][1], control_points[13][1], control_points[14][1], control_points[15][1]);
	
	Mat4f g_x(control_points[0][0], control_points[1][0], control_points[2][0], control_points[3][0],
			control_points[4][0], control_points[5][0], control_points[6][0], control_points[7][0],
			control_points[8][0], control_points[9][0], control_points[10][0], control_points[11][0],
			control_points[12][0], control_points[13][0], control_points[14][0], control_points[15][0]);

	Mat4f g_z(control_points[0][2], control_points[1][2], control_points[2][2], control_points[3][2],
		control_points[4][2], control_points[5][2], control_points[6][2], control_points[7][2],
		control_points[8][2], control_points[9][2], control_points[10][2], control_points[11][2],
		control_points[12][2], control_points[13][2], control_points[14][2], control_points[15][2]);

	Vec3f points;
	points[0] = M_1 * (M_3 * g_x * M_3.transpose() * M_2) / (double)36;
	points[1] = M_1 * (M_3 * g_y * M_3.transpose() * M_2) / (double)36;
	points[2] = M_1 * (M_3 * g_z * M_3.transpose() * M_2) / (double)36;

	return points;
}

void draw_surface()
{
	// cout << ModelerApplication::Instance()->GetControlValue(CTRLPOINT) << endl;
	vector<Vec3f> control_points = { Vec3f(0.0, 0.0, 0.0), Vec3f(0.33, 0.0, 0.0), Vec3f(0.67, 0.0, 0.0), Vec3f(1.0, 0.0, 0.0),
	Vec3f(0.0, 0.0, 0.33), Vec3f(0.33, 1.0, 0.33), Vec3f(0.67, 1.0, 0.33), Vec3f(1.0, 0.0, 0.33),
	Vec3f(0.0, 0.0, 0.67), Vec3f(0.33, 1.0, 0.67), Vec3f(0.67, 1.0, 0.67), Vec3f(1.0, 0.0, 0.67),
	Vec3f(0.0, 0.0, 1.0), Vec3f(0.33, 0.0, 1.0), Vec3f(0.67, 0.0, 1.0), Vec3f(1.0, 0.0, 1.0),
	};

	control_points[0][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_0);
	control_points[1][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_1);
	control_points[2][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_2);
	control_points[3][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_3);
	control_points[4][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_4);
	control_points[5][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_5);
	control_points[6][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_6);
	control_points[7][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_7);
	control_points[8][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_8);
	control_points[9][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_9);
	control_points[10][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_10);
	control_points[11][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_11);
	control_points[12][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_12);
	control_points[13][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_13);
	control_points[14][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_14);
	control_points[15][1] = ModelerApplication::Instance()->GetControlValue(CTRLPOINT_15);

	int sampling_rate = 90;
	for (int v = 0; v < sampling_rate; ++v)
	{
		glBegin(GL_QUAD_STRIP);
		for (int u = 0; u <= sampling_rate; ++u)
		{
			Vec3f temp_point_1 = surface_points((double)u / sampling_rate, (double)v / sampling_rate, control_points);
			Vec3f temp_point_2 = surface_points((double)u / sampling_rate, (double)(v + 1) / sampling_rate, control_points);
			Vec3f temp_point_3 = surface_points((double)(u + 1) / sampling_rate, (double)v / sampling_rate, control_points);

			Vec3f normal = (temp_point_2 - temp_point_1) ^ (temp_point_3 - temp_point_1);
			normal.normalize();
			glNormal3d(-normal[0], normal[1], -normal[2]);
			
			glVertex3f(temp_point_1[0], temp_point_1[1], temp_point_1[2]);
			glVertex3f(temp_point_2[0], temp_point_2[1], temp_point_2[2]);
		}
		glEnd();
	}

}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out Unicycle
void Unicycle::draw()
{
	// This call takes care of a lot of the nasty projection 
	// matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
	ModelerView::draw();
	
	int type = int(VAL(PS)+0.01);
	ModelerApplication::Instance()->SetParticleSystem((SampleParticleSystems)type);
	switch (type) {
	case DEFAULT: break;
	case SPRING: { drawSpring(); endDraw(); return; }
	case CLOTH: { drawCloth(); endDraw(); return; }
	case HAIR: { drawHair(); endDraw(); return; }
	case FLOCK: { drawFlock(); endDraw(); return; }
	default: break;
	}

	Mat4d CameraMatrix = getModelViewMatrix();
	// draw_surface();

	// glEnable(GL_LIGHT2);
	static GLfloat lightDiffuse[] = { 5, 5, 5, 1 };			// strengthen the intencity of the light0 and light1    BW1
	static GLfloat lightAmbient[] = { 0.1, 0.1, 0.1, 1 };   // create an ambient light with low intencity
	// glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	// glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse);
	// glLightfv(GL_LIGHT2, GL_AMBIENT, lightAmbient);

	// if (ModelerUserInterface::m_controlsAnimOnMenu->value() != 0) animated = true;
	// else animated = false; // Not compatible with PA4

	drawLSystem();

	// draw a surface
	glPushMatrix();
	glScaled(10, 10, 10);
	glTranslated(0, -0.7, 0);
	setDiffuseColor(0, 1, 0);
	draw_surface();
	glPopMatrix();

	if (VAL(META_BALL_CONTROL) == 1)
	{
		//setDiffuseColor(0, 1, 1);
		glPushMatrix();
		glEnable(GL_NORMALIZE);
		glTranslated(-3, 0, 0);
		glRotated(90, 1, 0, 0);
		glScaled(0.1, 0.1, 0.1);
		drawMetaball();
		glPopMatrix();
	}

	glPushMatrix();
	glScaled(1, -1, 1);
	glRotated(90, 1, 0, 0);

	// draw the unicycle
	setAmbientColor(0.1f,0.1f,0.1f);
	glScaled(SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR);

	// draw IK Target Point
	if (VAL(IK_VISUALIZE) == 1) {
		setDiffuseColor(0, 1, 0);
		glPushMatrix();
		glTranslated(VAL(IK_TARGETX), VAL(IK_TARGETY), VAL(IK_TARGETZ));
		drawSphere(SADDLE_HEIGHT / 3);
		glPopMatrix();
	}

	setDiffuseColor(0.4, 0.4, 0.4);

	// For IK control
	if (VAL(IK_MODE) == 1) {
		if (IK_Mode_Prev == false) {
			IK_Compute();
			IK_Mode_Prev = true;
		}
		for (int item = 0; item < NUMCONTROLS; item++) {
			if ((!IK_related[item]) && item != XPOS && item != YPOS && item != ZPOS) continue; // Do not change unrelated controls
			SET(item, VAL(IK_MOVE) / 100.0 * IK_target[item] + (1 - VAL(IK_MOVE) / 100.0) * IK_init[item]); //"(int)" & floor() may be needed
		}
		// Reset CRANK_DIR so that the final animation is more smooth
		/*if ((int)(IK_target[CRANK_DIR] - IK_init[CRANK_DIR]) % 360 <= (int)(IK_init[CRANK_DIR] - IK_target[CRANK_DIR]) % 360) {
			SET(CRANK_DIR, ((int)(IK_init[CRANK_DIR] + VAL(IK_MOVE) * ((int)(IK_target[CRANK_DIR] - IK_init[CRANK_DIR]) % 360)) % 360));
		} // Move clockwise to target
		else { // Move counterclockwise to target
			SET(CRANK_DIR, ((int)(IK_init[CRANK_DIR] - VAL(IK_MOVE) * ((int)(IK_init[CRANK_DIR] - IK_target[CRANK_DIR]) % 360)) % 360));
		}*/
		// This trial failed. Do not have time to find the bugs in this.
	}
	else {
		IK_Mode_Prev = false;
	}

	glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
		//draw the floor
		setAmbientColor(0, 0, 1);
		glPushMatrix();
		glTranslated(-500, -500, -WHEEL_RADIUS - 0.1);
		drawBox(1000, 1000, 0.1);
		glPopMatrix();
		setAmbientColor(0.1, 0.1, 0.1);
		//draw the axle
		glPushMatrix();
		if (VAL(HAPPINESS) == -1) {
			glRotated(VAL(AXLE_DIR) + 30 * sin(PI / 60 /* time*/), 0, 0, 1);
		}
		else {
			glRotated(VAL(AXLE_DIR), 0, 0, 1);
		}
		glRotated(90, 0, 1, 0);
		drawCylinder(AXLE_LENGTH_HALF, AXLE_RADIUS, AXLE_RADIUS);
		glScaled(-1, 1, -1);
		drawCylinder(AXLE_LENGTH_HALF, AXLE_RADIUS, AXLE_RADIUS);
		glScaled(-1, 1, -1);
		glRotated(-90, 0, 1, 0);
		if (VAL(LEVEL_OF_DETAILS) >= 2) {
			//draw the wheel
			glPushMatrix();
			// ====Spawn Particles====
			if (ModelerApplication::Instance()->GetParticleSystem()->isSimulate()) {
				glPushMatrix();
				srand((time(0)*seed)%65472);
				seed++;
				for (int i = 0; i < 4; ++i) {
					double rand_angle = double(rand() % 500) / 500 * 25;
					glRotated(-rand_angle, 1, 0, 0);
					double cur_crank_dir = VAL(CRANK_DIR);
					double rate = 2 * (VAL(CRANK_DIR) - prev_crank_dir) * WHEEL_RADIUS * ModelerApplication::Instance()->GetUI()->fps();
					rate /= 10;
					double rand_vx = double(rand() % 500) / 500 * rate / 5;
					if (rand() % 2) rand_vx *= -1;
					Vec4d v(rand_vx, -rate, 0, 1);
					double rand_x = double(rand() % 500) / 500 * WHEEL_WIDTH - WHEEL_WIDTH / 2;
					glTranslated(rand_x, 0, -WHEEL_RADIUS);
					if (rate > 0) {
						SpawnParticles(CameraMatrix, v);
					}
				}
				glPopMatrix();
			}
			prev_crank_dir = VAL(CRANK_DIR);
			glRotated(-2 * (VAL(CRANK_DIR)) * pow(2, VAL(HAPPINESS)), 1, 0, 0);
			// =======================
			setDiffuseColor(0.05, 0.05, 0.05);
			if (VAL(WHEEL_QUALITY) == 0) {
				glTranslated(-WHEEL_WIDTH / 2, 0, 0);
				glRotated(90, 0, 1, 0);
				drawCylinder(WHEEL_WIDTH, WHEEL_RADIUS, WHEEL_RADIUS);
			}
			else if (VAL(WHEEL_QUALITY) == 1) {
				glRotated(90, 0, 1, 0);
				drawTorus(WHEEL_RADIUS - WHEEL_WIDTH / 2, WHEEL_WIDTH / 2, dDegree);
				glRotated(-90, 0, 1, 0);
				//draw spokes
				setDiffuseColor(0.7, 0.7, 0.7);
				glTranslated(SPOKE_DIST, 0, 0);
				for (int deg = 0; deg < 360; deg += 40) {
					drawCylinder(SPOKE_LENGTH, SPOKE_RADIUS, SPOKE_RADIUS);
					glRotated(10, -1, 0, 0);
					drawCylinder(SPOKE_LENGTH, SPOKE_RADIUS, SPOKE_RADIUS);
					glRotated(30, -1, 0, 0);
				}
				glTranslated(-2 * SPOKE_DIST, 0, 0);
				glRotated(3, -1, 0, 0);
				for (int deg = 0; deg < 360; deg += 40) {
					drawCylinder(SPOKE_LENGTH, SPOKE_RADIUS, SPOKE_RADIUS);
					glRotated(10, -1, 0, 0);
					drawCylinder(SPOKE_LENGTH, SPOKE_RADIUS, SPOKE_RADIUS);
					glRotated(30, -1, 0, 0);
				}
				glTranslated(SPOKE_DIST, 0, 0);
			}
			glPopMatrix();
			//draw the left crank
			setDiffuseColor(0.4, 0.4, 0.4);
			glPushMatrix();
			glTranslated(-CRANK_DIST, 0, 0);
			glRotated(-90 - (VAL(CRANK_DIR)) * pow(2, VAL(HAPPINESS)), 1, 0, 0);
			drawCylinder(CRANK_LENGTH, CRANK_RADIUS_ROOT, CRANK_RADIUS_TOP);
			glTranslated(0, 0, CRANK_LENGTH);
			glRotated(90 + (VAL(CRANK_DIR)) * pow(2, VAL(HAPPINESS)), 1, 0, 0);
			if (VAL(LEVEL_OF_DETAILS) >= 3) {
				//draw the left pedal
				setDiffuseColor(0.05, 0.05, 0.05);
				glPushMatrix();
				glTranslated(-PEDAL_LENGTH, -PEDAL_WIDTH - PEDAL_GAP / 2, -PEDAL_HEIGHT / 2);
				drawBox(PEDAL_LENGTH, PEDAL_WIDTH, PEDAL_HEIGHT);
				glTranslated(0, PEDAL_WIDTH + PEDAL_GAP, 0);
				drawBox(PEDAL_LENGTH, PEDAL_WIDTH, PEDAL_HEIGHT);
				glPopMatrix();
			}
			glPopMatrix();
			//draw the right crank
			setDiffuseColor(0.4, 0.4, 0.4);
			glPushMatrix();
			glTranslated(CRANK_DIST, 0, 0);
			glRotated(90- (VAL(CRANK_DIR)) * pow(2, VAL(HAPPINESS)), 1, 0, 0);
			drawCylinder(CRANK_LENGTH, CRANK_RADIUS_ROOT, CRANK_RADIUS_TOP);
			glTranslated(0, 0, CRANK_LENGTH);
			glRotated(-90+ (VAL(CRANK_DIR)) * pow(2, VAL(HAPPINESS)), 1, 0, 0);
			if (VAL(LEVEL_OF_DETAILS) >= 3) {
				//draw the right pedal
				setDiffuseColor(0.05, 0.05, 0.05);
				glPushMatrix();
				glTranslated(0, -PEDAL_WIDTH - PEDAL_GAP / 2, -PEDAL_HEIGHT / 2);
				drawBox(PEDAL_LENGTH, PEDAL_WIDTH, PEDAL_HEIGHT);
				glTranslated(0, PEDAL_WIDTH + PEDAL_GAP, 0);
				drawBox(PEDAL_LENGTH, PEDAL_WIDTH, PEDAL_HEIGHT);
				glPopMatrix();
			}
			glPopMatrix();
			//draw the tube
			//left lower tube
			setDiffuseColor(1, 0, 0);
			if (VAL(HAPPINESS) == 1 && VAL(BEND) != 0) { // do some adjustment so that saddle --- axle do not rotate
				double a = TUBE_HEIGHT_UPPER + SEATPOST_HEIGHT;
				double b = TUBE_HEIGHT_LOWER + TUBE_HEIGHT_MIDDLE - AXLE_RADIUS;
				double theta = VAL(BEND) / 180 * PI;
				double phi = atan(a / theta * (1 - cos(theta)) / (b + a / theta * sin(theta)));
				double angle = phi / PI * 180;
				glRotated(-(VAL(TUBE_DIR) - angle), 1, 0, 0);
			}
			else {
				glRotated(-VAL(TUBE_DIR), 1, 0, 0);
			}
			glPushMatrix();
			glTranslated(-TUBE_DIST - TUBE_WIDTH / 2, -TUBE_LENGTH / 2, -AXLE_RADIUS);
			drawBox(TUBE_WIDTH, TUBE_LENGTH, TUBE_HEIGHT_LOWER);
			glPopMatrix();
			//right lower tube
			glPushMatrix();
			glTranslated(TUBE_DIST - TUBE_WIDTH / 2, -TUBE_LENGTH / 2, -AXLE_RADIUS);
			drawBox(TUBE_WIDTH, TUBE_LENGTH, TUBE_HEIGHT_LOWER);
			glPopMatrix();
			//middle tube
			glPushMatrix();
			glTranslated(-TUBE_DIST - TUBE_WIDTH / 2, -TUBE_LENGTH / 2, -AXLE_RADIUS + TUBE_HEIGHT_LOWER);
			drawBox(TUBE_WIDTH + TUBE_DIST * 2, TUBE_LENGTH, TUBE_HEIGHT_MIDDLE);
			glTranslated(TUBE_DIST + TUBE_WIDTH / 2, TUBE_LENGTH / 2, AXLE_RADIUS - TUBE_HEIGHT_LOWER);

			if (VAL(LEVEL_OF_DETAILS) >= 3) {
				glPushMatrix();
				glTranslated(0, 0, TUBE_HEIGHT_LOWER + TUBE_HEIGHT_MIDDLE - AXLE_RADIUS);
				if (VAL(BEND) == 0 && (VAL(HAPPINESS) != -1 || !(animated))) {
					drawCylinder(TUBE_HEIGHT_UPPER, TUBE_RADIUS, TUBE_RADIUS);
					glTranslated(0, 0, TUBE_HEIGHT_UPPER);
					setDiffuseColor(0.4, 0.4, 0.4);
					drawCylinder(SEATPOST_HEIGHT, SEATPOST_RADIUS, SEATPOST_RADIUS);
					glTranslated(0, 0, SEATPOST_HEIGHT);
				}
				else {
					//upper tube
					double rad = VAL(BEND) / 2 / 180 * PI;
					if (VAL(HAPPINESS) == -1 && animated) rad = min(max(rad,PI/180),(ceil(VAL(BEND) * 1.2 / 2 + 10)) / 180 * PI);
					else if (VAL(HAPPINESS) == -1 && !(animated)) rad = (ceil(VAL(BEND) * 1.2 / 2 + 10)) / 180 * PI; // To show the "mood" even without animation
					glPushMatrix();
					glScaled(1, -1, 1);
					glRotated(90, 0, 0, 1);
					glRotated(-90, 1, 0, 0);
					glScaled(-1, -1, 1);
					drawPartialTorus(TUBE_HEIGHT_UPPER, TUBE_RADIUS, rad, dDegree);
					glPopMatrix();

					//draw the seatpost
					setDiffuseColor(0.4, 0.4, 0.4);
					glTranslated(0, (1 - cos(rad)) * TUBE_HEIGHT_UPPER / (rad),
						sin(rad) * TUBE_HEIGHT_UPPER / (rad));
					glRotated(floor(rad * 180 / PI + 0.1), -1, 0, 0);
					glPushMatrix();
					glScaled(1, -1, 1);
					glRotated(90, 0, 0, 1);
					glRotated(-90, 1, 0, 0);
					glScaled(-1, -1, 1);
					drawPartialTorus(SEATPOST_HEIGHT, SEATPOST_RADIUS, rad, dDegree);
					glPopMatrix();
					glTranslated(0, (1 - cos(rad)) * SEATPOST_HEIGHT / (rad),
						sin(rad) * SEATPOST_HEIGHT / (rad));
					glRotated(floor(rad * 180 / PI + 0.1), -1, 0, 0);
				}

				if (VAL(LEVEL_OF_DETAILS) >= 4) {
					//draw the saddle
					setDiffuseColor(0.05, 0.05, 0.05);
					glPushMatrix();
					glRotated(-VAL(SADDLE_DIR), 0, 0, 1);
					glTranslated(-SADDLE_WIDTH / 2, -SADDLE_DIST, 0);

					if (VAL(SADDLE_QUALITY) >= 1) {
						setDiffuseColor(1, float(240 / 255), 1);
						draw_pad(); // BW5
						setDiffuseColor(0.05, 0.05, 0.05);
					}
					//draw_spring();
					if (VAL(SADDLE_QUALITY) == 2) {
						setDiffuseColor(0.4, 0.4, 0.4);
						glPushMatrix();
						glTranslated(SADDLE_WIDTH / 2, SADDLE_DIST, 0);
						glTranslated(-SPRING_X, -SPRING_Y, 0);
						glScaled(-1, -1, -1);
						drawCylinder(SPRING_CYLINDER_HEIGHT, SPRING_CYLINDER_RADIUS, SPRING_CYLINDER_RADIUS);
						glScaled(-1, -1, -1);
						glTranslated(0, 0, SPRING_RADIUS_SMALL);
						for (int i = 0; i < 9; i++) {
							glTranslated(0, 0, -SPRING_DIST_APART);
							drawTorus(SPRING_RADIUS_BIG[i], SPRING_RADIUS_SMALL, dDegree);
						}
						glPopMatrix();
						glPushMatrix();
						glTranslated(SADDLE_WIDTH / 2, SADDLE_DIST, 0);
						glTranslated(SPRING_X, -SPRING_Y, 0);
						glScaled(-1, -1, -1);
						drawCylinder(SPRING_CYLINDER_HEIGHT, SPRING_CYLINDER_RADIUS, SPRING_CYLINDER_RADIUS);
						glScaled(-1, -1, -1);
						glTranslated(0, 0, SPRING_RADIUS_SMALL);
						for (int i = 0; i < 9; i++) {
							glTranslated(0, 0, -SPRING_DIST_APART);
							drawTorus(SPRING_RADIUS_BIG[i], SPRING_RADIUS_SMALL, dDegree);
						}
						glPopMatrix();
					}

					drawTriangle(0,0,0,
								 SADDLE_WIDTH,0,0,
								 SADDLE_WIDTH/2,SADDLE_LENGTH,0);
					drawTriangle(0, 0, SADDLE_HEIGHT,
								 SADDLE_WIDTH / 2, SADDLE_LENGTH, SADDLE_HEIGHT,
								 SADDLE_WIDTH, 0, SADDLE_HEIGHT);
					drawTriangle(0, 0, 0,
								 0, 0, SADDLE_HEIGHT,
								 SADDLE_WIDTH, 0, 0);
					drawTriangle(0, 0, SADDLE_HEIGHT,
								 SADDLE_WIDTH, 0, SADDLE_HEIGHT,
								 SADDLE_WIDTH, 0, 0);
					drawTriangle(0, 0, 0,
								 SADDLE_WIDTH / 2, SADDLE_LENGTH, 0,
								 SADDLE_WIDTH / 2, SADDLE_LENGTH, SADDLE_HEIGHT);
					drawTriangle(0, 0, 0,
								 SADDLE_WIDTH / 2, SADDLE_LENGTH, SADDLE_HEIGHT,
								 0, 0, SADDLE_HEIGHT);
					drawTriangle(SADDLE_WIDTH, 0, 0,
								 SADDLE_WIDTH / 2, SADDLE_LENGTH, SADDLE_HEIGHT,
								 SADDLE_WIDTH / 2, SADDLE_LENGTH, 0);
					drawTriangle(SADDLE_WIDTH, 0, 0,
								 SADDLE_WIDTH, 0, SADDLE_HEIGHT,
								 SADDLE_WIDTH / 2, SADDLE_LENGTH, SADDLE_HEIGHT);

					if (VAL(LEVEL_OF_DETAILS) >= 5) {
						//draw the label
						glPushMatrix();
						setDiffuseColor(1, 0.9, 0.9);
						glTranslated(SADDLE_WIDTH / 2, SADDLE_DIST + 50, -LABEL_STRING_HEIGHT);
						glRotated(VAL(LABEL_DIR), 0, 0, 1);
						drawCylinder(LABEL_STRING_HEIGHT, LABEL_STRING_RADIUS, LABEL_STRING_RADIUS);
						glTranslated(-LABEL_LENGTH / 2, -LABEL_STRING_RADIUS, -LABEL_HEIGHT);
						drawBox(LABEL_LENGTH, 2 * LABEL_STRING_RADIUS, LABEL_HEIGHT);

						//BW4 Texture Mapping
						drawLabelTexture();
						glTranslated(LABEL_LENGTH, 2 * LABEL_STRING_RADIUS, 0);
						glScaled(-1, -1, 1);
						drawLabelTexture();

						glPopMatrix();
					}

					// Draw IK Fixed Point
					if (VAL(IK_VISUALIZE) == 1) {
						glTranslated(SADDLE_WIDTH / 2, SADDLE_LENGTH, 0);
						setDiffuseColor(1, 0, 0);
						drawSphere(SADDLE_HEIGHT / 3);
					}

					glPopMatrix();

				}
				glPopMatrix();
			}
			glPopMatrix();
		}
		glPopMatrix();
	glPopMatrix();

	endDraw();
}

Mat4d getModelViewMatrix() {
	GLdouble m[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, m);
	Mat4d matMV(m[0], m[1], m[2], m[3],
				m[4], m[5], m[6], m[7],
				m[8], m[9], m[10], m[11],
				m[12], m[13], m[14], m[15]);
	return matMV.transpose();
}

void SpawnParticles(Mat4d cameraTransforms, Vec4d local_v) {
	Mat4d MV = getModelViewMatrix();
	Mat4d WorldMatrix = cameraTransforms.inverse() * MV;
	Vec4d WorldPoint = WorldMatrix * Vec4d(0, 0, 0, 1);
	Vec3d x(WorldPoint[0], WorldPoint[1], WorldPoint[2]);
	Vec4d WorldVel = WorldMatrix * local_v;
	Vec3d v(WorldVel[0], WorldVel[1], WorldVel[2]);
	ModelerApplication::Instance()->GetParticleSystem()->add(new Particle(x,v));
}

void SpawnFlockParticles(Mat4d cameraTransforms) {
	Mat4d MV = getModelViewMatrix();
	Mat4d WorldMatrix = cameraTransforms.inverse() * MV;
	Vec4d WorldPoint = WorldMatrix * Vec4d(0, 0, 0, 1);
	Vec3d x(WorldPoint[0], WorldPoint[1], WorldPoint[2]);
	ModelerApplication::Instance()->GetParticleSystem()->add(new FlockParticle(x));
}

void SpawnSpringParticles(Mat4d cameraTransforms, bool isStartPt, bool heavier) {
	Mat4d MV = getModelViewMatrix();
	Mat4d WorldMatrix = cameraTransforms.inverse() * MV;
	Vec4d WorldPoint = WorldMatrix * Vec4d(0, 0, 0, 1);
	Vec3d x(WorldPoint[0], WorldPoint[1], WorldPoint[2]);
	if (heavier) {
		ModelerApplication::Instance()->GetParticleSystem()->add(new SpringParticle(x,false,true));
	}
	else if (isStartPt) {
		ModelerApplication::Instance()->GetParticleSystem()->add(new SpringParticle(x, true, false));
	}
	else {
		ModelerApplication::Instance()->GetParticleSystem()->add(new SpringParticle(x));
	}
}

ClothParticle* SpawnClothParticles(Mat4d cameraTransforms, bool isStartPt) {
	Mat4d MV = getModelViewMatrix();
	Mat4d WorldMatrix = cameraTransforms.inverse() * MV;
	Vec4d WorldPoint = WorldMatrix * Vec4d(0, 0, 0, 1);
	Vec3d x(WorldPoint[0], WorldPoint[1], WorldPoint[2]);
	if (isStartPt) {
		ClothParticle* p = new ClothParticle(x, true);
		ModelerApplication::Instance()->GetParticleSystem()->add(p);
		return p;
	}
	else {
		ClothParticle* p = new ClothParticle(x);
		ModelerApplication::Instance()->GetParticleSystem()->add(p);
		return p;
	}
}

HairParticle* SpawnHairParticles(Mat4d cameraTransforms, bool isStartPt) {
	Mat4d MV = getModelViewMatrix();
	Mat4d WorldMatrix = cameraTransforms.inverse() * MV;
	Vec4d WorldPoint = WorldMatrix * Vec4d(0, 0, 0, 1);
	Vec3d x(WorldPoint[0], WorldPoint[1], WorldPoint[2]);
	if (isStartPt) {
		HairParticle* p = new HairParticle(x, true);
		ModelerApplication::Instance()->GetParticleSystem()->add(p);
		return p;
	}
	else {
		HairParticle* p = new HairParticle(x);
		ModelerApplication::Instance()->GetParticleSystem()->add(p);
		return p;
	}
}

//BW4 Texture Mapping
void Unicycle::drawLabelTexture() {
	setDiffuseColor(1, 1, 1);
	glEnable(GL_TEXTURE_2D);
	unsigned int texture_id;
	glGenTextures(1, &texture_id);
	glBindTexture(GL_TEXTURE_2D, texture_id);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, BMPwidth, BMPheight, 0, GL_RGB, GL_UNSIGNED_BYTE, textureBMP);

	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	glBegin(GL_POLYGON);

	glTexCoord2f(0, 0);
	glVertex3f(0, 2 * LABEL_STRING_RADIUS + LABEL_TEXTURE_EPSILON, LABEL_HEIGHT);
	glTexCoord2f(1, 0);
	glVertex3f(0, 2 * LABEL_STRING_RADIUS + LABEL_TEXTURE_EPSILON, 0);
	glTexCoord2f(1, 1);
	glVertex3f(LABEL_LENGTH, 2 * LABEL_STRING_RADIUS + LABEL_TEXTURE_EPSILON, 0);
	glTexCoord2f(0, 1);
	glVertex3f(LABEL_LENGTH, 2 * LABEL_STRING_RADIUS + LABEL_TEXTURE_EPSILON, LABEL_HEIGHT);

	glEnd(); //Forgetting this line took me 2 hours to debug, stupid me

	glDisable(GL_TEXTURE_2D);
}

// From Actual Axle Center to Starting Point
Matrix<double, 3, 1> Unicycle::Diff(Matrix<double, 5, 1> t) {
	Matrix<double, 3, 1> s; s << 0, 0, 0;
	s(1) += (SADDLE_LENGTH - SADDLE_DIST);
	AngleAxisd saddle(-t(4), Vector3d(0, 0, 1));
	s = saddle.matrix() * s;
	AngleAxisd bend(-t(3), Vector3d(1, 0, 0));
	s = bend.matrix() * s;
	if (t(3) != 0) {
		s(1) += (TUBE_HEIGHT_UPPER + SEATPOST_HEIGHT) * (1 - cos(t(3))) / t(3);
		s(2) += (TUBE_HEIGHT_UPPER + SEATPOST_HEIGHT) * sin(t(3)) / t(3);
	}
	else {
		s(2) += (TUBE_HEIGHT_UPPER + SEATPOST_HEIGHT);
	}
	s(2) += (TUBE_HEIGHT_LOWER + TUBE_HEIGHT_MIDDLE - AXLE_RADIUS);
	AngleAxisd tube(-t(2), Vector3d(1, 0, 0));
	s = tube.matrix() * s;
	AngleAxisd axle(t(0), Vector3d(0, 0, 1));
	s = axle.matrix() * s;

	return s; // Can Vector3d be automatically converted to Matrix?
}

// Actual End Point. Formula := Starting Point - Diff + (Actual Axle Center to Actual End Point)
Matrix<double, 3, 1> Unicycle::End(Matrix<double, 5, 1> t, Matrix<double, 3, 1> s) {
	Matrix<double, 3, 1> E; E << 0, 0, 0;
	E(0) = -CRANK_DIST; E(1) = CRANK_LENGTH;
	AngleAxisd crank(-t(1), Vector3d(1, 0, 0));
	E = crank.matrix() * E;
	E(0) += -PEDAL_LENGTH/2;
	/*E(1) += (PEDAL_GAP / 2 + PEDAL_WIDTH);*/
	E(2) += (PEDAL_HEIGHT / 2);
	AngleAxisd axle(t(0), Vector3d(0, 0, 1)); //注意顺逆时针
	E = axle.matrix() * E;

	return s - Diff(t) + E;
}

void Unicycle::IK_Compute() {
	Matrix<double, 3, 1> xyz; xyz << VAL(XPOS), VAL(YPOS), VAL(ZPOS);
	Matrix<double, 3, 1> tar; tar << VAL(IK_TARGETX), VAL(IK_TARGETY), VAL(IK_TARGETZ);
	Matrix<double, 5, 1> c; // Means "IK_Current", but in rad.
	Matrix<double, 5, 5> d; // Take "a small step" on the angles when computing Jacobian
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 5; j++) {
			if (i == j) d(i,j) = 1.0/60;
			else d(i,j) = 0;
		}
	}
	double lr = 0.001; // How much should we change the end each time; similar to "learning rate" in deep learning
	int ceiling_reached = 0; // If performance does not increase for a long time, then we stop and claim that we cannot do better

	// Set the Initial Points
	for (int item = 0; item < NUMCONTROLS; item++) {
		if (!IK_related[item]) continue; // Do not change unrelated controls
		IK_init[item] = VAL(item);
	}
	IK_init[XPOS] = xyz(0); IK_init[YPOS] = xyz(1); IK_init[ZPOS] = xyz(2);

	c(0) = VAL(AXLE_DIR); c(1) = VAL(CRANK_DIR); c(2) = VAL(TUBE_DIR); c(3) = VAL(BEND); c(4) = VAL(SADDLE_DIR);
	c *= (PI / 180);
	Matrix<double, 3, 1> s; s = xyz + Diff(c); // Starting Point
	std::cout << (s).transpose() << std::endl << End(c, s).transpose() << std::endl;

	Matrix<double, 5, 1> prevc; prevc << 0, 0, 0, 0, 0;
	do {
		// Compute Jacobian
		Matrix<double, 3, 5> J; // Jacobian
		for (int i = 0; i < 5; i++) {
			J.col(i) = (End(c + d.col(i),s) - End(c,s)) / d(0);
		}

		//Compute Pseudoinverse
		Matrix<double, 5, 3> Jpi = J.transpose() * (J * J.transpose()).inverse();

		//Compute New "IK_Current" angles (A break condition within (if cannot improve anymore))
		Matrix<double, 3, 1> dE = tar - End(c,s);
		Matrix<double, 5, 1> dT = Jpi * dE; dT.normalize();
		c = c + lr * dT;
		std::cout << (tar - End(c, s)).norm() << " " << (tar - End(prevc, s)).norm() << std::endl;
		if (ceiling_reached >= 75) {
			std::cout << "Cannot Improve Anymore" << std::endl;
			break;
		}

		//Filter on "IK_Current" to satisfy the constrains
		if (c(0) < -PI) c(0) = -PI; if (c(0) > PI) c(0) = PI;//Axle
		if (c(1) < 0) c(1) += 2*PI; if (c(1) > 2*PI) c(1) -= 2*PI;//Crank
		if (c(2) < -PI/3) c(2) = -PI/3; if (c(2) > PI/3) c(2) = PI/3;//Tube
		if (c(3) < 0) c(3) = 0; if (c(3) > PI/2) c(3) = PI/2;//Bend
		if (c(4) < -PI/4) c(4) = PI/4; if (c(4) > PI/4) c(4) = PI/4;//Saddle

		if ((tar - End(c, s)).norm() - (tar - End(prevc, s)).norm() >= 0) ++ceiling_reached;
		prevc = c;

		//Determine breaking condition: Cannot improve anymore (written above) or close enough
		if ((tar - End(c, s)).norm() <= 1) {
			std::cout << "Close Enough" << std::endl;
			break;
		}
		
	} while (true);

	// Set the Target Angles
	int i = 0;
	for (int item = 0; item < NUMCONTROLS; item++) {
		if (!IK_related[item]) continue; // Do not change unrelated controls
		IK_target[item] = (int)round(c(i) / PI * 180);
		++i;
	}
	// Translate XYZPOS
	Matrix<double, 3, 1> newXYZ = s - Diff(c);
	IK_target[XPOS] = newXYZ(0); IK_target[YPOS] = newXYZ(1); IK_target[ZPOS] = newXYZ(2);
}

int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	// stepsize, defaultvalue)
	ModelerControl controls[NUMCONTROLS];
	controls[LEVEL_OF_DETAILS] = ModelerControl("Level of Details", 1, 5, 1, 4);
	controls[XPOS] = ModelerControl("X Position", -350, 350, 1, 0);
	controls[YPOS] = ModelerControl("Y Position", -350, 350, 1, 0);
	controls[ZPOS] = ModelerControl("Z Position", 0, 250, 1, 0);
	controls[AXLE_DIR] = ModelerControl("Direction of Axle", -180, 180, 1, 0);
	controls[WHEEL_DIR] = ModelerControl("Direction of Wheel", 0, 360, 1, 0);
	controls[WHEEL_QUALITY] = ModelerControl("Quality of Wheel", 0, 1, 1, 0);
	controls[CRANK_DIR] = ModelerControl("Direction of Crank", 0, 360, 1, 0);
	//controls[PEDAL_DIR] = ModelerControl("Direction of Pedal", -180, 180, 1, 0);
	controls[TUBE_DIR] = ModelerControl("Direction of Tube", -60, 60, 1, 0);
	controls[BEND] = ModelerControl("Bending Degree", 0, 90, 2, 0);
	controls[SADDLE_DIR] = ModelerControl("Direction of Saddle", -45, 45, 1, 0);
	controls[SADDLE_QUALITY] = ModelerControl("Quality of Saddle", 0, 2, 1, 0);
	controls[LABEL_DIR] = ModelerControl("Direction of Label", -180, 180, 1, 0);
	controls[HAPPINESS] = ModelerControl("Happiness of Character", -1, 1, 1, 0);
	controls[FRAME_ALL] = ModelerControl("See the whole character", 0, 1, 1, 0);
	controls[L_SYSTEM] = ModelerControl("L-System control", 0, 10, 1, 0);
	controls[META_BALL_CONTROL] = ModelerControl("metaball control:", 0, 1, 1, 0);
	controls[META_BALL1_RADIUS] = ModelerControl("metaball_1 radius:", 1.5, 3.5, 0.1, 3);
	controls[META_BALL2_RADIUS] = ModelerControl("metaball_2 radius:", 1.5, 3.5, 0.1, 3);
	controls[META_BALL3_RADIUS] = ModelerControl("metaball_3 radius:", 1.5, 3.5, 0.1, 3);
	controls[META_BALL4_RADIUS] = ModelerControl("metaball_4 radius:", 1.5, 3.5, 0.1, 3);
	controls[META_BALL_SCALE] = ModelerControl("metaball scale:", 0.7, 2, 0.01, 1);
	controls[IK_VISUALIZE] = ModelerControl("Visualize IK Chosen Points", 0, 1, 1, 1);
	controls[IK_TARGETX] = ModelerControl("IK Target XPOS of Pedal", -350, 350, 1, -50);
	controls[IK_TARGETY] = ModelerControl("IK Target YPOS of Pedal", -350, 350, 1, 75);
	controls[IK_TARGETZ] = ModelerControl("IK Target ZPOS of Pedal", 0, 250, 1, 50);
	controls[IK_MODE] = ModelerControl("Inverse Kinematics Mode", 0, 1, 1, 0);
	controls[IK_MOVE] = ModelerControl("Inverse Kinematics Motion %", 0, 100, 1, 0);
	controls[CR_TENSION] = ModelerControl("tension of catmull-rom", 0.01, 2, 0.01, 0.5);
	controls[EULER_PRECISION] = ModelerControl("Euler's Method Updating Precision", 1, 30, 1, 20);
	controls[RK] = ModelerControl("Runge-Kutta Method On?", 0, 1, 1, 0);
	controls[PS] = ModelerControl("Particle System Choice", 0, NUMPSS-1, 1, 4);
	controls[WIND] = ModelerControl("Wind Blowing Hair", 0, 2, 1, 0);
	controls[EDITCR] = ModelerControl("Inner Bezier Points On?", 0, 1, 1, 0);
	controls[FLOCKTARGET] = ModelerControl("Target Position of Flock", -180, 180, 1, 0);
	controls[FLOCKSPAWN] = ModelerControl("Spawn Particles into Flock", 0, 1, 1, 1);

	controls[CTRLPOINT_0] = ModelerControl("control point 0", 0, 2, 0.01, 0);
	controls[CTRLPOINT_1] = ModelerControl("control point 1", 0, 2, 0.01, 0);
	controls[CTRLPOINT_2] = ModelerControl("control point 2", 0, 2, 0.01, 0);
	controls[CTRLPOINT_3] = ModelerControl("control point 3", 0, 2, 0.01, 0);
	controls[CTRLPOINT_4] = ModelerControl("control point 4", 0, 2, 0.01, 0);
	controls[CTRLPOINT_5] = ModelerControl("control point 5", 0, 2, 0.01, 1);
	controls[CTRLPOINT_6] = ModelerControl("control point 6", 0, 2, 0.01, 1);
	controls[CTRLPOINT_7] = ModelerControl("control point 7", 0, 2, 0.01, 0);
	controls[CTRLPOINT_8] = ModelerControl("control point 8", 0, 2, 0.01, 0);
	controls[CTRLPOINT_9] = ModelerControl("control point 9", 0, 2, 0.01, 1);
	controls[CTRLPOINT_10] = ModelerControl("control point 10", 0, 2, 0.01, 1);
	controls[CTRLPOINT_11] = ModelerControl("control point 11", 0, 2, 0.01, 0);
	controls[CTRLPOINT_12] = ModelerControl("control point 12", 0, 2, 0.01, 0);
	controls[CTRLPOINT_13] = ModelerControl("control point 13", 0, 2, 0.01, 0);
	controls[CTRLPOINT_14] = ModelerControl("control point 14", 0, 2, 0.01, 0);
	controls[CTRLPOINT_15] = ModelerControl("control point 15", 0, 2, 0.01, 0);


	ModelerApplication::Instance()->SetParticleSystem(new FlockParticleSystem);
	ModelerApplication::Instance()->SetParticleSystem(new HairParticleSystem);
	ModelerApplication::Instance()->SetParticleSystem(new ClothParticleSystem);
	ModelerApplication::Instance()->SetParticleSystem(new SpringParticleSystem);
	ModelerApplication::Instance()->SetParticleSystem(new ParticleSystem);
	ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
	return ModelerApplication::Instance()->Run();
}