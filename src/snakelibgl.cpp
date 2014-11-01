/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  snakelibgl.cpp                                                                       |
|                                                                                       |
|  Current Author: Stephen Tully                                                        |
|  Description: Main file for running the medical snake visualization environment.      |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
|  Previous authors: Aaron Hoy (CMU), Cornell Wright (Cardiorobotics)                   |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#include <gtk/gtk.h>
#include "../probe_control/gui_config.h"
#include "../probe_control/gui_interface.h"
#include "../probe_control/gui_support.h"
#include "trackball.h"
#include "snakelibgl_gui.h"
#include <stdio.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <math.h>
#include <vector>
#include <time.h>
#include <windows.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include "registration.h"
#include "../probe_control/config_defaults.h"
#include "ATC3DGM.h"
#include "polygonMesh.h"
#include "ascension.h"
#include "registration.h"
#include "../probe_control/guided_motions.h"
#include "../probe_control/motor_abs.h"
#include "snakeEstimation.h"
#include "autonomy.h"
#include "snakelibgl.h"
#include <iostream>
#include <fstream>

using namespace std;

#define RANGE_USAGE 0.5

GtkWidget *snakelibgl_window;

snakeEstimation *mySnakeEstimation;				// instance of the snakeEstimation class for estimating snake shape
ascensionTracker *myAscensionTracker;			// instance of the ascensionTracker class for easy access to the tracker pose
registration *myRegistration;                   // instance of the registration class to perform registration between two point sets
autonomy *myAutonomy;							// instance of the autonomy class to draw a user defined path and control the robot autonomously
polygonMesh **myPolygonMeshes;                  // array of instances of the polygonMesh class to parse, store, and draw polygonMeshes
int numPolygonMeshes;							// number of polygonMeshes that we have loaded from file for display

static float eyex, eyey, eyez;					// x,y,z values for gluLookAt (location of eye)
static float focusx, focusy, focusz;			// x,y,z values for gluLookAt (location of focus)
bool mouseLeftButton = false;					// flag to say whether the left mouse button is depressed
bool mouseRightButton = false;					// flag to say whether the right mouse button is depressed
bool mouseMiddleButton = false;					// flag to say whether the middle mouse button is depressed
int mousex = 0, mousey = 0;						// the stored location of the mouse
int view_width = 640, view_height = 480;		// the view width and height for the opengl window
float curr_quaternion[4];						// the quaternion defining the rotation for the modelview

float phi = 0.0;
float theta = 0.0;

int prevGuidedMode = -1;
int currGuidedMode = -1;

FILE *snakeLibLog = NULL;

float prevCableLength[3];						// stored cable lengths for the three snake cables

// TODO: find for loops that can be replaced with gsl_matrix_memcpy
// TODO: try to draw things in different order to get transparency working better
// TODO: log more stuff, including constants when initializing
// TODO: maybe store snake estimation constants
// TODO: make sure to log timestamps in OTHER logs
// TODO: finish logging distal kalm
// TODO: finish autonomy logging
// TODO: fix advancing/retracting
// TODO: double and triple check logging
// TODO: tune covariances for estimation
// TODO: make it so ascension logging buttons don't work when clicked and no tracker present
// TODO: maybe save pictures in more than one format
// TODO: fix enable autonomy button when not initialized and stuff

/******************************************************************************\
*                                                                              *
*  SnakeLibGL::closeUpShop()                                                   *
*                                                                              *
*  This function is run when the program ends.                                 *
*                                                                              *
\******************************************************************************/
void SnakeLibGL::closeUpShop(void) {
	if (snakeLibLog) fclose(snakeLibLog);
	if (mySnakeEstimation) delete(mySnakeEstimation);
	if (myRegistration) delete(myRegistration);
	if (myAscensionTracker) delete(myAscensionTracker);
	if (myAutonomy) delete(myAutonomy);
	if (myPolygonMeshes) {
		for (int i = 0; i < numPolygonMeshes; i++)
			if (myPolygonMeshes[i]) delete(myPolygonMeshes[i]);
		free(myPolygonMeshes);
	}
}

/******************************************************************************\
*                                                                              *
*  SnakeLibGL::captureScreenshot()                                             *
*                                                                              *
*  This function takes a screen shot of the opengl window.                     *
*                                                                              *
\******************************************************************************/
bool SnakeLibGL::captureScreenshot(char *filename) {

	if (!filename) {
		printf("\tError: screenshot, empty filename.\n\n");
		return false;
	} else if (strlen(filename) <= 4) {
		printf("\tError: screenshot, improper filename given.\n\n");
		return false;
	} else if (!strcmp(&filename[strlen(filename)-4], ".ppm")) {

		// read the pixels into a buffer
		glPixelStorei(GL_PACK_ALIGNMENT, 1);
		GLubyte *pixels = new GLubyte[3*view_width*view_height];
		if (!pixels) return false;
		glReadPixels(0, 0, view_width, view_height, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid *)pixels);

		// write the pixel data to a file
		fstream imagefile;
		imagefile.open(filename, ios::out | ios::trunc | ios::binary);
		char sizeString[MAX_TEXT_LENGTH];
		sprintf_s(sizeString, MAX_TEXT_LENGTH, "%d %d 255", view_width, view_height);
		imagefile << "P6" << endl << sizeString << endl;
		for (int r = view_height-1; r >= 0; r--) {
			for (int c = 0; c < view_width; c++) {
				for (int i = 0; i < 3; i++) {
					imagefile << (char)pixels[3*view_width*r+3*c+i];
				}
			}
		}
		imagefile.close();

		// free the memory allocated for the pixels
		delete pixels;

	} else {
		printf("\tError: screenshot, must be ppm filename.\n\n");
		return false;
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  HandleMouseState()                                                          *
*                                                                              *
*  This function handles the change of state when a mouse buttong is changed.  *
*                                                                              *
\******************************************************************************/
void HandleMouseState(int button, int state, int x, int y) {
	// update our button state
	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) mouseLeftButton = true;
		else if (state == GLUT_UP) {
			mouseLeftButton = false;
			if (myAutonomy->getPickingPointsFlag()) {
				double nearx = 0, neary = 0, nearz = 0;
				double farx = 0, fary = 0, farz = 0;

				glPushMatrix();

				// move our eye to the right place
				gluLookAt(eyex, eyey, eyez, focusx, focusy, focusz, 1, 0, 0);

				// set the modelview
				GLfloat m[4][4];
				build_rotmatrix(m, curr_quaternion);
				glMultMatrixf(&m[0][0]);

				GLdouble model_matrix[16];
				glGetDoublev(GL_MODELVIEW_MATRIX, model_matrix);
				GLdouble proj_matrix[16];
				glGetDoublev(GL_PROJECTION_MATRIX, proj_matrix);
				GLint viewport[4] = {0, 0, view_width, view_height};
				glPopMatrix();

				gluUnProject(x, view_height-y, 0.0, model_matrix, proj_matrix, viewport, &nearx, &neary, &nearz);
				gluUnProject(x, view_height-y, 1.0, model_matrix, proj_matrix, viewport, &farx, &fary, &farz);
				myAutonomy->addControlRay(nearx, neary, nearz, farx, fary, farz, numPolygonMeshes, myPolygonMeshes);
				myAutonomy->updatePreCurvePathPoints(numPolygonMeshes, myPolygonMeshes);
				myAutonomy->computePath();
			}
		}
		else mouseLeftButton = false;
	} else if (button == GLUT_RIGHT_BUTTON) {
		if (state == GLUT_DOWN) mouseRightButton = true;
		else mouseRightButton = false;
	} else if (button == GLUT_MIDDLE_BUTTON) {
		if (state == GLUT_DOWN) mouseMiddleButton = true;
		else mouseMiddleButton = false;
	} else if (button == 3) { // wheel up, zooming in
		if (!myAutonomy->getPickingPointsFlag()) {
			eyex = (1 - 0.05f) * eyex + focusx * 0.05f;
			eyey = (1 - 0.05f) * eyey + focusy * 0.05f;
			eyez = (1 - 0.05f) * eyez + focusz * 0.05f;
		}
	} else if (button == 4) { // wheel down
		if (!myAutonomy->getPickingPointsFlag()) {
			eyex = (1 + 0.05f) * eyex - focusx * 0.05f;
			eyey = (1 + 0.05f) * eyey - focusy * 0.05f;
			eyez = (1 + 0.05f) * eyez - focusz * 0.05f;
		}
	}
	mousex = x;		// update the x position of the mouse
	mousey = y;		// update the y position of the mouse
}

/******************************************************************************\
*                                                                              *
*  HandleMouseMove()                                                           *
*                                                                              *
*  This function draws handles the rotation and translation change when we     *
*     move the mouse while a button is depressed.                              *
*                                                                              *
\******************************************************************************/
void HandleMouseMove(int x, int y)
{
	float change_quaternion[4];
	if (mouseLeftButton && !mouseRightButton && !mouseMiddleButton) {
		if (!myAutonomy->getPickingPointsFlag()) {
			// compute the quaternion rotation needed to obey the virtual trackball
			trackball(change_quaternion, (view_height-2.0f*mousey)/view_height, -(2.0f*mousex-view_width)/view_width,
					  (view_height-2.0f*y)/view_height,-(2.0f*x-view_width)/view_width);
			add_quats(change_quaternion, curr_quaternion, curr_quaternion);
		}
	} else if (!mouseLeftButton && mouseRightButton && !mouseMiddleButton) {
		if (!myAutonomy->getPickingPointsFlag()) {
			eyex -= (mousey - y)*0.1f;			// move our eye x position
			eyey -= (mousex - x)*0.1f;			// move our eye y position
			focusx -= (mousey - y)*0.1f;		// move our focus x position
			focusy -= (mousex - x)*0.1f;		// move out focus y position
		}
	}
	mousex = x;		// update the x position of the mouse
	mousey = y;		// update the y position of the mouse
}

/******************************************************************************\
*                                                                              *
*  display()                                                                   *
*                                                                              *
*  This function draws one link of the snake robot in the glut window,         *
*     according to the transformation matrix.                                  *
*                                                                              *
\******************************************************************************/
void display(void)
{
	// say that we're going to change the modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	// move our eye to the right place
	gluLookAt(eyex, eyey, eyez, focusx, focusy, focusz, 1, 0, 0);

	// set the modelview
	GLfloat m[4][4];
	build_rotmatrix(m, curr_quaternion);
	glMultMatrixf(&m[0][0]);

	// draw the text for describing the robot state
	char *string = NULL;
	if (guided_paused)
		string = "Paused";
	else if (guided_mode == STEPPED_STEERING)
		string = "Steering";
	else if (guided_mode == STEPPED_ADVANCE_INNER || guided_mode == STEPPED_ADVANCE_OUTER)
		string = "Advancing";
	else if (guided_mode == RETRACT_INNER || guided_mode == RETRACT_OUTER)
		string = "Retracting";
	else if (guided_mode == HOMING_SCREWS)
		string = "Homing";
	
	if (string) {
		glBlendFunc(GL_SRC_ALPHA,GL_ONE);
		glPushMatrix();
		glColor3f(1.0f, 1.0f, 1.0f);
		glLineWidth(2.0f);
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				float x = 22.0f + ((float)i)*0.1f;
				for (char *c = string; *c != '\0'; c++) {
					glLoadIdentity();
					glTranslatef(x, 28.5f + ((float)j)*0.1f, -100.0f);
					float scaleFactor = 0.03f;
					glScalef(scaleFactor, scaleFactor, scaleFactor);
					glutStrokeCharacter(GLUT_STROKE_ROMAN, *c);
					x += 1.25f*(float)glutStrokeWidth(GLUT_STROKE_ROMAN, *c)*scaleFactor;
				}
			}
		}
		glPopMatrix();
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	}

	// draw the autonomous path
	glDisable(GL_LINE_SMOOTH);
	if (myAutonomy->getUpdatePreCurveDrawFlag()) myAutonomy->drawPreCurvePathPoints(numPolygonMeshes+1);
	if (myAutonomy->getVisiblePreCurvePathPoints()) glCallList(numPolygonMeshes+1);
	if (myAutonomy->getUpdateCurveDrawFlag()) myAutonomy->drawCurvePathPoints(numPolygonMeshes+2);
	if (myAutonomy->getVisibleSplineCurve()) glCallList(numPolygonMeshes+2);
	glEnable(GL_LINE_SMOOTH);

	// draw the control look ahead point
	if (myAutonomy->getAutonomousDrivingFlag()) myAutonomy->drawControlLookAheadPoint();

	// draw the tracker measurement
	gsl_matrix *T_tracker2model = gsl_matrix_alloc(4,4);
	for (int i = 0; i < myAscensionTracker->getNumSensors(); i++) {
		if (myAscensionTracker->isSensorAttached(i)) {
			if (myRegistration->get_T_tracker2model(T_tracker2model)) {
				myAscensionTracker->drawTracker(T_tracker2model, i);
			} else {
				myAscensionTracker->drawTracker(NULL, i);
			}
		}
	}

	// draw the snake estimation stuff
	if (myRegistration->get_T_tracker2model(T_tracker2model)) {
		mySnakeEstimation->drawDistalKalm(T_tracker2model);
		mySnakeEstimation->drawFullDead(T_tracker2model);
		mySnakeEstimation->drawFullKalm(T_tracker2model);
		mySnakeEstimation->drawFullIterated(T_tracker2model);
	} else {
		mySnakeEstimation->drawDistalKalm(NULL);
		mySnakeEstimation->drawFullDead(NULL);
		mySnakeEstimation->drawFullKalm(NULL);
		mySnakeEstimation->drawFullIterated(NULL);
	}

	// draw the sensor trails
	if (myRegistration->get_T_tracker2model(T_tracker2model)) {
		myAscensionTracker->drawTrails(T_tracker2model);
	} else {
		myAscensionTracker->drawTrails(NULL);
	}
	if (T_tracker2model) gsl_matrix_free(T_tracker2model);

	// draw the polygon meshes
	for (int i = 0; i < numPolygonMeshes; i++) {
		if (myPolygonMeshes[i]->getUpdateDrawFlag()) myPolygonMeshes[i]->drawPolygonMesh(i+1);
		if (myPolygonMeshes[i]->getVisibleFlag()) {
			glCallList(i+1);
		}
	}

	glPopMatrix();
	glutSwapBuffers();
}

/******************************************************************************\
*                                                                              *
*  reshape()                                                                   *
*                                                                              *
*  This function draws one link of the snake robot in the glut window,         *
*     according to the transformation matrix.                                  *
*                                                                              *
\******************************************************************************/
void reshape(int width, int height) {
	if (width <= 0) view_width = 4;
	else view_width = width;
	if (height <= 0) view_height = 4;
	else view_height = height;
	while (view_width  % 4 != 0) view_width--;
	while (view_height % 4 != 0) view_height--;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40.0f, (float)view_width/(float)view_height, 0.1f, 4000.0f);
	glViewport(0, 0, view_width, view_height);
	display();
}

/******************************************************************************\
*                                                                              *
*  idle()                                                                      *
*                                                                              *
*  This function draws one link of the snake robot in the glut window,         *
*     according to the transformation matrix.                                  *
*                                                                              *
\******************************************************************************/
void idle(void) {		
	glutPostRedisplay();
}

/******************************************************************************\
*                                                                              *
*  SnakeLibGL:init()                                                           *
*                                                                              *
*  This function initializes the visualization software.                       *
*                                                                              *
\******************************************************************************/
void SnakeLibGL::init(int argc, char* argv[]) {

	// initialize my extra classes
	numPolygonMeshes = 0;
	mySnakeEstimation = new snakeEstimation;
	myRegistration = new registration;
	myPolygonMeshes = NULL;
	glGenLists(MAX_GL_LISTS);
	myAutonomy = new autonomy;

	// initialize the ascension tracker
	myAscensionTracker = new ascensionTracker;

	// open up the log file
	startNewLogFile("_defaultVisualizationLog.log");
	/*	
	fopen_s(&snakeLibLog, "defaultVisualizationLog.log", "w");
	if (!snakeLibLog) {
		printf("\tError: snakelib, could not open snake estimation log for writing.\n\n");
	} else {

		// TODO: log more here
		fprintf_s(snakeLibLog, "0 SNAKELIB_CABLE_RADIUS %lf\n", SNAKELIB_CABLE_RADIUS);
	}
	*/

	// initialize the gui
	snakelibgl_window = create_snakelibgl_window ();
	gtk_widget_show(snakelibgl_window);
	g_signal_connect ((gpointer) snakelibgl_window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

	// initialize the display and glut functions
	glutInit(&argc, argv);
  	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowSize(view_width,view_height);
	glutCreateWindow("Snake Visualization");
	glutDisplayFunc(display);
	glutMouseFunc(HandleMouseState);
	glutMotionFunc(HandleMouseMove);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);

	// initialize some opengl settings
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_RESCALE_NORMAL);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40.0f, (float)view_width/(float)view_height, 200.0f, 400.0f);
	glViewport(0, 0, view_width, view_height);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glDepthFunc(GL_LEQUAL);

	// initialize variables for controlling the view
	trackball(curr_quaternion, 0.0, 0.0, 0.0, 0.0);
	mouseLeftButton = mouseRightButton = mouseMiddleButton = false;
	mousex = mousey = 0;
	eyex = 0; eyey = 0; eyez = 400;
	focusx = focusy = focusz = 0.0f;

	// create some lighting for the view
	glMatrixMode(GL_MODELVIEW);
	static const GLfloat ambient_light_levels[]= {(GLfloat)0.3, (GLfloat)0.3, (GLfloat)0.3, (GLfloat)1.0};
	static const GLfloat diffuse_light_levels[]= {(GLfloat)0.5, (GLfloat)0.5, (GLfloat)0.5, (GLfloat)1.0};
	static const GLfloat specular_light_levels[]= {(GLfloat)0.7, (GLfloat)0.7, (GLfloat)0.7, (GLfloat)1.0};
	static const GLfloat light_coords[]= {(GLfloat)0.0, (GLfloat)0.0, (GLfloat)900.0, (GLfloat)1.0};
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_light_levels);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light_levels);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular_light_levels);
	glLightfv(GL_LIGHT0, GL_POSITION, light_coords);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	// initialize the cable length for the snake robot
	prevCableLength[0] = prevCableLength[1] = prevCableLength[2] = 0.0;
}

/******************************************************************************\
*                                                                              *
*  logging()                                                                   *
*                                                                              *
*  This function handles the data logging for everything.                      *
*                                                                              *
\******************************************************************************/
bool logging(double timeStamp) {

	if (snakeLibLog) {
		
		// log snakelibgl stuff
		int dataLabel = 1;
		fprintf_s(snakeLibLog, "%d %lf guidedMode %d guidedPaused %d prevGuidedMode %d currGuidedMode %d\n", dataLabel++, timeStamp, guided_mode, guided_paused, prevGuidedMode, currGuidedMode);
		fprintf_s(snakeLibLog, "%d %lf prevCableLength0 %f prevCableLength1 %f prevCableLength2 %f currCableLength0 %f currCableLength1 %f currCableLength2 %f\n", dataLabel++, timeStamp, prevCableLength[0], prevCableLength[1], prevCableLength[2], get_position(CABLE_ONE), get_position(CABLE_TWO), get_position(CABLE_THREE));
		fprintf_s(snakeLibLog, "%d %lf phi %f theta %f\n", dataLabel++, timeStamp, phi, theta);

		// log the other classes
		myAscensionTracker->logData(snakeLibLog, timeStamp, dataLabel++);
		myRegistration->logData(snakeLibLog, timeStamp, dataLabel++);
		mySnakeEstimation->logData(snakeLibLog, timeStamp, dataLabel++);
		myAutonomy->logData(snakeLibLog, timeStamp, dataLabel++);

		// flush the log file
		fflush(snakeLibLog);

	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  SnakeLibGL:startNewLogFile()                                                *
*                                                                              *
*  This function starts a fresh log file.                                      *
*                                                                              *
\******************************************************************************/
void SnakeLibGL::startNewLogFile(char *filename) {
	if (snakeLibLog) {
		fflush(snakeLibLog);
		fclose(snakeLibLog);
	}
	fopen_s(&snakeLibLog, filename, "w");
	if (!snakeLibLog) {
		printf("\tError: snakelib, could not open snake estimation log for writing.\n\n");
	} else {
		fprintf_s(snakeLibLog, "0 SNAKE_LINK_LENGTH %lf SNAKELIB_LINK_RADIUS %lf SNAKELIB_CABLE_RADIUS %lf RANGE_USAGE %lf\n", (double)SNAKELIB_LINK_LENGTH, (double)SNAKELIB_LINK_RADIUS, (double)SNAKELIB_CABLE_RADIUS, (double)RANGE_USAGE);
	}
	return;
}

/******************************************************************************\
*                                                                              *
*  steering()                                                                  *
*                                                                              *
*  This function handles the steering for the robot.                           *
*                                                                              *
\******************************************************************************/
bool steering() {

	float cable_diff[3];

	cable_diff[0] = get_position(CABLE_ONE) - prevCableLength[0];
	cable_diff[1] = get_position(CABLE_TWO) - prevCableLength[1];
	cable_diff[2] = get_position(CABLE_THREE) - prevCableLength[2];
	double c1 = cable_diff[0]-(cable_diff[0]+cable_diff[1]+cable_diff[2])/3;
    double c2 = cable_diff[1]-(cable_diff[0]+cable_diff[1]+cable_diff[2])/3;
    double c3 = cable_diff[2]-(cable_diff[0]+cable_diff[1]+cable_diff[2])/3;

    if (c1 > SNAKELIB_CABLE_RADIUS*RANGE_USAGE) c1 = SNAKELIB_CABLE_RADIUS*RANGE_USAGE;
    else if (c1 < -SNAKELIB_CABLE_RADIUS*RANGE_USAGE) c1 = -SNAKELIB_CABLE_RADIUS*RANGE_USAGE;
    if (c2 > SNAKELIB_CABLE_RADIUS*RANGE_USAGE) c2 = SNAKELIB_CABLE_RADIUS*RANGE_USAGE;
    else if (c2 < -SNAKELIB_CABLE_RADIUS*RANGE_USAGE) c2 = -SNAKELIB_CABLE_RADIUS*RANGE_USAGE;
    if (c3 > SNAKELIB_CABLE_RADIUS*RANGE_USAGE) c3 = SNAKELIB_CABLE_RADIUS*RANGE_USAGE;
    else if (c3 < -SNAKELIB_CABLE_RADIUS*RANGE_USAGE) c3 = -SNAKELIB_CABLE_RADIUS*RANGE_USAGE;          

    theta = (float)atan2((float)sqrt(3.0f)*(2.0f*c2+c1), 3.0f*c1);
    phi = (float)asin(-c1/(SNAKELIB_CABLE_RADIUS*cos(theta)));
    if (phi < 0) phi = -phi;

	return true;
}

/******************************************************************************\
*                                                                              *
*  SnakeLibGL:main_loop()                                                      *
*                                                                              *
*  This function is the main loop for the visualization software.              *
*                                                                              *
\******************************************************************************/
bool SnakeLibGL::main_loop(void) {

	// grab a time stamp
	long long currentTime;
	long long frequency;
	QueryPerformanceCounter((_LARGE_INTEGER*)&currentTime);
	QueryPerformanceFrequency((_LARGE_INTEGER*)&frequency);
	double timeStamp = (double)currentTime/(double)frequency;

	// grab a new ascension tracker measurement
	myAscensionTracker->updatePoseMatrix(timeStamp);

	// update the guided modes
	prevGuidedMode = currGuidedMode;
	currGuidedMode = guided_mode;

	switch(currGuidedMode) {
		case STEPPED_STEERING:
			if (currGuidedMode == STEPPED_STEERING && prevGuidedMode == STEPPED_ADVANCE_OUTER) {

				//mySnakeEstimation->advanceFullKalm(phi, theta);
				//mySnakeEstimation->advanceFullDead(phi, theta);
				//mySnakeEstimation->advanceFullIterated(phi, theta);
				//mySnakeEstimation->advanceDistalKalm(phi, theta);
				mySnakeEstimation->advanceFullKalm(0.0, 0.0);
				mySnakeEstimation->advanceFullDead(0.0, 0.0);
				mySnakeEstimation->advanceFullIterated(0.0, 0.0);
				mySnakeEstimation->advanceDistalKalm(0.0, 0.0);

			}
			/*
			TODO: Fix this.
			if (currGuidedMode == RETRACT_STEERING && prevGuidedMode == RETRACT_INNER) {
				mySnakeEstimation->retractFullKalm();
				mySnakeEstimation->retractFullDead();
				mySnakeEstimation->retractFullIterated();
				mySnakeEstimation->retractDistalKalm();
			}
			*/

			steering();

			// initialize the estimation stuff if necessary
			if (!mySnakeEstimation->isInitializedSnakeEstimation()) {
				for (int i = 0; i < myAscensionTracker->getNumSensors(); i++) {
					if (myAscensionTracker->isSensorAttached(i)) {
						gsl_matrix *Ttracker = gsl_matrix_alloc(4,4);
						if (myAscensionTracker->getPoseMatrix(Ttracker, i)) {
							mySnakeEstimation->initializeDistalKalm(Ttracker);
							mySnakeEstimation->initializeFullKalm(Ttracker);
							mySnakeEstimation->initializeFullIterated(Ttracker);
							mySnakeEstimation->initializeFullDead(Ttracker);
						}
						if (Ttracker) gsl_matrix_free(Ttracker);
						break;
					}
				}

			} else {

				// update the estimates
				mySnakeEstimation->steerPredictDistalKalm(phi, theta);
				mySnakeEstimation->steerPredictFullKalm(phi, theta);
				mySnakeEstimation->steerPredictFullIterated(phi, theta);
				mySnakeEstimation->steerPredictFullDead(phi, theta);
				for (int i = 0; i < myAscensionTracker->getNumSensors(); i++) {
					if (myAscensionTracker->isSensorAttached(i)) {
						gsl_matrix *Ttracker = gsl_matrix_alloc(4,4);
						if (myAscensionTracker->getPoseMatrix(Ttracker, i)) {
							mySnakeEstimation->steerCorrectDistalKalm(Ttracker);
							mySnakeEstimation->steerCorrectFullKalm(Ttracker);
							mySnakeEstimation->steerCorrectFullIterated(Ttracker);
						}
						if (Ttracker) gsl_matrix_free(Ttracker);
						break;
					}
				}

				// compute autonomous stuff
				if (myAutonomy->getAutonomousDrivingFlag()) {
					myAutonomy->computeControlLookAheadPoint();
					myAutonomy->computeAutonomousJoystickValues();
				}
			}
			break;
		case STEPPED_ADVANCE_INNER:
			if (myAscensionTracker->getSensorTrailsTriggerOnlyFlag() && !guided_paused && currGuidedMode != prevGuidedMode && myAscensionTracker->getTrailCapturingFlag()) myAscensionTracker->addToTrail(true);
			prevCableLength[0] = get_position(CABLE_ONE); prevCableLength[1] = get_position(CABLE_TWO); prevCableLength[2] = get_position(CABLE_THREE);
			break;
		case STEPPED_ADVANCE_OUTER:
			if (prevGuidedMode == RETRACT_INNER) {
				mySnakeEstimation->retractFullKalm();
				mySnakeEstimation->retractFullDead();
				mySnakeEstimation->retractFullIterated();
				mySnakeEstimation->retractDistalKalm();
			}
			if (myAscensionTracker->getSensorTrailsTriggerOnlyFlag() && !guided_paused && currGuidedMode != prevGuidedMode && myAscensionTracker->getTrailCapturingFlag()) myAscensionTracker->addToTrail(true);
			prevCableLength[0] = get_position(CABLE_ONE); prevCableLength[1] = get_position(CABLE_TWO); prevCableLength[2] = get_position(CABLE_THREE);			
			break;
		case RETRACT_INNER:
			if (myAscensionTracker->getSensorTrailsTriggerOnlyFlag() && !guided_paused && currGuidedMode != prevGuidedMode && myAscensionTracker->getTrailCapturingFlag()) myAscensionTracker->addToTrail(false);
			prevCableLength[0] = get_position(CABLE_ONE); prevCableLength[1] = get_position(CABLE_TWO); prevCableLength[2] = get_position(CABLE_THREE);
			break;
		case RETRACT_OUTER:
			if (myAscensionTracker->getSensorTrailsTriggerOnlyFlag() && !guided_paused && currGuidedMode != prevGuidedMode && myAscensionTracker->getTrailCapturingFlag()) myAscensionTracker->addToTrail(false);
			prevCableLength[0] = get_position(CABLE_ONE); prevCableLength[1] = get_position(CABLE_TWO); prevCableLength[2] = get_position(CABLE_THREE);
			break;
		case HOMING_SCREWS:
			mySnakeEstimation->destroyDistalKalm(); mySnakeEstimation->destroyFullKalm(); mySnakeEstimation->destroyFullDead();
			prevCableLength[0] = get_position(CABLE_ONE); prevCableLength[1] = get_position(CABLE_TWO); prevCableLength[2] = get_position(CABLE_THREE);
			break;
		default:
			break;
	}

	if (!myAscensionTracker->getSensorTrailsTriggerOnlyFlag() && myAscensionTracker->getTrailCapturingFlag()) {
		myAscensionTracker->addToTrail(true);	
	}

	// log everything here
	logging(timeStamp);

	// update gtk
	while (gtk_events_pending()) gtk_main_iteration();

	// display everything
	display();

	return true;
}

