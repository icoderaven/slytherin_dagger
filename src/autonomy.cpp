/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  autonomy.cpp                                                                         |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions dealing with autonomous driving of the snake robot.           |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#include <windows.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <math.h>
#include "ascension.h"
#include "registration.h"
#include "snakeEstimation.h"
#include "polygonMesh.h"
#include "../probe_control/config_defaults.h"
#include "snakelibgl.h"
#include "autonomy.h"

#define CONTROL_LOOK_AHEAD_DIST 2.0

extern snakeEstimation *mySnakeEstimation;
extern registration *myRegistration;

/******************************************************************************\
*                                                                              *
*  autonomy::logData()                                                         *
*                                                                              *
*  This function logs the important data.                                      *
*                                                                              *
\******************************************************************************/
bool autonomy::logData(FILE *fptr, double timeStamp, int dataLabel) {
	if (loggingFlag) {
		fprintf_s(fptr, "%d %lf autonomy autonomousDrivingFlag %d numControlPoints %d", dataLabel, timeStamp, (int)autonomousDrivingFlag, numControlPoints);
		for (int i = 0; i < numControlPoints; i++) {
			fprintf_s(fptr, " controlPointIndex %d controlRayNearPoint %lf %lf %lf", i, controlRayNearPoints[i][0], controlRayNearPoints[i][1], controlRayNearPoints[i][2]);
			fprintf_s(fptr, " controlRayFarPoint %lf %lf %lf", controlRayFarPoints[i][0], controlRayFarPoints[i][1], controlRayFarPoints[i][2]);
			fprintf_s(fptr, " controlSurfacePoint %lf %lf %lf", controlSurfacePoints[i][0], controlSurfacePoints[i][1], controlSurfacePoints[i][2]);
			fprintf_s(fptr, " controlSurfacePointPushedOut %lf %lf %lf", controlSurfacePointsPushedOut[i][0], controlSurfacePointsPushedOut[i][1], controlSurfacePointsPushedOut[i][2]);
		}
		fprintf_s(fptr, " SPLINE_DIMENSION %d NUM_SPLINE_PARAMETERS %d numSplineSegments %d", (int)SPLINE_DIMENSION, (int)NUM_SPLINE_PARAMETERS, numSplineSegments);
		for (int i = 0; i < numSplineSegments; i++) {
			fprintf_s(fptr, " splineSegmentIndex %d", i);
			if (!splineParametersForEachSegment[i]) {
				fprintf_s(fptr, " NULL");			
			} else {
				fprintf_s(fptr, " splineParameters");
				for (int j = 0; j < 2*NUM_SPLINE_PARAMETERS; j++) {
					fprintf_s(fptr, " %lf", splineParametersForEachSegment[i][j]);
				}
			}
		}
		if (controlLookAheadPoint) {
			fprintf_s(fptr, " controlLookAheadPoint %lf %lf %lf", controlLookAheadPoint[0], controlLookAheadPoint[1], controlLookAheadPoint[2]);
		}
		fprintf_s(fptr, "\n");
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::setLoggingFlag()                                                  *
*                                                                              *
*  This function sets the logging flag.                                        *
*                                                                              *
\******************************************************************************/
void autonomy::setLoggingFlag(bool val) {
	loggingFlag = val;
}

/******************************************************************************\
*                                                                              *
*  autonomy::computeAutonomousJoystickValues()                                 *
*                                                                              *
*  This function computes the X and Y joystick commands for autonomy.          *
*    NOTE: this needs X_distal_kalm in snakeEstimation in order for this       *
*    to work properly.                                                         *
*                                                                              *
\******************************************************************************/
bool autonomy::computeAutonomousJoystickValues(void) {

	// safety check that everything's okay
	if (!autonomousDrivingFlag) {
		autonomousJoystickX = 0.0;
		autonomousJoystickY = 0.0;
		printf("\tError: autonomy, should not determine joystick values if not in autonomous mode.\n\n");		
		return false;
	} else if (!controlLookAheadPoint) {
		autonomousJoystickX = 0.0;
		autonomousJoystickY = 0.0;
		printf("\tError: autonomy, can't compute joystick values if look ahead when point not allocated.\n\n");
		return false;
	}

	// allocate matrices for computing the proper joystick values
	gsl_matrix *T = gsl_matrix_calloc(4,4);
	gsl_matrix *Tpose = gsl_matrix_calloc(4,4);
	gsl_matrix *iTpose = gsl_matrix_calloc(4,4);
	gsl_permutation *p = gsl_permutation_alloc(4);
	gsl_matrix *lookAhead = gsl_matrix_alloc(4,1);
	gsl_matrix *lookAheadTransformed = gsl_matrix_alloc(4,1);

	// get the 6-DOF estimate of the distal link of the snake
	if (!mySnakeEstimation->getDistalTransformationMatrix(Tpose)) {
		if (T) gsl_matrix_free(T);
		if (Tpose) gsl_matrix_free(Tpose);
		printf("\tError: autonomy, couldn't grab distal transformation matrix from snakeEstimation.\n\n");
		return false;
	}

	// get the transformation matrix for registration and then transform Tpose
	if (myRegistration->get_T_tracker2model(T)) {
		gsl_matrix *Ttemp = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, T, Tpose, 0.0, Ttemp);
		gsl_matrix_free(Tpose);
		Tpose = Ttemp;
	}

	// invert the transformation to get iTpose
	int s = 0;
	gsl_linalg_LU_decomp(Tpose, p, &s);
	gsl_linalg_LU_invert(Tpose, p, iTpose);

	// apply iTpose to the look ahead point
	for (int i = 0; i < 3; i++) gsl_matrix_set(lookAhead, i, 0, controlLookAheadPoint[i]);
	gsl_matrix_set(lookAhead, 3, 0, 1.0);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, iTpose, lookAhead, 0.0, lookAheadTransformed);

	// compute the angular errors
	double rz = atan2(gsl_matrix_get(lookAheadTransformed, 1, 0),gsl_matrix_get(lookAheadTransformed, 0, 0));
	while (rz < -M_PI) rz += 2*M_PI;
	while (rz >  M_PI) rz -= 2*M_PI;
	double ry = atan2(gsl_matrix_get(lookAheadTransformed, 2, 0), sqrt(gsl_matrix_get(lookAheadTransformed, 0, 0)*gsl_matrix_get(lookAheadTransformed, 0, 0) + gsl_matrix_get(lookAheadTransformed, 1, 0)*gsl_matrix_get(lookAheadTransformed, 1, 0)));
	while (ry < -M_PI) ry += 2*M_PI;
	while (ry >  M_PI) ry -= 2*M_PI;

	// compute a correction to the joystick to steer towards the look ahead point
	autonomousJoystickX += -0.03f*(float)rz;
	autonomousJoystickY +=  0.03f*(float)ry;

	// fix the joystick values
	if (autonomousJoystickX < -1.0) autonomousJoystickX = -1.0;
	if (autonomousJoystickX >  1.0) autonomousJoystickX =  1.0;
	if (autonomousJoystickY < -1.0) autonomousJoystickY = -1.0;
	if (autonomousJoystickY >  1.0) autonomousJoystickY =  1.0;

	// release the memory we allocated for matrices
	if (T) gsl_matrix_free(T);
	if (Tpose) gsl_matrix_free(Tpose);
	if (iTpose) gsl_matrix_free(iTpose);
	if (p) gsl_permutation_free(p);
	if (lookAhead) gsl_matrix_free(lookAhead);
	if (lookAheadTransformed) gsl_matrix_free(lookAheadTransformed);

	// return true if everything went well
	return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::drawControlLookAheadPoint()                                       *
*                                                                              *
*  This function draws the autonomous look ahead point.                        *
*                                                                              *
\******************************************************************************/
bool autonomy::drawControlLookAheadPoint(void) {

	// safety check that everything's okay
	if (!autonomousDrivingFlag) {
		printf("\tError: autonomy, can't draw autonomous look ahead when not in autonomous mode.\n\n");
		return false;
	} else if (!controlLookAheadPoint) {
		printf("\tError: autonomy, can't draw autonomous look ahead when point not allocated.\n\n");
		return false;
	}

	// parameters for drawing a sphere
	int numThetas = 72; int numPhis = 36;
	int numVertices = 4*numThetas*numPhis;
	int numPolygons = numThetas*numPhis;
	float radius = 1.0f;

	// allocate the vertices and polygon vectors
	GLfloat *vertices = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*3);
	GLfloat *normals = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*3);
	GLfloat *colors = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*4);
	unsigned int *indices = (unsigned int *)malloc(sizeof(unsigned int)*numPolygons*4);

	// compute all of the polygons
	int numVerticesAdded = 0;
	for (int thetaIdx = 0; thetaIdx < numThetas; thetaIdx++) {
		for (int phiIdx = 0; phiIdx < numPhis; phiIdx++) {
			for (int j = 0; j <= 1; j++) {
				for (int k = 0; k <= 1; k++) {
					float h = (float)((j || k) && !(j && k));
					vertices[3*numVerticesAdded] = (GLfloat)controlLookAheadPoint[0] + radius*cos(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
					vertices[3*numVerticesAdded+1] = (GLfloat)controlLookAheadPoint[1] + radius*sin(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
					vertices[3*numVerticesAdded+2] = (GLfloat)controlLookAheadPoint[2] + radius*sin(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
					normals[3*numVerticesAdded] = radius*cos(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
					normals[3*numVerticesAdded+1] = radius*sin(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
					normals[3*numVerticesAdded+2] = radius*sin(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
					colors[4*numVerticesAdded] = 1.0; colors[4*numVerticesAdded+1] = 1.0; colors[4*numVerticesAdded+2] = 0.0; colors[4*numVerticesAdded+3] = 1.0;
					numVerticesAdded++;
				}
			}
		}
	}

	// there is no sharing of vertices between polygons
	for (int i = 0; i < 4*numPolygons; i++) indices[i] = i;

	// write out the gl commands into the list
	glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
	glNormalPointer(GL_FLOAT, 0, normals);
    glColorPointer(4, GL_FLOAT, 0, colors);
    glVertexPointer(3, GL_FLOAT, 0, vertices);
    glDrawElements(GL_QUADS, numPolygons*4, GL_UNSIGNED_INT, indices);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

	// free allocated memory
	if (vertices) free(vertices);
	if (normals) free(normals);
	if (colors) free(colors);
	if (indices) free(indices);

	// return true if everything went well
	return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::computeControlLookAheadPoint()                                    *
*                                                                              *
*  This function computes the joystick inputs to try to steer towards the      *
*    spline curves. NOTE: this needs X_distal_kalm in snakeEstimation in       *
*    order for this to work properly.                                          *
*                                                                              *
\******************************************************************************/
bool autonomy::computeControlLookAheadPoint() {

	// check the we're estimating the pose of the distal link of the snake
	if (!mySnakeEstimation->isInitializedSnakeEstimation()) {
		printf("\tError: autonomy, can't compute autonomous joystick inputs without distal kalm estimate.\n\n");
		return false;
	}

	// allocate some matrices
	gsl_matrix *T = gsl_matrix_alloc(4,4);
	gsl_matrix *Tpose = gsl_matrix_alloc(4,4);

	// get the 6-DOF estimate of the distal link of the snake
	if (!mySnakeEstimation->getDistalTransformationMatrix(Tpose)) {
		if (T) gsl_matrix_free(T);
		if (Tpose) gsl_matrix_free(Tpose);
		printf("\tError: autonomy, couldn't grab distal transformation matrix from snakeEstimation.\n\n");
		return false;
	}

	// get the transformation matrix for registration and then transform Tpose
	if (myRegistration->get_T_tracker2model(T)) {
		gsl_matrix *Ttemp = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, T, Tpose, 0.0, Ttemp);
		gsl_matrix_free(Tpose);
		Tpose = Ttemp;
	}

	// find the closest point on the target path
	double bestDistSquared = 1e9; int bestSpline = 0; int bestPointIndex = 0;
	for (int i = 0; i < numSplineSegments; i++) {
		for (int j = 0; j < numCurvePathPointsInEachSegment[i]; j++) {
			double distSquared = 0.0;
			for (int k = 0; k < 3; k++)
				distSquared += (curvePathPoints[i][j][k]-gsl_matrix_get(Tpose, k, 3))*(curvePathPoints[i][j][k]-gsl_matrix_get(Tpose, k, 3));
			if (distSquared < bestDistSquared) {
				bestDistSquared = distSquared;
				bestSpline = i;
				bestPointIndex = j;
			}
		}
	}

	// find the look ahead point that we'll aim the robot towards
	double dist = 0.0;
	int i = 0, j = 0;
	if (!controlLookAheadPoint) controlLookAheadPoint = (double *)malloc(sizeof(double)*3);
	for (i = bestSpline; i < numSplineSegments && dist < CONTROL_LOOK_AHEAD_DIST; i++) {
		int startPointIndex = 0;
		if (i == bestSpline) startPointIndex = bestPointIndex;
		for (j = startPointIndex+1; j < numCurvePathPointsInEachSegment[i] && dist < CONTROL_LOOK_AHEAD_DIST; j++) {
			double addDistSquared = 0.0;
			for (int k = 0; k < 3; k++)
				addDistSquared += (curvePathPoints[i][j][k]-curvePathPoints[i][j-1][k])*(curvePathPoints[i][j][k]-curvePathPoints[i][j-1][k]);
			dist += sqrt(addDistSquared);
			if (dist >= CONTROL_LOOK_AHEAD_DIST) {
				for (int k = 0; k < 3; k++) controlLookAheadPoint[k] = curvePathPoints[i][j][k];
			}
		}
	}

	// free the extra memory
	if (T) gsl_matrix_free(T);
	if (Tpose) gsl_matrix_free(Tpose);

	// return that we've successfully chosen joystick commands
	return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::getAutonomousJoystickX()                                          *
*                                                                              *
*  This function gets the value of the autonomous joystick x value.            *
*                                                                              *
\******************************************************************************/
float autonomy::getAutonomousJoystickX() {
	return autonomousJoystickX;
}

/******************************************************************************\
*                                                                              *
*  autonomy::setAutonomousJoystickY()                                          *
*                                                                              *
*  This function gets the value of the autonomous joystick y value.            *
*                                                                              *
\******************************************************************************/
float autonomy::getAutonomousJoystickY() {
	return autonomousJoystickY;
}

/******************************************************************************\
*                                                                              *
*  autonomy::setAutonomousDrivingFlag()                                        *
*                                                                              *
*  This function sets the value of the autonomous driving flag.                *
*                                                                              *
\******************************************************************************/
void autonomy::setAutonomousDrivingFlag(bool val) {
	autonomousJoystickX = 0.0;
	autonomousJoystickY = 0.0;
	if (val == false) {
		if (controlLookAheadPoint) free(controlLookAheadPoint);
		controlLookAheadPoint = NULL;
		autonomousDrivingFlag = val;
	} else if (pickingPointsFlag) {
		printf("\tError: autonomy, could not enable autonomy while picking points.\n\n");
		return;
	} else if (numSplineSegments <= 0) {
		printf("\tError: autonomy, could not enable autonomy with insufficient spline paths to follow.\n\n");
		return;
	} else if (!mySnakeEstimation->isInitializedSnakeEstimation()) {
		printf("\tError: autonomy, could not enable autonomy when snake estimation is not initialized.\n\n");
		return;
	} else {
		autonomousDrivingFlag = true;
	}
}

/******************************************************************************\
*                                                                              *
*  autonomy::getAutonomousDrivingFlag()                                        *
*                                                                              *
*  This function returns the value of autonomous driving flag.                 *
*                                                                              *
\******************************************************************************/
bool autonomy::getAutonomousDrivingFlag() {
	return autonomousDrivingFlag;
}

/******************************************************************************\
*                                                                              *
*  autonomy::setVisibleSplineCurve()                                           *
*                                                                              *
*  This function sets the value of the visibleSplineCurve flag.                *
*                                                                              *
\******************************************************************************/
void autonomy::setVisibleSplineCurve(bool val) {
	visibleSplineCurve = val;
}

/******************************************************************************\
*                                                                              *
*  autonomy::getVisibleSplineCurve()                                           *
*                                                                              *
*  This function returns the value of the visibleSplineCurve flag.             *
*                                                                              *
\******************************************************************************/
bool autonomy::getVisibleSplineCurve() {
	return visibleSplineCurve;
}

/******************************************************************************\
*                                                                              *
*  autonomy::setVisiblePreCurvePathPoints()                                    *
*                                                                              *
*  This function sets the value of the visiblePreCurvePathPoints flag.         *
*                                                                              *
\******************************************************************************/
void autonomy::setVisiblePreCurvePathPoints(bool val) {
	visiblePreCurvePathPoints = val;
}

/******************************************************************************\
*                                                                              *
*  autonomy::getVisiblePreCurvePathPoints()                                    *
*                                                                              *
*  This function returns the value of the visiblePreCurvePathPoints flag.      *
*                                                                              *
\******************************************************************************/
bool autonomy::getVisiblePreCurvePathPoints() {
	return visiblePreCurvePathPoints;
}

/******************************************************************************\
*                                                                              *
*  autonomy::setPickingPointsFlag()                                            *
*                                                                              *
*  This function sets the value of the pickingPointsFlag.                      *
*                                                                              *
\******************************************************************************/
void autonomy::setPickingPointsFlag(bool val) {
	if (val == false) {
		pickingPointsFlag = val;
	} else {
		if (autonomousDrivingFlag) {
			printf("\tError: autonomy, could not enable picking points while in autonomy.\n\n");
			return;
		}
		pickingPointsFlag = val;
	}

}

/******************************************************************************\
*                                                                              *
*  autonomy::getPickingPointsFlag()                                            *
*                                                                              *
*  This function returns the value of the pickingPointsFlag.                   *
*                                                                              *
\******************************************************************************/
bool autonomy::getPickingPointsFlag() {
	return pickingPointsFlag;
}

/******************************************************************************\
*                                                                              *
*  autonomy::getUpdatePreCurveDrawFlag()                                       *
*                                                                              *
*  This function returns the value of the precurve updateDrawFlag.             *
*                                                                              *
\******************************************************************************/
bool autonomy::getUpdatePreCurveDrawFlag() {
	return updatePreCurveDrawFlag;
}

/******************************************************************************\
*                                                                              *
*  autonomy::getUpdateCurveDrawFlag()                                          *
*                                                                              *
*  This function returns the value of the curve updateDrawFlag.                *
*                                                                              *
\******************************************************************************/
bool autonomy::getUpdateCurveDrawFlag() {
	return updateCurveDrawFlag;
}

/******************************************************************************\
*                                                                              *
*  autonomy::checkSameClockDir()                                               *
*                                                                              *
*  This function checks that the intersection pt is good.                      *
*                                                                              *
\******************************************************************************/
bool autonomy::checkSameClockDir(float *pt1, float *pt2, float *pt3, float *norm) {  

	float testi, testj, testk;
	float dotprod;

	// normal of triangle
	testi = (((pt2[1] - pt1[1])*(pt3[2] - pt1[2])) - ((pt3[1] - pt1[1])*(pt2[2] - pt1[2])));
	testj = (((pt2[2] - pt1[2])*(pt3[0] - pt1[0])) - ((pt3[2] - pt1[2])*(pt2[0] - pt1[0])));
	testk = (((pt2[0] - pt1[0])*(pt3[1] - pt1[1])) - ((pt3[0] - pt1[0])*(pt2[1] - pt1[1])));

	// dot product with triangle normal
	dotprod = testi*norm[0] + testj*norm[1] + testk*norm[2];

	// answer
	if (dotprod < 0) return false;
	else return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::computeTriangleIntersection()                                     *
*                                                                              *
*  This function computes the intersection pt between a line and a triangle.   *
*                                                                              *
\******************************************************************************/
bool autonomy::computeLineTriangleIntersection(float *tri_pt1, float *tri_pt2, float *tri_pt3, float *line_pt1, float *line_pt2, float *intersection_pt) {

	// vector from line pt1 to pt2
	float vectx = line_pt2[0]-line_pt1[0];
	float vecty = line_pt2[1]-line_pt1[1];
	float vectz = line_pt2[2]-line_pt1[2];
	
	// vector from triangle pt1 to pt2
	float V1x = tri_pt2[0] - tri_pt1[0];
	float V1y = tri_pt2[1] - tri_pt1[1];
	float V1z = tri_pt2[2] - tri_pt1[2];
	
	// vector from triangle pt2 to pt3
	float V2x = tri_pt3[0] - tri_pt2[0];
	float V2y = tri_pt3[1] - tri_pt2[1];
	float V2z = tri_pt3[2] - tri_pt2[2];
	
	// vector normal of triangle
	float norm[3];
	norm[0] = V1y*V2z-V1z*V2y;
	norm[1] = V1z*V2x-V1x*V2z;
	norm[2] = V1x*V2y-V1y*V2x;
	
	// dot product of normal and line's vector if zero line is parallel to triangle
	float dotprod = norm[0]*vectx + norm[1]*vecty + norm[2]*vectz;
	
	if (dotprod < 0) {
		
		// Find point of intersect to triangle plane, find t to intersect point
		float t = -(norm[0]*(line_pt1[0]-tri_pt1[0])+norm[1]*(line_pt1[1]-tri_pt1[1])+norm[2]*(line_pt1[2]-tri_pt1[2]))/(norm[0]*vectx+norm[1]*vecty+norm[2]*vectz);
		
		// if ds is neg line started past triangle so can't hit triangle.
		if (t < 0) return false;
		
		intersection_pt[0] = line_pt1[0] + vectx*t;
		intersection_pt[1] = line_pt1[1] + vecty*t;
		intersection_pt[2] = line_pt1[2] + vectz*t;
		
		if (checkSameClockDir(tri_pt1, tri_pt2, intersection_pt, norm)) {
			if (checkSameClockDir(tri_pt2, tri_pt3, intersection_pt, norm)) {
				if (checkSameClockDir(tri_pt3, tri_pt1, intersection_pt, norm)) {
					return true;
				}
			}
		}
	}
	return false;
}

/******************************************************************************\
*                                                                              *
*  autonomy::clearPath()                                                       *
*                                                                              *
*  This function clears all information stored for the path.                   *
*                                                                              *
\******************************************************************************/
bool autonomy::clearPath() {

	// free control ray near points array
	if (controlRayNearPoints) {
		for (int i = 0; i < numControlPoints; i++) {
			if (controlRayNearPoints[i]) free(controlRayNearPoints[i]);
			controlRayNearPoints[i] = NULL;
		}
		free(controlRayNearPoints);
	}
	controlRayNearPoints = NULL;

	// free control ray far points array
	if (controlRayFarPoints) {
		for (int i = 0; i < numControlPoints; i++) {
			if (controlRayFarPoints[i]) free(controlRayFarPoints[i]);
			controlRayFarPoints[i] = NULL;
		}
		free(controlRayFarPoints);
	}
	controlRayFarPoints = NULL;

	// free control surface points array
	if (controlSurfacePoints) {
		for (int i = 0; i < numControlPoints; i++) {
			if (controlSurfacePoints[i]) free(controlSurfacePoints[i]);
			controlSurfacePoints[i] = NULL;
		}
		free(controlSurfacePoints);
	}
	controlSurfacePoints = NULL;

	// free control surface points pushed out array
	if (controlSurfacePointsPushedOut) {
		for (int i = 0; i < numControlPoints; i++) {
			if (controlSurfacePointsPushedOut[i]) free(controlSurfacePointsPushedOut[i]);
			controlSurfacePointsPushedOut[i] = NULL;
		}
		free(controlSurfacePointsPushedOut);
	}
	controlSurfacePointsPushedOut = NULL;

	// free pre-curve points array
	if (preCurvePathPoints) {
		for (int i = 0; i < numSegmentsAddedToPreCurvePathPoints; i++) {
			for (int j = 0; j < numPreCurvePathPointsInEachSegment[i]; j++) {
				if (preCurvePathPoints[i][j]) free(preCurvePathPoints[i][j]);
				preCurvePathPoints[i][j] = NULL;
			}
			if (preCurvePathPoints[i]) free(preCurvePathPoints[i]);
			preCurvePathPoints[i] = NULL;
		}
		free(preCurvePathPoints);
	}
	preCurvePathPoints = NULL;

	// free pre-curve pushed out points array
	if (preCurvePathPointsPushedOut) {
		for (int i = 0; i < numSegmentsAddedToPreCurvePathPoints; i++) {
			for (int j = 0; j < numPreCurvePathPointsInEachSegment[i]; j++) {
				if (preCurvePathPointsPushedOut[i][j]) free(preCurvePathPointsPushedOut[i][j]);
				preCurvePathPointsPushedOut[i][j] = NULL;
			}
			if (preCurvePathPointsPushedOut[i]) free(preCurvePathPointsPushedOut[i]);
			preCurvePathPointsPushedOut[i] = NULL;
		}
		free(preCurvePathPointsPushedOut);
	}
	preCurvePathPointsPushedOut = NULL;

	// free the spline parameters
	if (splineParametersForEachSegment) {
		for (int i = 0; i < numSplineSegments; i++) {
			if (splineParametersForEachSegment[i]) free(splineParametersForEachSegment[i]);
			splineParametersForEachSegment[i] = NULL;
		}
		free(splineParametersForEachSegment);
	}
	splineParametersForEachSegment = NULL;

	// free the curve points generated from the splines
	if (curvePathPoints) {
		for (int i = 0; i < numSplineSegments; i++) {
			for (int j = 0; j < numCurvePathPointsInEachSegment[i]; j++) {
				if (curvePathPoints[i][j]) free(curvePathPoints[i][j]);
				curvePathPoints[i][j] = NULL;
			}
			if (curvePathPoints[i]) free(curvePathPoints[i]);
			curvePathPoints[i] = NULL;
		}
		free(curvePathPoints);
	}
	curvePathPoints = NULL;

	// free the array that stores the num pre-curve points in each segment
	if (numPreCurvePathPointsInEachSegment) free(numPreCurvePathPointsInEachSegment);
	numPreCurvePathPointsInEachSegment = NULL;

	// free the array that stores the num curve points in each segment
	if (numCurvePathPointsInEachSegment) free(numCurvePathPointsInEachSegment);
	numCurvePathPointsInEachSegment = NULL;

	// erase the control look ahead point if it exists
	if (controlLookAheadPoint) free(controlLookAheadPoint);
	controlLookAheadPoint = NULL;

	// reinitialize variables
	numSplineSegments = 0;
	numSegmentsAddedToPreCurvePathPoints = 0;
	numControlPoints = 0;
	updatePreCurveDrawFlag = true;
	updateCurveDrawFlag = true;
	autonomousDrivingFlag = false;
	autonomousJoystickX = 0.0;
	autonomousJoystickY = 0.0;

	// return if everything freed correctly
	return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::updatePreCurvePathPoints()                                        *
*                                                                              *
*  This function computes a set of intermediate intersection points between    *
*     two clicked-on control points when choosing the autonomous path.         *
*                                                                              *
\******************************************************************************/
bool autonomy::updatePreCurvePathPoints(int numPolygonMeshes, polygonMesh **polygonMeshes) {

	float line_pt1[3], line_pt2[3];
	float tri_pt1[3], tri_pt2[3], tri_pt3[3], intersection_pt[3];

	if (numControlPoints > numSegmentsAddedToPreCurvePathPoints + 2 || numControlPoints <= numSegmentsAddedToPreCurvePathPoints) {
		printf("\tError: autonomy, updating preCurvePathPoints, this error should not happen.\n\n");
		return false;
	} else if (numControlPoints == numSegmentsAddedToPreCurvePathPoints + 2) {  // then we must add a new segment of points
		double ***preCurvePathPointsNew = (double ***)malloc(sizeof(double **)*(numSegmentsAddedToPreCurvePathPoints+1));
		double ***preCurvePathPointsPushedOutNew = (double ***)malloc(sizeof(double **)*(numSegmentsAddedToPreCurvePathPoints+1));
		int *numPreCurvePathPointsInEachSegmentNew = (int *)malloc(sizeof(int)*(numSegmentsAddedToPreCurvePathPoints+1));
		if (numSegmentsAddedToPreCurvePathPoints > 0) {
			memcpy_s(preCurvePathPointsNew, sizeof(double **)*numSegmentsAddedToPreCurvePathPoints, preCurvePathPoints, sizeof(double **)*numSegmentsAddedToPreCurvePathPoints);
			memcpy_s(preCurvePathPointsPushedOutNew, sizeof(double **)*numSegmentsAddedToPreCurvePathPoints, preCurvePathPointsPushedOut, sizeof(double **)*numSegmentsAddedToPreCurvePathPoints);
			memcpy_s(numPreCurvePathPointsInEachSegmentNew, sizeof(int)*numSegmentsAddedToPreCurvePathPoints, numPreCurvePathPointsInEachSegment, sizeof(int)*numSegmentsAddedToPreCurvePathPoints);
		}
		if (preCurvePathPoints) free(preCurvePathPoints);
		preCurvePathPoints = preCurvePathPointsNew;
		if (preCurvePathPointsPushedOut) free(preCurvePathPointsPushedOut);
		preCurvePathPointsPushedOut = preCurvePathPointsPushedOutNew;
		if (numPreCurvePathPointsInEachSegment) free(numPreCurvePathPointsInEachSegment);
		numPreCurvePathPointsInEachSegment = numPreCurvePathPointsInEachSegmentNew;

		double **newSegmentPoints = NULL;
		double **newSegmentPointsPushedOut = NULL;
		int numNewSegmentPoints = 0;

		float lineLength = (float)sqrt(((float)controlSurfacePoints[numControlPoints-2][0]-(float)controlSurfacePoints[numControlPoints-1][0])*((float)controlSurfacePoints[numControlPoints-2][0]-(float)controlSurfacePoints[numControlPoints-1][0]) +
									   ((float)controlSurfacePoints[numControlPoints-2][1]-(float)controlSurfacePoints[numControlPoints-1][1])*((float)controlSurfacePoints[numControlPoints-2][1]-(float)controlSurfacePoints[numControlPoints-1][1]) +
									   ((float)controlSurfacePoints[numControlPoints-2][2]-(float)controlSurfacePoints[numControlPoints-1][2])*((float)controlSurfacePoints[numControlPoints-2][2]-(float)controlSurfacePoints[numControlPoints-1][2]));

		for (float u = 3.0f/lineLength; u < 1.0f -(3.0f/lineLength)/2.0f; u += 3.0f/lineLength) {

			line_pt1[0] = (float)controlSurfacePoints[numControlPoints-2][0] + u*((float)controlSurfacePoints[numControlPoints-1][0] - (float)controlSurfacePoints[numControlPoints-2][0]);
			line_pt1[1] = (float)controlSurfacePoints[numControlPoints-2][1] + u*((float)controlSurfacePoints[numControlPoints-1][1] - (float)controlSurfacePoints[numControlPoints-2][1]);
			line_pt1[2] = (float)controlSurfacePoints[numControlPoints-2][2] + u*((float)controlSurfacePoints[numControlPoints-1][2] - (float)controlSurfacePoints[numControlPoints-2][2]);
			line_pt2[0] = (float)controlSurfacePoints[numControlPoints-2][0] + u*((float)controlSurfacePoints[numControlPoints-1][0] - (float)controlSurfacePoints[numControlPoints-2][0]);
			line_pt2[1] = (float)controlSurfacePoints[numControlPoints-2][1] + u*((float)controlSurfacePoints[numControlPoints-1][1] - (float)controlSurfacePoints[numControlPoints-2][1]);
			line_pt2[2] = (float)controlSurfacePoints[numControlPoints-2][2] + u*((float)controlSurfacePoints[numControlPoints-1][2] - (float)controlSurfacePoints[numControlPoints-2][2]);

			float vectorNearToFar1[3], vectorNearToFar2[3];
			vectorNearToFar1[0] = (float)controlRayFarPoints[numControlPoints-2][0] - (float)controlRayNearPoints[numControlPoints-2][0];
			vectorNearToFar1[1] = (float)controlRayFarPoints[numControlPoints-2][1] - (float)controlRayNearPoints[numControlPoints-2][1];
			vectorNearToFar1[2] = (float)controlRayFarPoints[numControlPoints-2][2] - (float)controlRayNearPoints[numControlPoints-2][2];
			vectorNearToFar2[0] = (float)controlRayFarPoints[numControlPoints-1][0] - (float)controlRayNearPoints[numControlPoints-1][0];
			vectorNearToFar2[1] = (float)controlRayFarPoints[numControlPoints-1][1] - (float)controlRayNearPoints[numControlPoints-1][1];
			vectorNearToFar2[2] = (float)controlRayFarPoints[numControlPoints-1][2] - (float)controlRayNearPoints[numControlPoints-1][2];

			line_pt1[0] -= vectorNearToFar1[0]/2.0f + u*(vectorNearToFar2[0] - vectorNearToFar1[0]);
			line_pt1[1] -= vectorNearToFar1[1]/2.0f + u*(vectorNearToFar2[1] - vectorNearToFar1[1]);
			line_pt1[2] -= vectorNearToFar1[2]/2.0f + u*(vectorNearToFar2[2] - vectorNearToFar1[2]);
			
			line_pt2[0] += vectorNearToFar1[0]/2.0f + u*(vectorNearToFar2[0] - vectorNearToFar1[0]);
			line_pt2[1] += vectorNearToFar1[1]/2.0f + u*(vectorNearToFar2[1] - vectorNearToFar1[1]);
			line_pt2[2] += vectorNearToFar1[2]/2.0f + u*(vectorNearToFar2[2] - vectorNearToFar1[2]);

			float dist = 0.0;
			int numIntersections = 0;
			for (int j = 0; j < numPolygonMeshes; j++) {
				if (!polygonMeshes[j]->getVisibleFlag()) continue;
				if (polygonMeshes[j]->getNumVerticesPerPolygon() != 3) {
					printf("\tError: autonomy, cannot intersect ray with faces other than triangles.\n\n");
				} else {
					for (int k = 0; k < polygonMeshes[j]->getNumPolygons(); k++) {

						intersection_pt[0] = 0; intersection_pt[1] = 0; intersection_pt[2] = 0;
						tri_pt1[0] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k));
						tri_pt1[1] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k)+1);
						tri_pt1[2] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k)+2);
						tri_pt2[0] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+1));
						tri_pt2[1] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+1)+1);
						tri_pt2[2] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+1)+2);
						tri_pt3[0] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+2));
						tri_pt3[1] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+2)+1);
						tri_pt3[2] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+2)+2);

						bool returnVal1 = computeLineTriangleIntersection(tri_pt1, tri_pt2, tri_pt3, line_pt1, line_pt2, intersection_pt);
						bool returnVal2 = computeLineTriangleIntersection(tri_pt1, tri_pt3, tri_pt2, line_pt1, line_pt2, intersection_pt);

						// vector from triangle pt1 to pt2
						float V1x = tri_pt2[0] - tri_pt1[0];
						float V1y = tri_pt2[1] - tri_pt1[1];
						float V1z = tri_pt2[2] - tri_pt1[2];
	
						// vector from triangle pt2 to pt3
						float V2x = tri_pt3[0] - tri_pt2[0];
						float V2y = tri_pt3[1] - tri_pt2[1];
						float V2z = tri_pt3[2] - tri_pt2[2];
	
						// vector normal of triangle
						float norm[3];
						norm[0] = V1y*V2z-V1z*V2y;
						norm[1] = V1z*V2x-V1x*V2z;
						norm[2] = V1x*V2y-V1y*V2x;
						float normLength = sqrt(norm[0]*norm[0] + norm[1]*norm[1] + norm[2]*norm[2]);

						if (returnVal1 || returnVal2) {
							if (numIntersections == 0) {

								double **newSegmentPointsNew = (double **)malloc(sizeof(double *)*(numNewSegmentPoints+1));
								double **newSegmentPointsPushedOutNew = (double **)malloc(sizeof(double *)*(numNewSegmentPoints+1));

								newSegmentPointsNew[numNewSegmentPoints] = (double *)malloc(sizeof(double)*3);
								newSegmentPointsNew[numNewSegmentPoints][0] = intersection_pt[0];
								newSegmentPointsNew[numNewSegmentPoints][1] = intersection_pt[1];
								newSegmentPointsNew[numNewSegmentPoints][2] = intersection_pt[2];

								newSegmentPointsPushedOutNew[numNewSegmentPoints] = (double *)malloc(sizeof(double)*3);
								newSegmentPointsPushedOutNew[numNewSegmentPoints][0] = intersection_pt[0] + 6*norm[0]/normLength;
								newSegmentPointsPushedOutNew[numNewSegmentPoints][1] = intersection_pt[1] + 6*norm[1]/normLength;
								newSegmentPointsPushedOutNew[numNewSegmentPoints][2] = intersection_pt[2] + 6*norm[2]/normLength;

								if (numNewSegmentPoints > 0) {
									memcpy_s(newSegmentPointsNew, sizeof(double *)*numNewSegmentPoints, newSegmentPoints, sizeof(double *)*numNewSegmentPoints);
									memcpy_s(newSegmentPointsPushedOutNew, sizeof(double *)*numNewSegmentPoints, newSegmentPointsPushedOut, sizeof(double *)*numNewSegmentPoints);
								}

								if (newSegmentPoints) free(newSegmentPoints);
								newSegmentPoints = newSegmentPointsNew;
								if (newSegmentPointsPushedOut) free(newSegmentPointsPushedOut);
								newSegmentPointsPushedOut = newSegmentPointsPushedOutNew;
								dist = (line_pt1[0]-intersection_pt[0])*(line_pt1[0]-intersection_pt[0]) + (line_pt1[1]-intersection_pt[1])*(line_pt1[1]-intersection_pt[1]) + (line_pt1[2]-intersection_pt[2])*(line_pt1[2]-intersection_pt[2]); 
								numNewSegmentPoints++;

							} else {
								float newDist = (line_pt1[0]-intersection_pt[0])*(line_pt1[0]-intersection_pt[0]) + (line_pt1[1]-intersection_pt[1])*(line_pt1[1]-intersection_pt[1]) + (line_pt1[2]-intersection_pt[2])*(line_pt1[2]-intersection_pt[2]); 
								if (newDist < dist) {
									dist = newDist;

									newSegmentPoints[numNewSegmentPoints-1][0] = intersection_pt[0];
									newSegmentPoints[numNewSegmentPoints-1][1] = intersection_pt[1];
									newSegmentPoints[numNewSegmentPoints-1][2] = intersection_pt[2];

									newSegmentPointsPushedOut[numNewSegmentPoints-1][0] = intersection_pt[0] + 6*norm[0]/normLength;
									newSegmentPointsPushedOut[numNewSegmentPoints-1][1] = intersection_pt[1] + 6*norm[1]/normLength;
									newSegmentPointsPushedOut[numNewSegmentPoints-1][2] = intersection_pt[2] + 6*norm[2]/normLength;
								}
							}
							numIntersections++;
						}
					}
				}
			}	
		}

		preCurvePathPoints[numSegmentsAddedToPreCurvePathPoints] = newSegmentPoints;
		preCurvePathPointsPushedOut[numSegmentsAddedToPreCurvePathPoints] = newSegmentPointsPushedOut;
		numPreCurvePathPointsInEachSegment[numSegmentsAddedToPreCurvePathPoints] = numNewSegmentPoints;
		numSegmentsAddedToPreCurvePathPoints++;

	}

	updatePreCurveDrawFlag = true;
	return true;
}


/******************************************************************************\
*                                                                              *
*  autonomy::computePath()                                                     *
*                                                                              *
*  This function computes the desired path for autonomous control.             *
*                                                                              *
\******************************************************************************/
bool autonomy::computePath() {
	
	// free spline parameters stored already
	if (splineParametersForEachSegment) {
		for (int i = 0; i < numSplineSegments; i++) {
			if (splineParametersForEachSegment[i]) free(splineParametersForEachSegment[i]);
			splineParametersForEachSegment[i] = NULL;
		}
		free(splineParametersForEachSegment);
	}
	splineParametersForEachSegment = NULL;
	numSplineSegments = 0;

	// free curve path points stored for drawing as we will update these
	if (curvePathPoints) {
		for (int i = 0; i < numSplineSegments; i++) {
			for (int j = 0; j < numCurvePathPointsInEachSegment[i]; j++) {
				if (curvePathPoints[i][j]) free(curvePathPoints[i][j]);
				curvePathPoints[i][j] = NULL;
			}
			if (curvePathPoints[i]) free(curvePathPoints[i]);
			curvePathPoints[i] = NULL;
		}
		free(curvePathPoints);
	}
	curvePathPoints = NULL;

	// free the num curve path points array
	if (numCurvePathPointsInEachSegment) free(numCurvePathPointsInEachSegment);
	numCurvePathPointsInEachSegment = NULL;

	// only compute splines if there are enough control points
	if (numControlPoints >= 2) {
		if (numControlPoints != numSegmentsAddedToPreCurvePathPoints + 1) {
			printf("\tError: autonomy, mismatch between numControlPoints and numSegmentsAddedToPreCurvePathPoints+1 in computePath().\n\n");
			return false;
		} else {

			// allocate some matrices
			gsl_matrix *point = gsl_matrix_alloc(4,1);
			gsl_matrix *transformedPoint = gsl_matrix_alloc(4,1);
			gsl_matrix *T = gsl_matrix_calloc(4,4);
			gsl_matrix_set(T, 3, 3, 1.0);
			gsl_matrix *AtA = gsl_matrix_alloc(2*(NUM_SPLINE_PARAMETERS-2),2*(NUM_SPLINE_PARAMETERS-2));
			gsl_matrix *iAtA = gsl_matrix_alloc(2*(NUM_SPLINE_PARAMETERS-2),2*(NUM_SPLINE_PARAMETERS-2));
			gsl_permutation *p = gsl_permutation_alloc(2*(NUM_SPLINE_PARAMETERS-2));
			gsl_permutation *p2 = gsl_permutation_alloc(4);
			gsl_matrix *W = gsl_matrix_calloc(4,4);
			gsl_matrix *iT = gsl_matrix_calloc(4,4);

			// initialize spline data for storing
			numSplineSegments = numSegmentsAddedToPreCurvePathPoints;
			splineParametersForEachSegment = (double **)malloc(sizeof(double *)*numSplineSegments);

			// initialize curve path points stuff			
			numCurvePathPointsInEachSegment = (int *)malloc(sizeof(int)*numSplineSegments);
			curvePathPoints = (double ***)malloc(sizeof(double **)*numSplineSegments);

			// for each spline segment
			for (int i = 0; i < numSplineSegments; i++) {

				numCurvePathPointsInEachSegment[i] = 0;
				curvePathPoints[i] = NULL;
				splineParametersForEachSegment[i] = NULL;
				if (numPreCurvePathPointsInEachSegment[i] < NUM_SPLINE_PARAMETERS-2)
					continue;

				// for each spline, allocate an array to store the parameters
				splineParametersForEachSegment[i] = (double *)malloc(sizeof(double)*2*NUM_SPLINE_PARAMETERS);

				// allocate matrices for solving for the spline parameters
				gsl_matrix *A = gsl_matrix_calloc(2*numPreCurvePathPointsInEachSegment[i], 2*(NUM_SPLINE_PARAMETERS-2));
				gsl_matrix *x = gsl_matrix_alloc(2*(NUM_SPLINE_PARAMETERS-2), 1);
				gsl_matrix *b = gsl_matrix_alloc(2*numPreCurvePathPointsInEachSegment[i], 1);
				gsl_matrix *iAtAAt = gsl_matrix_alloc(2*(NUM_SPLINE_PARAMETERS-2), 2*numPreCurvePathPointsInEachSegment[i]);
				gsl_matrix *transformedPointStart = gsl_matrix_alloc(4,1);
				gsl_matrix *transformedPointEnd = gsl_matrix_alloc(4,1);

				// compute the transformation to put the pre curve points on the x-axis
				double yaw = atan2(controlSurfacePointsPushedOut[i+1][1]-controlSurfacePointsPushedOut[i][1],controlSurfacePointsPushedOut[i+1][0]-controlSurfacePointsPushedOut[i][0]);
				double pitch = atan2(controlSurfacePointsPushedOut[i+1][2]-controlSurfacePointsPushedOut[i][2],sqrt((controlSurfacePointsPushedOut[i+1][1]-controlSurfacePointsPushedOut[i][1])*(controlSurfacePointsPushedOut[i+1][1]-controlSurfacePointsPushedOut[i][1])+(controlSurfacePointsPushedOut[i+1][0]-controlSurfacePointsPushedOut[i][0])*(controlSurfacePointsPushedOut[i+1][0]-controlSurfacePointsPushedOut[i][0])));

				// compute the transformation matrix and the inverse transformation matrix
				int s;
				gsl_matrix_set(T, 0, 0, cos(pitch)*cos(yaw)); gsl_matrix_set(T, 0, 1, cos(pitch)*sin(yaw)); gsl_matrix_set(T, 0, 2, sin(pitch));
				gsl_matrix_set(T, 0, 3, cos(pitch)*(-cos(yaw)*controlSurfacePointsPushedOut[i][0]-sin(yaw)*controlSurfacePointsPushedOut[i][1])-sin(pitch)*controlSurfacePointsPushedOut[i][2]);
				gsl_matrix_set(T, 1, 0, -sin(yaw)); gsl_matrix_set(T, 1, 1, cos(yaw)); gsl_matrix_set(T, 1, 2, 0.0);
				gsl_matrix_set(T, 1, 3, sin(yaw)*controlSurfacePointsPushedOut[i][0] - cos(yaw)*controlSurfacePointsPushedOut[i][1]);
				gsl_matrix_set(T, 2, 0, -sin(pitch)*cos(yaw)); gsl_matrix_set(T, 2, 1, -sin(pitch)*sin(yaw)); gsl_matrix_set(T, 2, 2, cos(pitch));
				gsl_matrix_set(T, 2, 3, -sin(pitch)*(-cos(yaw)*controlSurfacePointsPushedOut[i][0]-sin(yaw)*controlSurfacePointsPushedOut[i][1])-cos(pitch)*controlSurfacePointsPushedOut[i][2]);
				gsl_matrix_memcpy(W, T);
				gsl_linalg_LU_decomp(W, p2, &s);
				gsl_linalg_LU_invert(W, p2, iT);

				gsl_matrix_set(point, 0, 0, controlSurfacePointsPushedOut[i][0]);
				gsl_matrix_set(point, 1, 0, controlSurfacePointsPushedOut[i][1]);
				gsl_matrix_set(point, 2, 0, controlSurfacePointsPushedOut[i][2]);
				gsl_matrix_set(point, 3, 0, 1.0);
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, T, point, 0.0, transformedPointStart);
				gsl_matrix_set(point, 0, 0, controlSurfacePointsPushedOut[i+1][0]);
				gsl_matrix_set(point, 1, 0, controlSurfacePointsPushedOut[i+1][1]);
				gsl_matrix_set(point, 2, 0, controlSurfacePointsPushedOut[i+1][2]);
				gsl_matrix_set(point, 3, 0, 1.0);
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, T, point, 0.0, transformedPointEnd);


				// for each pre curve point, add it to the A matrix and b matrix
				for (int j = 0; j < numPreCurvePathPointsInEachSegment[i]; j++) {
					
					// transform the point to the new coordinate frame
					gsl_matrix_set(point, 0, 0, preCurvePathPointsPushedOut[i][j][0]);
					gsl_matrix_set(point, 1, 0, preCurvePathPointsPushedOut[i][j][1]);
					gsl_matrix_set(point, 2, 0, preCurvePathPointsPushedOut[i][j][2]);
					gsl_matrix_set(point, 3, 0, 1.0);
					gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, T, point, 0.0, transformedPoint);

					for (int k = 0; k < NUM_SPLINE_PARAMETERS-2; k++) {
						gsl_matrix_set(A, 2*j,   k, pow(gsl_matrix_get(transformedPoint, 0, 0), NUM_SPLINE_PARAMETERS-1-k) - gsl_matrix_get(transformedPoint, 0, 0)*pow(gsl_matrix_get(transformedPointEnd, 0, 0), NUM_SPLINE_PARAMETERS-2-k));
						gsl_matrix_set(A, 2*j+1, k+NUM_SPLINE_PARAMETERS-2, pow(gsl_matrix_get(transformedPoint, 0, 0), NUM_SPLINE_PARAMETERS-1-k) - gsl_matrix_get(transformedPoint, 0, 0)*pow(gsl_matrix_get(transformedPointEnd, 0, 0), NUM_SPLINE_PARAMETERS-2-k));
					}
					gsl_matrix_set(b, 2*j,   0, gsl_matrix_get(transformedPoint, 1, 0) - gsl_matrix_get(transformedPointStart, 1, 0) - (gsl_matrix_get(transformedPointEnd, 1, 0) - gsl_matrix_get(transformedPointStart, 1, 0))*gsl_matrix_get(transformedPoint, 0, 0)/gsl_matrix_get(transformedPointEnd, 0, 0));
					gsl_matrix_set(b, 2*j+1, 0, gsl_matrix_get(transformedPoint, 2, 0) - gsl_matrix_get(transformedPointStart, 2, 0) - (gsl_matrix_get(transformedPointEnd, 2, 0) - gsl_matrix_get(transformedPointStart, 2, 0))*gsl_matrix_get(transformedPoint, 0, 0)/gsl_matrix_get(transformedPointEnd, 0, 0));
				}

				// compute the psuedo inverse and get the spline parameters
				gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, A, A, 0.0, AtA);
				gsl_linalg_LU_decomp(AtA, p, &s);
				gsl_linalg_LU_invert(AtA, p, iAtA);
				gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, iAtA, A, 0.0, iAtAAt);
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, iAtAAt, b, 0.0, x);

				// store the spline parameters we computed
				double savedValue = 0.0;
				for (int j = 0; j < NUM_SPLINE_PARAMETERS-2; j++) {
					splineParametersForEachSegment[i][j] = gsl_matrix_get(x, j, 0);
					savedValue += -gsl_matrix_get(x, j, 0)*pow(gsl_matrix_get(transformedPointEnd, 0, 0), NUM_SPLINE_PARAMETERS-1-j);
				}
				splineParametersForEachSegment[i][NUM_SPLINE_PARAMETERS-2] = (savedValue + gsl_matrix_get(transformedPointEnd, 1, 0) - gsl_matrix_get(transformedPointStart, 1, 0))/gsl_matrix_get(transformedPointEnd, 0, 0);
				splineParametersForEachSegment[i][NUM_SPLINE_PARAMETERS-1] = gsl_matrix_get(transformedPointStart, 1, 0);
				savedValue = 0.0;
				for (int j = 0; j < NUM_SPLINE_PARAMETERS-2; j++) {
					splineParametersForEachSegment[i][j+NUM_SPLINE_PARAMETERS] = gsl_matrix_get(x, j+NUM_SPLINE_PARAMETERS-2, 0);
					savedValue += -gsl_matrix_get(x, j+NUM_SPLINE_PARAMETERS-2, 0)*pow(gsl_matrix_get(transformedPointEnd, 0, 0), NUM_SPLINE_PARAMETERS-1-j);
				}
				splineParametersForEachSegment[i][NUM_SPLINE_PARAMETERS-2+NUM_SPLINE_PARAMETERS] = (savedValue + gsl_matrix_get(transformedPointEnd, 2, 0) - gsl_matrix_get(transformedPointStart, 2, 0))/gsl_matrix_get(transformedPointEnd, 0, 0);
				splineParametersForEachSegment[i][NUM_SPLINE_PARAMETERS-1+NUM_SPLINE_PARAMETERS] = gsl_matrix_get(transformedPointStart, 2, 0);
								
				// store points along the spline
				int numCurvePathPointsToAdd = 0;
				double **curvePathPointsToAdd = NULL;

				// step along the x dimension of the spline
				for (double x = 0.0; x < gsl_matrix_get(transformedPointEnd, 0, 0); x += 0.1) {

					// compute y and z from the spline parameters
					double y = 0.0;
					double z = 0.0;
					for (int j = 0; j < NUM_SPLINE_PARAMETERS; j++) {
						y += splineParametersForEachSegment[i][j]*pow(x, NUM_SPLINE_PARAMETERS-1-j);
						z += splineParametersForEachSegment[i][j+NUM_SPLINE_PARAMETERS]*pow(x, NUM_SPLINE_PARAMETERS-1-j);
					}
					
					// transform x,y,z back to the original coordinate frame
					gsl_matrix_set(transformedPoint, 0, 0, x);
					gsl_matrix_set(transformedPoint, 1, 0, y);
					gsl_matrix_set(transformedPoint, 2, 0, z);
					gsl_matrix_set(transformedPoint, 3, 0, 1.0);
					gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, iT, transformedPoint, 0.0, point);

					// update the storage for including another point
					if (numCurvePathPointsToAdd > 0) {
						double **curvePathPointsToAddNew = (double **)malloc(sizeof(double *)*(numCurvePathPointsToAdd+1));
						memcpy(curvePathPointsToAddNew, curvePathPointsToAdd, sizeof(double *)*numCurvePathPointsToAdd);
						if (curvePathPointsToAdd) free(curvePathPointsToAdd);
						curvePathPointsToAdd = curvePathPointsToAddNew;
					} else {
						curvePathPointsToAdd = (double **)malloc(sizeof(double *)*1);
					}

					// add the new point that is along the spline to the array
					curvePathPointsToAdd[numCurvePathPointsToAdd] = (double *)malloc(sizeof(double)*3);
					curvePathPointsToAdd[numCurvePathPointsToAdd][0] = gsl_matrix_get(point, 0, 0);
					curvePathPointsToAdd[numCurvePathPointsToAdd][1] = gsl_matrix_get(point, 1, 0);
					curvePathPointsToAdd[numCurvePathPointsToAdd][2] = gsl_matrix_get(point, 2, 0);
					numCurvePathPointsToAdd++;
				}

				// add the last point of the spline to the curvePathPoints array
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, iT, transformedPointEnd, 0.0, point);
				if (numCurvePathPointsToAdd > 0) {
					double **curvePathPointsToAddNew = (double **)malloc(sizeof(double *)*(numCurvePathPointsToAdd+1));
					memcpy(curvePathPointsToAddNew, curvePathPointsToAdd, sizeof(double *)*numCurvePathPointsToAdd);
					if (curvePathPointsToAdd) free(curvePathPointsToAdd);
					curvePathPointsToAdd = curvePathPointsToAddNew;
				} else {
					curvePathPointsToAdd = (double **)malloc(sizeof(double *)*1);
				}
				curvePathPointsToAdd[numCurvePathPointsToAdd] = (double *)malloc(sizeof(double)*3);
				curvePathPointsToAdd[numCurvePathPointsToAdd][0] = gsl_matrix_get(point, 0, 0);
				curvePathPointsToAdd[numCurvePathPointsToAdd][1] = gsl_matrix_get(point, 1, 0);
				curvePathPointsToAdd[numCurvePathPointsToAdd][2] = gsl_matrix_get(point, 2, 0);
				numCurvePathPointsToAdd++;
	
				// save the points along the spline to the big array
				curvePathPoints[i] = curvePathPointsToAdd;
				numCurvePathPointsInEachSegment[i] = numCurvePathPointsToAdd;

				// free some stored memory
				if (A) gsl_matrix_free(A);
				if (x) gsl_matrix_free(x);
				if (b) gsl_matrix_free(b);
				if (iAtAAt) gsl_matrix_free(iAtAAt);
				if (transformedPointStart) gsl_matrix_free(transformedPointStart);
				if (transformedPointEnd) gsl_matrix_free(transformedPointEnd);

			}

			// free up matrices we were using
			if (point) gsl_matrix_free(point);
			if (transformedPoint) gsl_matrix_free(transformedPoint);
			if (T) gsl_matrix_free(T);
			if (AtA) gsl_matrix_free(AtA);
			if (iAtA) gsl_matrix_free(iAtA);
			if (p) gsl_permutation_free(p);
			if (p2) gsl_permutation_free(p2);
			if (W) gsl_matrix_free(W);
			if (iT) gsl_matrix_free(iT);
		}
	}

	// tell the program that we should update the display list for the splines
	updateCurveDrawFlag = true;
	return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::drawCurvePathPoints()                                             *
*                                                                              *
*  This function draws the computed desired path for autonomous control.       *
*                                                                              *
\******************************************************************************/
bool autonomy::drawCurvePathPoints(int glListIndex) {

	if (numSplineSegments == 0 || !numCurvePathPointsInEachSegment) {
		glNewList(glListIndex, GL_COMPILE);
		glEndList();
		updateCurveDrawFlag = false;
		return true;
	}

	int numVertices = 0;
	int numLines = 0;
	for (int i = 0; i < numSplineSegments; i++) {
		if (numCurvePathPointsInEachSegment[i] >= 2) {
			numLines += numCurvePathPointsInEachSegment[i]-1;
		}
	}
	if (numLines < 1) {
		glNewList(glListIndex, GL_COMPILE);
		glEndList();
		updateCurveDrawFlag = false;
		return true;
	}

	numVertices = 2*numLines;

	// allocate space for the vertices and the colors
	GLfloat *vertices = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*3);
	GLfloat *colors = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*4);
	unsigned int *indices = (unsigned int *)malloc(sizeof(unsigned int)*numLines*2);

	// for each spline and for each starting point in the chain of lines
	int numVerticesAdded = 0;
	for (int q = 0; q < numSplineSegments; q++) {
		for (int i = 0; i < numCurvePathPointsInEachSegment[q]-1; i++) {
			
			// check that something weird went wrong
			if (numVerticesAdded >= numVertices) {
				printf("\tError: autonomy, problem adding to vertices list.\n\n");
			}

			// find the vector of the line
			float dx = (GLfloat)curvePathPoints[q][i+1][0] - (GLfloat)curvePathPoints[q][i][0];
			float dy = (GLfloat)curvePathPoints[q][i+1][1] - (GLfloat)curvePathPoints[q][i][1];
			float dz = (GLfloat)curvePathPoints[q][i+1][2] - (GLfloat)curvePathPoints[q][i][2];
			float norm = sqrt(dx*dx + dy*dy + dz*dz);
			dx /= norm; dy /= norm; dz /= norm;

			// first vertex of the line
			vertices[3*numVerticesAdded] = (GLfloat)curvePathPoints[q][i][0] - 0.1f*dx;
			vertices[3*numVerticesAdded+1] = (GLfloat)curvePathPoints[q][i][1] - 0.1f*dy;
			vertices[3*numVerticesAdded+2] = (GLfloat)curvePathPoints[q][i][2] - 0.1f*dz;
			colors[4*numVerticesAdded] = 0.4f; colors[4*numVerticesAdded+1] = 0.4f; colors[4*numVerticesAdded+2] = 1.0f; colors[4*numVerticesAdded+3] = 1.0f;
			numVerticesAdded++;

			// check that something weird went wrong
			if (numVerticesAdded >= numVertices) {
				printf("\tError: autonomy, problem adding to vertices list.\n\n");
			}

			// second vertex of the line
			vertices[3*numVerticesAdded] = (GLfloat)curvePathPoints[q][i+1][0] + 0.1f*dx;
			vertices[3*numVerticesAdded+1] = (GLfloat)curvePathPoints[q][i+1][1] + 0.1f*dy;
			vertices[3*numVerticesAdded+2] = (GLfloat)curvePathPoints[q][i+1][2] + 0.1f*dz;
			colors[4*numVerticesAdded] = 0.4f; colors[4*numVerticesAdded+1] = 0.4f; colors[4*numVerticesAdded+2] = 1.0f; colors[4*numVerticesAdded+3] = 1.0f;
			numVerticesAdded++;
		}
	}

	// there is no sharing of vertices between lines
	for (int i = 0; i < 2*numLines; i++) indices[i] = i;

	// write out the gl commands into the list
	glNewList(glListIndex, GL_COMPILE);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glColorPointer(4, GL_FLOAT, 0, colors);
    glVertexPointer(3, GL_FLOAT, 0, vertices);
	glLineWidth(2.0);
    glDrawElements(GL_LINES, numLines*2, GL_UNSIGNED_INT, indices);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
	glEndList();

	// free allocated memory
	if (vertices) free(vertices);
	if (colors) free(colors);
	if (indices) free(indices);

	// tell snakelibgl that we need to update the display
	updateCurveDrawFlag = false;
	return true;
}


/******************************************************************************\
*                                                                              *
*  autonomy::drawPreCurvePathPoints()                                          *
*                                                                              *
*  This function draws the computed desired path for autonomous control.       *
*                                                                              *
\******************************************************************************/
bool autonomy::drawPreCurvePathPoints(int glListIndex) {

	if (numControlPoints == 0) {
		glNewList(glListIndex, GL_COMPILE);
		glEndList();
		updatePreCurveDrawFlag = false;
		return true;
	}

	int numThetas = 72;
	int numPhis = 36;
	int numVertices = 4*numThetas*numPhis*numControlPoints;
	int numPolygons = numThetas*numPhis*numControlPoints;
	for (int i = 0; i < numSegmentsAddedToPreCurvePathPoints; i++) {
		numPolygons += numThetas*numPhis*numPreCurvePathPointsInEachSegment[i];
		numVertices += 4*numThetas*numPhis*numPreCurvePathPointsInEachSegment[i];
	}
	float radius = 1.0f;

	GLfloat *vertices = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*3);
	GLfloat *normals = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*3);
	GLfloat *colors = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*4);
	unsigned int *indices = (unsigned int *)malloc(sizeof(unsigned int)*numPolygons*4);

	int numVerticesAdded = 0;
	for (int q = 0; q < numSegmentsAddedToPreCurvePathPoints; q++) {
		for (int i = 0; i < numPreCurvePathPointsInEachSegment[q]; i++) {
			for (int thetaIdx = 0; thetaIdx < numThetas; thetaIdx++) {
				for (int phiIdx = 0; phiIdx < numPhis; phiIdx++) {
					for (int j = 0; j <= 1; j++) {
						for (int k = 0; k <= 1; k++) {
							float h = (float)((j || k) && !(j && k));
							if (numVerticesAdded >= numVertices) {
								printf("\tError: autonomy, problem adding to vertices list.\n\n");
							}
							vertices[3*numVerticesAdded] = (GLfloat)preCurvePathPoints[q][i][0] + radius*cos(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
							vertices[3*numVerticesAdded+1] = (GLfloat)preCurvePathPoints[q][i][1] + radius*sin(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
							vertices[3*numVerticesAdded+2] = (GLfloat)preCurvePathPoints[q][i][2] + radius*sin(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
							normals[3*numVerticesAdded] = radius*cos(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
							normals[3*numVerticesAdded+1] = radius*sin(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
							normals[3*numVerticesAdded+2] = radius*sin(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
							colors[4*numVerticesAdded] = 0.0; colors[4*numVerticesAdded+1] = 1.0; colors[4*numVerticesAdded+2] = 0.0; colors[4*numVerticesAdded+3] = 1.0;
							numVerticesAdded++;
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < numControlPoints; i++) {
		for (int thetaIdx = 0; thetaIdx < numThetas; thetaIdx++) {
			for (int phiIdx = 0; phiIdx < numPhis; phiIdx++) {
				for (int j = 0; j <= 1; j++) {
					for (int k = 0; k <= 1; k++) {
						float h = (float)((j || k) && !(j && k));
						vertices[3*numVerticesAdded] = (GLfloat)controlSurfacePoints[i][0] + radius*cos(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
						vertices[3*numVerticesAdded+1] = (GLfloat)controlSurfacePoints[i][1] + radius*sin(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
						vertices[3*numVerticesAdded+2] = (GLfloat)controlSurfacePoints[i][2] + radius*sin(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
						normals[3*numVerticesAdded] = radius*cos(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
						normals[3*numVerticesAdded+1] = radius*sin(((float)thetaIdx+(float)j)*360.0f/(float)numThetas)*cos(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
						normals[3*numVerticesAdded+2] = radius*sin(((float)phiIdx+h-numPhis/2)*180.0f/(float)numPhis);
						colors[4*numVerticesAdded] = 1.0; colors[4*numVerticesAdded+1] = 0.0; colors[4*numVerticesAdded+2] = 0.0; colors[4*numVerticesAdded+3] = 1.0;
						numVerticesAdded++;
					}
				}
			}
		}
	}

	// there is no sharing of vertices between polygons
	for (int i = 0; i < 4*numPolygons; i++) indices[i] = i;

	// write out the gl commands into the list
	glNewList(glListIndex, GL_COMPILE);
	glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
	glNormalPointer(GL_FLOAT, 0, normals);
    glColorPointer(4, GL_FLOAT, 0, colors);
    glVertexPointer(3, GL_FLOAT, 0, vertices);
    glDrawElements(GL_QUADS, numPolygons*4, GL_UNSIGNED_INT, indices);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
	glEndList();

	// free allocated memory
	if (vertices) free(vertices);
	if (normals) free(normals);
	if (colors) free(colors);
	if (indices) free(indices);

	// tell snakelibgl that we need to update the display
	updatePreCurveDrawFlag = false;
	return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::addControlRay()                                                   *
*                                                                              *
*  This function adds a point to help define the path.                         *
*                                                                              *
\******************************************************************************/
bool autonomy::addControlRay(double nearx, double neary, double nearz, double farx, double fary, double farz, int numPolygonMeshes, polygonMesh **polygonMeshes) {

	float line_pt1[3], line_pt2[3];
	float tri_pt1[3], tri_pt2[3], tri_pt3[3], intersection_pt[3], best_intersection_pt[3];

	line_pt1[0] = (float)nearx; line_pt1[1] = (float)neary; line_pt1[2] = (float)nearz;
	line_pt2[0] = (float)farx;  line_pt2[1] = (float)fary;  line_pt2[2] = (float)farz;

	float norm[3];
	float normLength = 0.0;
	float dist = 0.0;
	int numIntersections = 0;
	for (int j = 0; j < numPolygonMeshes; j++) {
		if (!polygonMeshes[j]->getVisibleFlag()) continue;
		if (polygonMeshes[j]->getNumVerticesPerPolygon() != 3) {
			printf("\tError: autonomy, cannot intersect ray with faces other than triangles.\n\n");
		} else {
			for (int k = 0; k < polygonMeshes[j]->getNumPolygons(); k++) {
				
				intersection_pt[0] = 0; intersection_pt[1] = 0; intersection_pt[2] = 0;
				tri_pt1[0] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k));
				tri_pt1[1] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k)+1);
				tri_pt1[2] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k)+2);
				tri_pt2[0] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+1));
				tri_pt2[1] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+1)+1);
				tri_pt2[2] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+1)+2);
				tri_pt3[0] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+2));
				tri_pt3[1] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+2)+1);
				tri_pt3[2] = polygonMeshes[j]->getVertexValue(3*polygonMeshes[j]->getIndex(3*k+2)+2);

				bool returnVal1 = computeLineTriangleIntersection(tri_pt1, tri_pt2, tri_pt3, line_pt1, line_pt2, intersection_pt);
				bool returnVal2 = computeLineTriangleIntersection(tri_pt1, tri_pt3, tri_pt2, line_pt1, line_pt2, intersection_pt);
				if (returnVal1 || returnVal2) {
					float newDist = (line_pt1[0]-intersection_pt[0])*(line_pt1[0]-intersection_pt[0]) + (line_pt1[1]-intersection_pt[1])*(line_pt1[1]-intersection_pt[1]) + (line_pt1[2]-intersection_pt[2])*(line_pt1[2]-intersection_pt[2]); 
					if (newDist < dist || numIntersections == 0) {
						dist = newDist;

						best_intersection_pt[0] = intersection_pt[0];
						best_intersection_pt[1] = intersection_pt[1];
						best_intersection_pt[2] = intersection_pt[2];

						// vector from triangle pt1 to pt2
						float V1x = tri_pt2[0] - tri_pt1[0];
						float V1y = tri_pt2[1] - tri_pt1[1];
						float V1z = tri_pt2[2] - tri_pt1[2];
	
						// vector from triangle pt2 to pt3
						float V2x = tri_pt3[0] - tri_pt2[0];
						float V2y = tri_pt3[1] - tri_pt2[1];
						float V2z = tri_pt3[2] - tri_pt2[2];
	
						// vector normal of triangle
						norm[0] = V1y*V2z-V1z*V2y;
						norm[1] = V1z*V2x-V1x*V2z;
						norm[2] = V1x*V2y-V1y*V2x;
						normLength = sqrt(norm[0]*norm[0] + norm[1]*norm[1] + norm[2]*norm[2]);

					}
					numIntersections++;
				}
			}
		}
	}

	if (numIntersections == 0) {
		printf("\tError: autonomy, clicked point does not intersect with surface.\n\n");
	} else {
		double **controlRayNearPointsNew = (double **)malloc(sizeof(double *)*(numControlPoints + 1));
		double **controlRayFarPointsNew = (double **)malloc(sizeof(double *)*(numControlPoints + 1));
		double **controlSurfacePointsNew = (double **)malloc(sizeof(double *)*(numControlPoints + 1));
		double **controlSurfacePointsPushedOutNew = (double **)malloc(sizeof(double *)*(numControlPoints + 1));
		if (numControlPoints > 0) {
			memcpy_s(controlRayNearPointsNew, sizeof(double *)*numControlPoints, controlRayNearPoints, sizeof(double *)*numControlPoints);
			memcpy_s(controlRayFarPointsNew, sizeof(double *)*numControlPoints, controlRayFarPoints, sizeof(double *)*numControlPoints);
			memcpy_s(controlSurfacePointsNew, sizeof(double *)*numControlPoints, controlSurfacePoints, sizeof(double *)*numControlPoints);
			memcpy_s(controlSurfacePointsPushedOutNew, sizeof(double *)*numControlPoints, controlSurfacePointsPushedOut, sizeof(double *)*numControlPoints);
			free(controlRayNearPoints); free(controlRayFarPoints); free(controlSurfacePoints); free(controlSurfacePointsPushedOut);
		}
		controlRayNearPointsNew[numControlPoints] = (double *)malloc(sizeof(double)*3);
		controlRayFarPointsNew[numControlPoints]  = (double *)malloc(sizeof(double)*3);
		controlSurfacePointsNew[numControlPoints] = (double *)malloc(sizeof(double)*3);
		controlSurfacePointsPushedOutNew[numControlPoints] = (double *)malloc(sizeof(double)*3);
		controlRayNearPointsNew[numControlPoints][0] = nearx;
		controlRayNearPointsNew[numControlPoints][1] = neary;
		controlRayNearPointsNew[numControlPoints][2] = nearz;
		controlRayFarPointsNew[numControlPoints][0] = farx;
		controlRayFarPointsNew[numControlPoints][1] = fary;
		controlRayFarPointsNew[numControlPoints][2] = farz;
		controlSurfacePointsNew[numControlPoints][0] = best_intersection_pt[0];
		controlSurfacePointsNew[numControlPoints][1] = best_intersection_pt[1];
		controlSurfacePointsNew[numControlPoints][2] = best_intersection_pt[2];
		controlSurfacePointsPushedOutNew[numControlPoints][0] = best_intersection_pt[0] + 6.0f*norm[0]/normLength;
		controlSurfacePointsPushedOutNew[numControlPoints][1] = best_intersection_pt[1] + 6.0f*norm[1]/normLength;
		controlSurfacePointsPushedOutNew[numControlPoints][2] = best_intersection_pt[2] + 6.0f*norm[2]/normLength;
		controlRayNearPoints = controlRayNearPointsNew;
		controlRayFarPoints = controlRayFarPointsNew;
		controlSurfacePoints = controlSurfacePointsNew;
		controlSurfacePointsPushedOut = controlSurfacePointsPushedOutNew;
		numControlPoints++;
	}
	updatePreCurveDrawFlag = true;
	updateCurveDrawFlag = true;
	return true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::autonomy()                                                        *
*                                                                              *
*  This function is the constructor for the autonomy class.                    *
*                                                                              *
\******************************************************************************/
autonomy::autonomy() {
	preCurvePathPoints = NULL;
	preCurvePathPointsPushedOut = NULL;
	numPreCurvePathPointsInEachSegment = NULL;
	controlRayNearPoints = NULL;
	controlRayFarPoints = NULL;
	controlSurfacePoints = NULL;
	controlSurfacePointsPushedOut = NULL;
	splineParametersForEachSegment = NULL;
	numCurvePathPointsInEachSegment = NULL;
	curvePathPoints = NULL;
	updatePreCurveDrawFlag = false;
	updateCurveDrawFlag = false;
	visiblePreCurvePathPoints = true;
	visibleSplineCurve = true;
	pickingPointsFlag = false;
	numSegmentsAddedToPreCurvePathPoints = 0;
	numSplineSegments = 0;
	numControlPoints = 0;
	autonomousDrivingFlag = false;
	controlLookAheadPoint = NULL;
	loggingFlag = true;
}

/******************************************************************************\
*                                                                              *
*  autonomy::~autonomy()                                                       *
*                                                                              *
*  This function is the destructor for the autonomy class.                     *
*                                                                              *
\******************************************************************************/
autonomy::~autonomy() {
	clearPath();
}