/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  snakeEstimation.cpp                                                                  |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions dealing with snake shape estimation with a Kalman filter.     |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#include <windows.h>
#include <math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_multimin.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include "../probe_control/config_defaults.h"
#include "snakelibgl.h"
#include "ascension.h"
#include "snakeEstimation.h"

// some constants that we need to define
#define INIT_SPATIAL_UNCERTAINTY					((0.01)*(0.01))
#define INIT_ANGULAR_UNCERTAINTY					((10.0*M_PI/180.0)*(10.0*M_PI/180.0))
#define INIT_ROLL_UNCERTAINTY						((0.001*M_PI/180.0)*(0.001*M_PI/180.0))
#define STEER_SPATIAL_UNCERTAINTY_DISTAL_KALM		((0.1)*(0.1))
#define STEER_ANGULAR_UNCERTAINTY_DISTAL_KALM		((0.1*M_PI/180.0)*(0.1*M_PI/180.0))
#define STEER_ANGULAR_UNCERTAINTY_FULL_PHI			((3.0*M_PI/180.0)*(3.0*M_PI/180.0))
#define STEER_ANGULAR_UNCERTAINTY_FULL_THETA		((3.0*M_PI/180.0)*(3.0*M_PI/180.0))
#define LINK_LENGTHS_TURNING						1
#define LINK_LENGTHS_INIT							7
#define ADVANCE_SPATIAL_UNCERTAINTY_DISTAL_KALM		((5.0)*(5.0))
#define ADVANCE_ANGULAR_UNCERTAINTY_DISTAL_KALM		((3.0*M_PI/180.0)*(3.0*M_PI/180.0))
#define RETRACT_SPATIAL_UNCERTAINTY_DISTAL_KALM		((5.0)*(5.0))
#define RETRACT_ANGULAR_UNCERTAINTY_DISTAL_KALM		((3.0*M_PI/180.0)*(3.0*M_PI/180.0))
#define ADVANCE_ANGULAR_UNCERTAINTY_FULL			((3.0*M_PI/180.0)*(3.0*M_PI/180.0))
#define MEASUREMENT_SPATIAL_UNCERTAINTY				((0.4)*(0.4))
#define MEASUREMENT_ANGULAR_UNCERTAINTY				((2.0*M_PI/180.0)*(2.0*M_PI/180.0))
#define INIT_PHI_UNCERTAINTY_FULL					((0.001*M_PI/180.0)*(0.001*M_PI/180.0))
#define INIT_THETA_UNCERTAINTY_FULL					((0.001*M_PI/180.0)*(0.001*M_PI/180.0))
#define MAX_NUM_FILTER_ITERS						1000


/******************************************************************************\
*                                                                              *
*  snakeEstimation::logData()                                                  *
*                                                                              *
*  This function logs the important data.                                      *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::logData(FILE *fptr, double timeStamp, int dataLabel) {
	if (loggingFlag) {
		fprintf_s(fptr, "%d %lf snakeEstimation", dataLabel, timeStamp);
		if (!isInitializedSnakeEstimation()) {
			fprintf_s(fptr, " numFullLinks 0");
		} else {
			fprintf_s(fptr, " numFullLinks %d", (((int)X_full_kalm->size1)-6)/2+1);
			fprintf_s(fptr, " full_kalm_updating %d", (int)fullKalmUpdating); 
			fprintf_s(fptr, " X_full_kalm");
			for (int i = 0; i < (int)X_full_kalm->size1; i++) fprintf_s(fptr, " %lf", gsl_matrix_get(X_full_kalm,i,0));
			fprintf_s(fptr, " P_full_kalm");
			for (int i = 0; i < (int)P_full_kalm->size1; i++) {
				for (int j = 0; j < (int)P_full_kalm->size2; j++) {
					fprintf_s(fptr, " %lf", gsl_matrix_get(P_full_kalm,i,j));
				}
			}
			fprintf_s(fptr, " full_iterated_updating %d", (int)fullIteratedUpdating); 
			fprintf_s(fptr, " simplifiedUpdateFullIterated %d", (int)simplifiedUpdateFullIterated); 
			fprintf_s(fptr, " X_full_iterated");
			for (int i = 0; i < (int)X_full_iterated->size1; i++) fprintf_s(fptr, " %lf", gsl_matrix_get(X_full_iterated,i,0));
			fprintf_s(fptr, " P_full_iterated");
			for (int i = 0; i < (int)P_full_iterated->size1; i++) {
				for (int j = 0; j < (int)P_full_iterated->size2; j++) {
					fprintf_s(fptr, " %lf", gsl_matrix_get(P_full_iterated,i,j));
				}
			}
			fprintf_s(fptr, " X_full_dead");
			for (int i = 0; i < (int)X_full_dead->size1; i++) fprintf_s(fptr, " %lf", gsl_matrix_get(X_full_dead,i,0));
			fprintf_s(fptr, " distal_kalm_updating %d", (int)distalKalmUpdating); 
			fprintf_s(fptr, " X_distal_kalm");
			for (int i = 0; i < (int)X_distal_kalm->size1; i++) fprintf_s(fptr, " %lf", gsl_matrix_get(X_distal_kalm,i,0));
			fprintf_s(fptr, " P_distal_kalm");
			for (int i = 0; i < (int)P_distal_kalm->size1; i++) {
				for (int j = 0; j < (int)P_distal_kalm->size2; j++) {
					fprintf_s(fptr, " %lf", gsl_matrix_get(P_distal_kalm,i,j));
				}
			}
			fprintf_s(fptr, " thetaDistalKalm %f phiDistalKalm %f", thetaDistalKalm, phiDistalKalm);
		} fprintf_s(fptr, "\n");
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  getFullTransformationMatrix()                                               *
*                                                                              *
*  This function returns the transformation matrix for one of the links.       *
*                                                                              *
\******************************************************************************/
bool getFullTransformationMatrix(gsl_matrix *X, gsl_matrix *Tin, int linkNum) {

	// safety check that the argument is okay
	if (!Tin || Tin->size1 != 4 || Tin->size2 != 4) {
		printf("\tError: getFullTransformation, input argument Tin is bad.\n\n");	
		return false;
	}
	if (X->size1 < 6) {
		printf("\tError: getFullTransformation, input argument X is bad.\n\n");	
		return false;
	}

	// extract the roll, pitch, and yaw for convenience
	double rz = gsl_matrix_get(X, 3, 0);
	double ry = gsl_matrix_get(X, 4, 0);
	double rx = gsl_matrix_get(X, 5, 0);

	// fill in the appropriate values into the matrix
	gsl_matrix_set(Tin, 0, 0, cos(rz)*cos(ry)); gsl_matrix_set(Tin, 0, 1, -sin(rz)*cos(rx)+cos(rz)*sin(ry)*sin(rx));
	gsl_matrix_set(Tin, 0, 2, sin(rz)*sin(rx)+cos(rz)*sin(ry)*cos(rx)); gsl_matrix_set(Tin, 1, 0, sin(rz)*cos(ry));
	gsl_matrix_set(Tin, 1, 1, cos(rz)*cos(rx)+sin(rz)*sin(ry)*sin(rx)); gsl_matrix_set(Tin, 1, 2, -cos(rz)*sin(rx)+sin(rz)*sin(ry)*cos(rx));
	gsl_matrix_set(Tin, 2, 0, -sin(ry)); gsl_matrix_set(Tin, 2, 1, cos(ry)*sin(rx)); gsl_matrix_set(Tin, 2, 2, cos(ry)*cos(rx));
	gsl_matrix_set(Tin, 0, 3, gsl_matrix_get(X, 0, 0)); gsl_matrix_set(Tin, 1, 3, gsl_matrix_get(X, 1, 0));
	gsl_matrix_set(Tin, 2, 3, gsl_matrix_get(X, 2, 0)); gsl_matrix_set(Tin, 3, 3, 1.0);
	gsl_matrix_set(Tin, 3, 0, 0.0); gsl_matrix_set(Tin, 3, 1, 0.0); gsl_matrix_set(Tin, 3, 2, 0.0);

	// advance and then transform for each link
	gsl_matrix *TapplyPhiTheta = gsl_matrix_calloc(4,4);
	gsl_matrix *Ttemp = gsl_matrix_alloc(4,4);
	for (int i = 0; i < linkNum; i++) {

		// extract the phi and theta values
		double phi = gsl_matrix_get(X, 2*i+6, 0);
		double theta = gsl_matrix_get(X, 2*i+7, 0);

		// compute the post-multiplied matrix
		gsl_matrix_set(TapplyPhiTheta, 0, 0,  cos(phi));
		gsl_matrix_set(TapplyPhiTheta, 0, 1, -sin(phi)*cos(theta-M_PI/2.0));
		gsl_matrix_set(TapplyPhiTheta, 0, 2, -sin(phi)*sin(theta-M_PI/2.0));
		gsl_matrix_set(TapplyPhiTheta, 1, 0,  sin(phi)*cos(theta-M_PI/2.0));
		gsl_matrix_set(TapplyPhiTheta, 1, 1,  cos(phi)*cos(theta-M_PI/2.0)*cos(theta-M_PI/2.0)+sin(theta-M_PI/2.0)*sin(theta-M_PI/2.0));
		gsl_matrix_set(TapplyPhiTheta, 1, 2,  cos(phi)*cos(theta-M_PI/2.0)*sin(theta-M_PI/2.0)-sin(theta-M_PI/2.0)*cos(theta-M_PI/2.0));
		gsl_matrix_set(TapplyPhiTheta, 2, 0,  sin(phi)*sin(theta-M_PI/2.0));
		gsl_matrix_set(TapplyPhiTheta, 2, 1,  cos(phi)*sin(theta-M_PI/2.0)*cos(theta-M_PI/2.0)-cos(theta-M_PI/2.0)*sin(theta-M_PI/2.0));
		gsl_matrix_set(TapplyPhiTheta, 2, 2,  cos(phi)*sin(theta-M_PI/2.0)*sin(theta-M_PI/2.0)+cos(theta-M_PI/2.0)*cos(theta-M_PI/2.0));
		gsl_matrix_set(TapplyPhiTheta, 3, 3,  1.0);

		// transform the transformation matrix by phi and theta
		gsl_matrix_memcpy(Ttemp, Tin);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Ttemp, TapplyPhiTheta, 0.0, Tin);

		// calculate the yaw and pitch values from the transformation matrix
		rz = atan2(gsl_matrix_get(Tin, 1, 0), gsl_matrix_get(Tin, 0, 0));
		ry = atan2(-gsl_matrix_get(Tin, 2, 0), sqrt(gsl_matrix_get(Tin, 2, 1)*gsl_matrix_get(Tin, 2, 1) + gsl_matrix_get(Tin, 2, 2)*gsl_matrix_get(Tin, 2, 2)));

		// move forward the position by SNAKELIB_LINK_LENGTH, keep the orientation the same
		gsl_matrix_set(Tin, 0, 3, gsl_matrix_get(Tin, 0, 3) + cos(rz)*cos(ry)*SNAKELIB_LINK_LENGTH);
		gsl_matrix_set(Tin, 1, 3, gsl_matrix_get(Tin, 1, 3) + sin(rz)*cos(ry)*SNAKELIB_LINK_LENGTH);
		gsl_matrix_set(Tin, 2, 3, gsl_matrix_get(Tin, 2, 3) - sin(ry)*SNAKELIB_LINK_LENGTH);

	}

	// delete leftover memory
	if (TapplyPhiTheta) gsl_matrix_free(TapplyPhiTheta);
	if (Ttemp) gsl_matrix_free(Ttemp);

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setLoggingFlag()                                           *
*                                                                              *
*  This function sets the logging flag.                                        *
*                                                                              *
\******************************************************************************/
void snakeEstimation::setLoggingFlag(bool val) {
	loggingFlag = val;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::snakeEstimation()                                          *
*                                                                              *
*  This function is the constructor for the snakeEstimation class.             *
*                                                                              *
\******************************************************************************/
snakeEstimation::snakeEstimation() {

	alphaDistalKalm = 0.3f; visibleDistalKalm = false;
	X_distal_kalm = NULL; P_distal_kalm = NULL;
	thetaDistalKalm = 0.0; phiDistalKalm = 0.0;
	distalKalmUpdating = true;

	alphaFullIterated = 0.3f; visibleFullIterated = false;
	X_full_iterated = NULL; P_full_iterated = NULL;
	fullIteratedUpdating = true;
	simplifiedUpdateFullIterated = true;

	alphaFullKalm = 0.3f; visibleFullKalm = false;
	X_full_kalm = NULL; P_full_kalm = NULL;
	fullKalmUpdating = true;

	alphaFullDead = 0.3f; visibleFullDead = false;
	X_full_dead = NULL;

	loggingFlag = true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::~snakeEstimation()                                         *
*                                                                              *
*  This function is the destructor for the snakeEstimation class.              *
*                                                                              *
\******************************************************************************/
snakeEstimation::~snakeEstimation() {
	destroyDistalKalm();
	destroyFullKalm();
	destroyFullIterated();
	destroyFullDead();
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::getDistalTransformationMatrix()                            *
*                                                                              *
*  This function returns the transformation associated with X_distal_kalm.     *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::getDistalTransformationMatrix(gsl_matrix *Tin) {

	// safety check that the argument is okay
	if (!Tin || Tin->size1 != 4 || Tin->size2 != 4) {
		printf("\tError: getDistalTransformation, input argument Tin is bad.\n\n");	
		return false;
	}

	// extract the roll, pitch, and yaw for convenience
	double rz = gsl_matrix_get(X_distal_kalm, 3, 0);
	double ry = gsl_matrix_get(X_distal_kalm, 4, 0);
	double rx = gsl_matrix_get(X_distal_kalm, 5, 0);

	// fill in the appropriate values into the matrix
	gsl_matrix_set(Tin, 0, 0, cos(rz)*cos(ry)); gsl_matrix_set(Tin, 0, 1, -sin(rz)*cos(rx)+cos(rz)*sin(ry)*sin(rx));
	gsl_matrix_set(Tin, 0, 2, sin(rz)*sin(rx)+cos(rz)*sin(ry)*cos(rx)); gsl_matrix_set(Tin, 1, 0, sin(rz)*cos(ry));
	gsl_matrix_set(Tin, 1, 1, cos(rz)*cos(rx)+sin(rz)*sin(ry)*sin(rx)); gsl_matrix_set(Tin, 1, 2, -cos(rz)*sin(rx)+sin(rz)*sin(ry)*cos(rx));
	gsl_matrix_set(Tin, 2, 0, -sin(ry)); gsl_matrix_set(Tin, 2, 1, cos(ry)*sin(rx)); gsl_matrix_set(Tin, 2, 2, cos(ry)*cos(rx));
	gsl_matrix_set(Tin, 0, 3, gsl_matrix_get(X_distal_kalm, 0, 0)); gsl_matrix_set(Tin, 1, 3, gsl_matrix_get(X_distal_kalm, 1, 0));
	gsl_matrix_set(Tin, 2, 3, gsl_matrix_get(X_distal_kalm, 2, 0)); gsl_matrix_set(Tin, 3, 3, 1.0);
	gsl_matrix_set(Tin, 3, 0, 0.0); gsl_matrix_set(Tin, 3, 1, 0.0); gsl_matrix_set(Tin, 3, 2, 0.0);

	// return true if everything went well
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::isInitializedSnakeEstimation()                             *
*                                                                              *
*  This function states whether snake estimation is initialized or not.        *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::isInitializedSnakeEstimation() {
	if (X_distal_kalm && X_full_kalm && X_full_dead && X_full_iterated) {
		return true;
	} else {
		return false;
	}
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setUpdatingDistalKalm()                                    *
*                                                                              *
*  This function sets the updating flag for the distal kalm estimate.          *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setUpdatingDistalKalm(bool val) {
	distalKalmUpdating = val;	
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setSimplifiedUpdateFullIterated()                          *
*                                                                              *
*  This function sets the simplified update flag for the full iterated est.    *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setSimplifiedUpdateFullIterated(bool val) {
	simplifiedUpdateFullIterated = val;	
	return true;
}
/******************************************************************************\
*                                                                              *
*  snakeEstimation::setUpdatingFullKalm()                                      *
*                                                                              *
*  This function sets the updating flag for the full kalm estimate.            *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setUpdatingFullKalm(bool val) {
	fullKalmUpdating = val;	
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setUpdatingFullIterated()                                  *
*                                                                              *
*  This function sets the updating flag for the full iterated estimate.        *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setUpdatingFullIterated(bool val) {
	fullIteratedUpdating = val;	
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setVisibleFullIterated()                                   *
*                                                                              *
*  This function sets the visible flag for the full iterated estimate.         *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setVisibleFullIterated(bool val) {
	visibleFullIterated = val;	
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setVisibleFullKalm()                                       *
*                                                                              *
*  This function sets the visible flag for the full kalm estimate.             *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setVisibleFullKalm(bool val) {
	visibleFullKalm = val;	
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setVisibleFullDead()                                       *
*                                                                              *
*  This function sets the visible flag for the full dead estimate.             *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setVisibleFullDead(bool val) {
	visibleFullDead = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setVisibleDistalKalm()                                     *
*                                                                              *
*  This function sets the visible flag for the distal kalm estimate.           *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setVisibleDistalKalm(bool val) {
	visibleDistalKalm = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setAlphaFullIterated()                                     *
*                                                                              *
*  This function sets the transparency alpha value for the iterated estimate.  *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setAlphaFullIterated(float val) {
	if (val < 0.0) return false;
	if (val > 1.0) return false;
	alphaFullIterated = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setAlphaFullKalm()                                         *
*                                                                              *
*  This function sets the transparency alpha value for the kalm estimate.      *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setAlphaFullKalm(float val) {
	if (val < 0.0) return false;
	if (val > 1.0) return false;
	alphaFullKalm = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setAlphaFullDead()                                         *
*                                                                              *
*  This function sets the transparency alpha value for the dead estimate.      *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setAlphaFullDead(float val) {
	if (val < 0.0) return false;
	if (val > 1.0) return false;
	alphaFullDead = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::setAlphaDistal()                                           *
*                                                                              *
*  This function sets the transparency alpha value for the distal estimate.    *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::setAlphaDistalKalm(float val) {
	if (val < 0.0) return false;
	if (val > 1.0) return false;
	alphaDistalKalm = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::drawFullIterated()                                         *
*                                                                              *
*  This function draws the full snake shape iterated estimate.                 *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::drawFullIterated(gsl_matrix *Ttransform) {

	if (visibleFullIterated && isInitializedSnakeEstimation()) {
		gsl_matrix *Tpose = gsl_matrix_alloc(4,4);
		gsl_matrix *TposeTransformed = gsl_matrix_alloc(4,4);

		int numLinks = (X_full_iterated->size1-6)/2 + 1;
		if (numLinks <= 0) {
			printf("\tError: snakeEstimation, drawFullIterated, compute number of links seems wrong.\n\n");
			return false;
		}
		for (int i = 0; i < numLinks; i++) {
			if (!getFullTransformationMatrix(X_full_iterated, Tpose, i)) {
				printf("\tError: snakeEstimation, drawFullIterated, couldn't grab full iterated transformation matrix.\n\n");
				return false;
			}
			if (Ttransform) {
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Ttransform, Tpose, 0.0, TposeTransformed);	
			} else {
				gsl_matrix_memcpy(TposeTransformed, Tpose);
			}
		
			float colorTop[3] = {0.5f, 0.5f, 0.5f};
			float colorBottom[3] = {0.8f, 0.8f, 0.8f};
			drawLink(TposeTransformed, colorTop, colorBottom, alphaFullIterated, SNAKELIB_LINK_RADIUS, SNAKELIB_LINK_LENGTH);
		
		}
		if (Tpose) gsl_matrix_free(Tpose);
		if (TposeTransformed) gsl_matrix_free(TposeTransformed);
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::drawFullKalm()                                             *
*                                                                              *
*  This function draws the full snake shape kalm estimate.                     *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::drawFullKalm(gsl_matrix *Ttransform) {

	if (visibleFullKalm && isInitializedSnakeEstimation()) {
		gsl_matrix *Tpose = gsl_matrix_alloc(4,4);
		gsl_matrix *TposeTransformed = gsl_matrix_alloc(4,4);

		int numLinks = (X_full_kalm->size1-6)/2 + 1;
		if (numLinks <= 0) {
			printf("\tError: snakeEstimation, drawFullKalm, compute number of links seems wrong.\n\n");
			return false;
		}
		for (int i = 0; i < numLinks; i++) {
			if (!getFullTransformationMatrix(X_full_kalm, Tpose, i)) {
				printf("\tError: snakeEstimation, drawFullKalm, couldn't grab full kalm transformation matrix.\n\n");
				return false;
			}
			if (Ttransform) {
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Ttransform, Tpose, 0.0, TposeTransformed);	
			} else {
				gsl_matrix_memcpy(TposeTransformed, Tpose);
			}
		
			float colorTop[3] = {0.0f, 0.75f, 0.75f};
			float colorBottom[3] = {0.0f, 1.0f, 1.0f};
			drawLink(TposeTransformed, colorTop, colorBottom, alphaFullKalm, SNAKELIB_LINK_RADIUS, SNAKELIB_LINK_LENGTH);
		
		}
		if (Tpose) gsl_matrix_free(Tpose);
		if (TposeTransformed) gsl_matrix_free(TposeTransformed);
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::drawFullDead()                                             *
*                                                                              *
*  This function draws the full snake shape dead estimate.                     *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::drawFullDead(gsl_matrix *Ttransform) {

	if (visibleFullDead && isInitializedSnakeEstimation()) {
		gsl_matrix *Tpose = gsl_matrix_alloc(4,4);
		gsl_matrix *TposeTransformed = gsl_matrix_alloc(4,4);

		int numLinks = (X_full_dead->size1-6)/2 + 1;
		if (numLinks <= 0) {
			printf("\tError: snakeEstimation, drawFullDead, compute number of links seems wrong.\n\n");
			return false;
		}
		for (int i = 0; i < numLinks; i++) {
			if (!getFullTransformationMatrix(X_full_dead, Tpose, i)) {
				printf("\tError: snakeEstimation, drawFullDead, couldn't grab full dead transformation matrix.\n\n");
				return false;
			}
			if (Ttransform) {
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Ttransform, Tpose, 0.0, TposeTransformed);	
			} else {
				gsl_matrix_memcpy(TposeTransformed, Tpose);
			}
		
			float colorTop[3] = {0.75f, 0.0f, 0.75f};
			float colorBottom[3] = {1.0f, 0.0f, 1.0f};
			drawLink(TposeTransformed, colorTop, colorBottom, alphaFullDead, SNAKELIB_LINK_RADIUS, SNAKELIB_LINK_LENGTH);

		}
		if (Tpose) gsl_matrix_free(Tpose);
		if (TposeTransformed) gsl_matrix_free(TposeTransformed);
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::drawDistalKalm()                                           *
*                                                                              *
*  This function draws the distal kalm estimate.                               *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::drawDistalKalm(gsl_matrix *Ttransform) {

	if (visibleDistalKalm && isInitializedSnakeEstimation()) {
		gsl_matrix *Tpose = gsl_matrix_alloc(4,4);
		gsl_matrix *TposeTransformed = gsl_matrix_alloc(4,4);

		if (!getDistalTransformationMatrix(Tpose)) {
			printf("\tError: snakeEstimation, drawDistalKalm, couldn't grab distal kalm transformation matrix.\n\n");
			return false;
		}

		if (Ttransform) {
			gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Ttransform, Tpose, 0.0, TposeTransformed);	
		} else {
			gsl_matrix_memcpy(TposeTransformed, Tpose);
		}
		
		float colorTop[3] = {0.0f, 0.75f, 0.0f};
		float colorBottom[3] = {0.0f, 1.0f, 0.0f};
		drawLink(TposeTransformed, colorTop, colorBottom, alphaDistalKalm, SNAKELIB_LINK_RADIUS, SNAKELIB_LINK_LENGTH);

		if (Tpose) gsl_matrix_free(Tpose);
		if (TposeTransformed) gsl_matrix_free(TposeTransformed);
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::initializeFullIterated()                                   *
*                                                                              *
*  This initializes the estimation of the full snake shape iterated filter.    *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::initializeFullIterated(gsl_matrix *trackerPose) {

	// some safety checks
	if (!trackerPose || trackerPose->size1 != 4 || trackerPose->size2 != 4) {
		printf("\tError: initialize full iterated, something wrong with trackerPose argument.\n\n");
		return false;
	}

	// make sure to clear away old matrices to start with
	if (X_full_iterated) free(X_full_iterated);
	if (P_full_iterated) free(P_full_iterated);
	X_full_iterated = gsl_matrix_calloc(6+(LINK_LENGTHS_INIT-1)*2,1);
	P_full_iterated = gsl_matrix_calloc(6+(LINK_LENGTHS_INIT-1)*2,6+(LINK_LENGTHS_INIT-1)*2);

	// calculate the roll and pitch values from the transformation matrix
	double rz = atan2(gsl_matrix_get(trackerPose, 1, 0),gsl_matrix_get(trackerPose, 0, 0));
	double ry = atan2(-gsl_matrix_get(trackerPose, 2, 0),sqrt(gsl_matrix_get(trackerPose, 2, 1)*gsl_matrix_get(trackerPose, 2, 1) + gsl_matrix_get(trackerPose, 2, 2)*gsl_matrix_get(trackerPose, 2, 2)));

	// set the necessary X and P values
	for (int i = 0; i < 3; i++) {
		gsl_matrix_set(X_full_iterated, i, 0, gsl_matrix_get(trackerPose, i, 3));
		gsl_matrix_set(P_full_iterated, i, i, INIT_SPATIAL_UNCERTAINTY);
	}
	gsl_matrix_set(X_full_iterated, 3, 0, rz);
	gsl_matrix_set(X_full_iterated, 4, 0, ry);
	gsl_matrix_set(X_full_iterated, 5, 0, 0.0);
	gsl_matrix_set(P_full_iterated, 3, 3, INIT_ANGULAR_UNCERTAINTY);
	gsl_matrix_set(P_full_iterated, 4, 4, INIT_ANGULAR_UNCERTAINTY);
	gsl_matrix_set(P_full_iterated, 5, 5, INIT_ROLL_UNCERTAINTY);

	// move back the position with LINK_LENGTHS_INIT, keep the orientation the same
	gsl_matrix_set(X_full_iterated, 0, 0, gsl_matrix_get(X_full_iterated, 0, 0) - cos(rz)*cos(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_full_iterated, 1, 0, gsl_matrix_get(X_full_iterated, 1, 0) - sin(rz)*cos(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_full_iterated, 2, 0, gsl_matrix_get(X_full_iterated, 2, 0) + sin(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);

	for (int i = 0; i < LINK_LENGTHS_INIT-1; i++) {
		gsl_matrix_set(P_full_iterated, 6+2*i, 6+2*i, INIT_PHI_UNCERTAINTY_FULL);
		gsl_matrix_set(P_full_iterated, 6+2*i+1, 6+2*i+1, INIT_THETA_UNCERTAINTY_FULL);
	}

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::initializeFullKalm()                                       *
*                                                                              *
*  This initializes the estimation of the full snake shape Kalman filter.      *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::initializeFullKalm(gsl_matrix *trackerPose) {

	// some safety checks
	if (!trackerPose || trackerPose->size1 != 4 || trackerPose->size2 != 4) {
		printf("\tError: initialize full kalm, something wrong with trackerPose argument.\n\n");
		return false;
	}

	// make sure to clear away old matrices to start with
	if (X_full_kalm) free(X_full_kalm);
	if (P_full_kalm) free(P_full_kalm);
	X_full_kalm = gsl_matrix_calloc(6+(LINK_LENGTHS_INIT-1)*2,1);
	P_full_kalm = gsl_matrix_calloc(6+(LINK_LENGTHS_INIT-1)*2,6+(LINK_LENGTHS_INIT-1)*2);

	// calculate the roll and pitch values from the transformation matrix
	double rz = atan2(gsl_matrix_get(trackerPose, 1, 0),gsl_matrix_get(trackerPose, 0, 0));
	double ry = atan2(-gsl_matrix_get(trackerPose, 2, 0),sqrt(gsl_matrix_get(trackerPose, 2, 1)*gsl_matrix_get(trackerPose, 2, 1) + gsl_matrix_get(trackerPose, 2, 2)*gsl_matrix_get(trackerPose, 2, 2)));

	// set the necessary X and P values
	for (int i = 0; i < 3; i++) {
		gsl_matrix_set(X_full_kalm, i, 0, gsl_matrix_get(trackerPose, i, 3));
		gsl_matrix_set(P_full_kalm, i, i, INIT_SPATIAL_UNCERTAINTY);
	}
	gsl_matrix_set(X_full_kalm, 3, 0, rz);
	gsl_matrix_set(X_full_kalm, 4, 0, ry);
	gsl_matrix_set(X_full_kalm, 5, 0, 0.0);
	gsl_matrix_set(P_full_kalm, 3, 3, INIT_ANGULAR_UNCERTAINTY);
	gsl_matrix_set(P_full_kalm, 4, 4, INIT_ANGULAR_UNCERTAINTY);
	gsl_matrix_set(P_full_kalm, 5, 5, INIT_ROLL_UNCERTAINTY);

	// move back the position with LINK_LENGTHS_INIT, keep the orientation the same
	gsl_matrix_set(X_full_kalm, 0, 0, gsl_matrix_get(X_full_kalm, 0, 0) - cos(rz)*cos(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_full_kalm, 1, 0, gsl_matrix_get(X_full_kalm, 1, 0) - sin(rz)*cos(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_full_kalm, 2, 0, gsl_matrix_get(X_full_kalm, 2, 0) + sin(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);

	for (int i = 0; i < LINK_LENGTHS_INIT-1; i++) {
		gsl_matrix_set(P_full_kalm, 6+2*i, 6+2*i, INIT_PHI_UNCERTAINTY_FULL);
		gsl_matrix_set(P_full_kalm, 6+2*i+1, 6+2*i+1, INIT_THETA_UNCERTAINTY_FULL);
	}

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::initializeFullDead()                                       *
*                                                                              *
*  This initializes the estimation of the full snake shape dead reckoning.     *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::initializeFullDead(gsl_matrix *trackerPose) {

	// some safety checks
	if (!trackerPose || trackerPose->size1 != 4 || trackerPose->size2 != 4) {
		printf("\tError: initialize full dead, something wrong with trackerPose argument.\n\n");
		return false;
	}

	// make sure to clear away old matrices to start with
	if (X_full_dead) free(X_full_dead);
	X_full_dead = gsl_matrix_calloc(6+(LINK_LENGTHS_INIT-1)*2,1);

	// calculate the roll and pitch values from the transformation matrix
	double rz = atan2(gsl_matrix_get(trackerPose, 1, 0),gsl_matrix_get(trackerPose, 0, 0));
	double ry = atan2(-gsl_matrix_get(trackerPose, 2, 0),sqrt(gsl_matrix_get(trackerPose, 2, 1)*gsl_matrix_get(trackerPose, 2, 1) + gsl_matrix_get(trackerPose, 2, 2)*gsl_matrix_get(trackerPose, 2, 2)));

	// set the necessary X and P values
	for (int i = 0; i < 3; i++) {
		gsl_matrix_set(X_full_dead, i, 0, gsl_matrix_get(trackerPose, i, 3));
	}
	gsl_matrix_set(X_full_dead, 3, 0, rz);
	gsl_matrix_set(X_full_dead, 4, 0, ry);
	gsl_matrix_set(X_full_dead, 5, 0, 0.0);

	// move back the position with LINK_LENGTHS_INIT, keep the orientation the same
	gsl_matrix_set(X_full_dead, 0, 0, gsl_matrix_get(X_full_dead, 0, 0) - cos(rz)*cos(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_full_dead, 1, 0, gsl_matrix_get(X_full_dead, 1, 0) - sin(rz)*cos(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_full_dead, 2, 0, gsl_matrix_get(X_full_dead, 2, 0) + sin(ry)*(LINK_LENGTHS_INIT-1)*SNAKELIB_LINK_LENGTH);

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::initializeDistalKalm()                                     *
*                                                                              *
*  This initializes the estimation of the distal link Kalman filter.           *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::initializeDistalKalm(gsl_matrix *trackerPose) {

	// some safety checks
	if (!trackerPose || trackerPose->size1 != 4 || trackerPose->size2 != 4) {
		printf("\tError: initialize distal kalm, something wrong with trackerPose argument.\n\n");
		return false;
	}

	// make sure to clear away old matrices to start with
	if (X_distal_kalm) free(X_distal_kalm);
	if (P_distal_kalm) free(P_distal_kalm);
	X_distal_kalm = gsl_matrix_alloc(6,1);
	P_distal_kalm = gsl_matrix_calloc(6,6);

	// calculate the roll and pitch values from the transformation matrix
	double rz = atan2(gsl_matrix_get(trackerPose, 1, 0),gsl_matrix_get(trackerPose, 0, 0));
	double ry = atan2(-gsl_matrix_get(trackerPose, 2, 0),sqrt(gsl_matrix_get(trackerPose, 2, 1)*gsl_matrix_get(trackerPose, 2, 1) + gsl_matrix_get(trackerPose, 2, 2)*gsl_matrix_get(trackerPose, 2, 2)));

	// set the necessary X and P values
	for (int i = 0; i < 3; i++) {
		gsl_matrix_set(X_distal_kalm, i, 0, gsl_matrix_get(trackerPose, i, 3));
		gsl_matrix_set(P_distal_kalm, i, i, INIT_SPATIAL_UNCERTAINTY);
	}
	gsl_matrix_set(X_distal_kalm, 3, 0, rz);
	gsl_matrix_set(X_distal_kalm, 4, 0, ry);
	gsl_matrix_set(X_distal_kalm, 5, 0, 0.0);
	gsl_matrix_set(P_distal_kalm, 3, 3, INIT_ANGULAR_UNCERTAINTY);
	gsl_matrix_set(P_distal_kalm, 4, 4, INIT_ANGULAR_UNCERTAINTY);
	gsl_matrix_set(P_distal_kalm, 5, 5, INIT_ROLL_UNCERTAINTY);

	thetaDistalKalm = 0.0;
	phiDistalKalm = 0.0;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::destroyFullDead()                                          *
*                                                                              *
*  Erases and clears the memory associated with the full kalm estimate.        *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::destroyFullDead() {
	if (X_full_dead) gsl_matrix_free(X_full_dead);
	X_full_dead = NULL;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::destroyFullIterated()                                      *
*                                                                              *
*  Erases and clears the memory associated with the full iterated estimate.    *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::destroyFullIterated() {
	if (X_full_iterated) gsl_matrix_free(X_full_iterated);
	X_full_iterated = NULL;
	if (P_full_iterated) gsl_matrix_free(P_full_iterated);
	P_full_iterated = NULL;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::destroyFullKalm()                                          *
*                                                                              *
*  Erases and clears the memory associated with the full kalm estimate.        *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::destroyFullKalm() {
	if (X_full_kalm) gsl_matrix_free(X_full_kalm);
	X_full_kalm = NULL;
	if (P_full_kalm) gsl_matrix_free(P_full_kalm);
	P_full_kalm = NULL;
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::destroyDistalKalm()                                        *
*                                                                              *
*  Erases and clears the memory associated with the distal kalm estimate.      *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::destroyDistalKalm() {
	if (X_distal_kalm) gsl_matrix_free(X_distal_kalm);
	X_distal_kalm = NULL;
	if (P_distal_kalm) gsl_matrix_free(P_distal_kalm);
	P_distal_kalm = NULL;
	thetaDistalKalm = 0.0;
	phiDistalKalm = 0.0;
	return true;
}

gsl_matrix *Ttip_g, *x_g, *x_diff_g, *trackerPose_g, *res_g, *iR_g;
gsl_matrix *resTiR_g, *resTiRres_g, *x_diffTiP_g, *x_diffTiPx_diff_g;
gsl_matrix *iP_full_iterated_g, *X_full_iterated_g;

/******************************************************************************\
*                                                                              *
*  steerCorrectFullCostFunction()                                              *
*                                                                              *
*  Cost function for iterated update for steering correction.                  *
*                                                                              *
\******************************************************************************/
double steerCorrectFullCostFunction(const gsl_vector *v, void *par)
{
	// allocate some matrices
	if (!Ttip_g) Ttip_g = gsl_matrix_alloc(4,4);
	if (!x_g) x_g = gsl_matrix_alloc(v->size,1);
	if (!x_diff_g) x_diff_g = gsl_matrix_alloc(v->size,1);
	if (!trackerPose_g) trackerPose_g = gsl_matrix_alloc(4,4);
	if (!res_g) res_g = gsl_matrix_alloc(5,1);
	if (!iR_g) iR_g = gsl_matrix_calloc(5,5);
	if (!resTiR_g) resTiR_g = gsl_matrix_alloc(1,5);
	if (!resTiRres_g) resTiRres_g = gsl_matrix_alloc(1,1);
	if (!x_diffTiP_g) x_diffTiP_g = gsl_matrix_alloc(1,v->size);
	if (!x_diffTiPx_diff_g) x_diffTiPx_diff_g = gsl_matrix_alloc(1,1);
	if (!iP_full_iterated_g) iP_full_iterated_g = gsl_matrix_alloc(v->size,v->size);
	if (!X_full_iterated_g) X_full_iterated_g = gsl_matrix_alloc(v->size,1);

	// check if anything needs to be resized
	if (x_g->size1 != v->size) { gsl_matrix_free(x_g); x_g = gsl_matrix_alloc(v->size,1); }
	if (x_diff_g->size1 != v->size) { gsl_matrix_free(x_diff_g); x_diff_g = gsl_matrix_alloc(v->size,1); }
	if (x_diffTiP_g->size2 != v->size) { gsl_matrix_free(x_diffTiP_g); x_diffTiP_g = gsl_matrix_alloc(1,v->size); }
	if (iP_full_iterated_g->size1 != v->size) { gsl_matrix_free(iP_full_iterated_g); iP_full_iterated_g = gsl_matrix_alloc(v->size,v->size); }
	if (X_full_iterated_g->size1 != v->size) { gsl_matrix_free(X_full_iterated_g); X_full_iterated_g = gsl_matrix_alloc(v->size,1); }


	// copy over the x and trackerPose vectors from the arguments
	double *params = (double *)par;
	int index = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			gsl_matrix_set(trackerPose_g, i, j, params[index++]);
		}
	}
	for (int i = 0; i < (int)v->size; i++) {
		gsl_matrix_set(X_full_iterated_g, i, 0, params[index++]);
	}
	for (int i = 0; i < (int)v->size; i++) {
		for (int j = 0; j < (int)v->size; j++) {
			gsl_matrix_set(iP_full_iterated_g, i, j, params[index++]);
		}
		gsl_matrix_set(x_g, i, 0, gsl_vector_get(v,i));
		gsl_matrix_set(x_diff_g, i, 0, gsl_vector_get(v,i) - gsl_matrix_get(X_full_iterated_g, i, 0));
	}

	// get the transformation matrix for the tip of the snake
	if (!getFullTransformationMatrix(x_g, Ttip_g, (((int)x_g->size1)-6)/2)) {
		printf("\tError: steerCorrectFullCostFunction, something wrong when grabbing Ttip.\n\n");
		return false;
	}

	// calculate the roll and pitch values from the transformation matrix
	double rz_sensed = atan2(gsl_matrix_get(trackerPose_g, 1, 0),gsl_matrix_get(trackerPose_g, 0, 0));
	double ry_sensed = atan2(-gsl_matrix_get(trackerPose_g, 2, 0),sqrt(gsl_matrix_get(trackerPose_g, 2, 1)*gsl_matrix_get(trackerPose_g, 2, 1) + gsl_matrix_get(trackerPose_g, 2, 2)*gsl_matrix_get(trackerPose_g, 2, 2)));

	// calculate the roll and pitch values from the transformation matrix
	double rz_state = atan2(gsl_matrix_get(Ttip_g, 1, 0), gsl_matrix_get(Ttip_g, 0, 0));
	double ry_state = atan2(-gsl_matrix_get(Ttip_g, 2, 0), sqrt(gsl_matrix_get(Ttip_g, 2, 1)*gsl_matrix_get(Ttip_g, 2, 1) + gsl_matrix_get(Ttip_g, 2, 2)*gsl_matrix_get(Ttip_g, 2, 2)));

	// set the values in the residual vector
	gsl_matrix_set(res_g, 0, 0, gsl_matrix_get(trackerPose_g, 0, 3) - gsl_matrix_get(Ttip_g, 0, 3));
	gsl_matrix_set(res_g, 1, 0, gsl_matrix_get(trackerPose_g, 1, 3) - gsl_matrix_get(Ttip_g, 1, 3));
	gsl_matrix_set(res_g, 2, 0, gsl_matrix_get(trackerPose_g, 2, 3) - gsl_matrix_get(Ttip_g, 2, 3));
	gsl_matrix_set(res_g, 3, 0, rz_sensed - rz_state);
	while (gsl_matrix_get(res_g, 3, 0) < -M_PI) gsl_matrix_set(res_g, 3, 0, gsl_matrix_get(res_g, 3, 0) + 2*M_PI);
	while (gsl_matrix_get(res_g, 3, 0) >  M_PI) gsl_matrix_set(res_g, 3, 0, gsl_matrix_get(res_g, 3, 0) - 2*M_PI);
	gsl_matrix_set(res_g, 4, 0, ry_sensed - ry_state);
	while (gsl_matrix_get(res_g, 4, 0) < -M_PI) gsl_matrix_set(res_g, 4, 0, gsl_matrix_get(res_g, 4, 0) + 2*M_PI);
	while (gsl_matrix_get(res_g, 4, 0) >  M_PI) gsl_matrix_set(res_g, 4, 0, gsl_matrix_get(res_g, 4, 0) - 2*M_PI);

	// fill in the measurement covariance matrix
	for (int i = 0; i < 3; i++) gsl_matrix_set(iR_g, i, i, (1.0/MEASUREMENT_SPATIAL_UNCERTAINTY));
	for (int i = 3; i < 5; i++) gsl_matrix_set(iR_g, i, i, (1.0/MEASUREMENT_ANGULAR_UNCERTAINTY));

	//  multiply the two components of the cost
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, res_g, iR_g, 0.0, resTiR_g);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, resTiR_g, res_g, 0.0, resTiRres_g);
	gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, x_diff_g, iP_full_iterated_g, 0.0, x_diffTiP_g);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, x_diffTiP_g, x_diff_g, 0.0, x_diffTiPx_diff_g);

	// compute the cost
	double cost = gsl_matrix_get(x_diffTiPx_diff_g, 0, 0) + gsl_matrix_get(resTiRres_g, 0, 0);

	// return the cost
	return cost;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::steerCorrectFullIterated()                                 *
*                                                                              *
*  Iterated update for steering, updates with the tracker.                     *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::steerCorrectFullIterated(gsl_matrix *trackerPose) {

	#define PRINT_ITERATION_INFO false

	// some safety checks
	if (!trackerPose || trackerPose->size1 != 4 || trackerPose->size2 != 4) {
		printf("\tError: steerCorrectFullIterated, something wrong with trackerPose argument.\n\n");
		return false;
	}

	// check if we're supposed to update
	if (!fullIteratedUpdating) return false;

	// grab a time stamp
	long long startTime, endTime;
	long long frequency;
	QueryPerformanceCounter((_LARGE_INTEGER*)&startTime);
	QueryPerformanceFrequency((_LARGE_INTEGER*)&frequency);
	double startTimeStamp = (double)startTime/(double)frequency;

	// add the tracker pose and state/covariance to the un-optimized parameters
	int parLength = 16 + (int)X_full_iterated->size1 + (int)(P_full_iterated->size1*P_full_iterated->size2);
	double *par = (double *)malloc(sizeof(double)*parLength);
	int index = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			par[index++] = gsl_matrix_get(trackerPose, i, j);
		}
	}
	for (int i = 0; i < (int)X_full_iterated->size1; i++) {
		par[index++] = gsl_matrix_get(X_full_iterated, i, 0);
	}
	gsl_matrix *work_iP = gsl_matrix_alloc((int)P_full_iterated->size1, (int)P_full_iterated->size2);
	gsl_matrix *iP_full_iterated = gsl_matrix_alloc((int)P_full_iterated->size1, (int)P_full_iterated->size2);
	gsl_permutation *p_iP = gsl_permutation_alloc((int)P_full_iterated->size1);
	int s_iP = 0;
	gsl_matrix_memcpy(work_iP, P_full_iterated);
	gsl_linalg_LU_decomp(work_iP, p_iP, &s_iP);
	gsl_linalg_LU_invert(work_iP, p_iP, iP_full_iterated);
	for (int i = 0; i < (int)iP_full_iterated->size1; i++) {
		for (int j = 0; j < (int)iP_full_iterated->size2; j++) {
			par[index++] = gsl_matrix_get(iP_full_iterated, i, j);
		}
	}
	gsl_matrix_free(work_iP);
	gsl_matrix_free(iP_full_iterated);
	gsl_permutation_free(p_iP);

	// initialize the optimization
	const gsl_multimin_fminimizer_type *myMinimizationType = gsl_multimin_fminimizer_nmsimplex2;
	gsl_multimin_fminimizer *myMinimizer = NULL;
	gsl_vector *myStepSizes = gsl_vector_alloc((int)X_full_iterated->size1);
	gsl_vector *x = gsl_vector_alloc((int)X_full_iterated->size1);
	gsl_multimin_function myFunction;
	myFunction.n = (int)X_full_iterated->size1;
	myFunction.f = steerCorrectFullCostFunction;
	myFunction.params = par;
	
	// fill in the guess and step sizes
	for (int i = 0; i < (int)x->size; i++) {
		gsl_vector_set(x, i, gsl_matrix_get(X_full_iterated, i, 0));
		gsl_vector_set(myStepSizes, i, 1.0);
	}

	// allocate the minimizer and set the info
	myMinimizer = gsl_multimin_fminimizer_alloc(myMinimizationType, (int)X_full_iterated->size1);
	gsl_multimin_fminimizer_set(myMinimizer, &myFunction, x, myStepSizes);

	// start the iteration, only if its set to do the iterated update, otherwise EKF
	if (!simplifiedUpdateFullIterated) {
		int numIters = 0;
		int status = GSL_CONTINUE;
		while (numIters < MAX_NUM_FILTER_ITERS && status == GSL_CONTINUE) {
			numIters++;
			status = gsl_multimin_fminimizer_iterate(myMinimizer);          
			if (status) break;  
			double size = gsl_multimin_fminimizer_size(myMinimizer);
			status = gsl_multimin_test_size(size, 1e-2);
			if (PRINT_ITERATION_INFO) {
				if (status == GSL_SUCCESS) printf ("\tConverged to minimum.\n"); 
				printf ("\t%5d f() = %7.3f size = %.3f\n", numIters, myMinimizer->fval, size);
			}
		}
		if (status == GSL_CONTINUE && PRINT_ITERATION_INFO) {
			printf("\tsteerCorrectFullIterated, reached maximum iteration limit.\n\n");
		}
		for (int i = 0; i < (int)X_full_iterated->size1; i++) {
			gsl_matrix_set(X_full_iterated, i, 0, gsl_vector_get(myMinimizer->x, i));
		}
	}

	// free the minimizer and the state
    gsl_vector_free(x);
    gsl_vector_free(myStepSizes);
    gsl_multimin_fminimizer_free(myMinimizer);
	if (par) free(par);

	// allocate the matrices needed for the Kalman filter
	gsl_matrix *res = gsl_matrix_alloc(5,1);
	gsl_matrix *deltaX = gsl_matrix_alloc((int)X_full_iterated->size1,1);
	gsl_matrix *H = gsl_matrix_calloc(5, (int)X_full_iterated->size1);
	gsl_matrix *HP = gsl_matrix_alloc(5, (int)X_full_iterated->size1);
	gsl_matrix *S = gsl_matrix_calloc(5, 5);
	gsl_matrix *work = gsl_matrix_alloc(5, 5);
	gsl_matrix *iS = gsl_matrix_alloc(5, 5);
	gsl_permutation *p = gsl_permutation_alloc(5);
	gsl_matrix *PHt = gsl_matrix_alloc((int)X_full_iterated->size1, 5);
	gsl_matrix *K = gsl_matrix_alloc((int)X_full_iterated->size1, 5);
	gsl_matrix *KHP = gsl_matrix_alloc((int)X_full_iterated->size1, (int)X_full_iterated->size1);
	gsl_matrix *Ttip = gsl_matrix_alloc(4,4);
	gsl_matrix *TtipDelta = gsl_matrix_alloc(4,4);
	gsl_matrix *X_delta = gsl_matrix_alloc((int)X_full_iterated->size1,1);

	// get the transformation matrix for the tip of the snake
	if (!getFullTransformationMatrix(X_full_iterated, Ttip, (((int)X_full_iterated->size1)-6)/2)) {
		printf("\tError: steerCorrectFullIterated, something wrong when grabbing Ttip.\n\n");
		return false;
	}

	// calculate the roll and pitch values from the transformation matrix
	double rz_sensed = atan2(gsl_matrix_get(trackerPose, 1, 0),gsl_matrix_get(trackerPose, 0, 0));
	double ry_sensed = atan2(-gsl_matrix_get(trackerPose, 2, 0),sqrt(gsl_matrix_get(trackerPose, 2, 1)*gsl_matrix_get(trackerPose, 2, 1) + gsl_matrix_get(trackerPose, 2, 2)*gsl_matrix_get(trackerPose, 2, 2)));

	// calculate the roll and pitch values from the transformation matrix
	double rz_state = atan2(gsl_matrix_get(Ttip, 1, 0), gsl_matrix_get(Ttip, 0, 0));
	double ry_state = atan2(-gsl_matrix_get(Ttip, 2, 0), sqrt(gsl_matrix_get(Ttip, 2, 1)*gsl_matrix_get(Ttip, 2, 1) + gsl_matrix_get(Ttip, 2, 2)*gsl_matrix_get(Ttip, 2, 2)));

	// set the values in the residual vector
	gsl_matrix_set(res, 0, 0, gsl_matrix_get(trackerPose, 0, 3) - gsl_matrix_get(Ttip, 0, 3));
	gsl_matrix_set(res, 1, 0, gsl_matrix_get(trackerPose, 1, 3) - gsl_matrix_get(Ttip, 1, 3));
	gsl_matrix_set(res, 2, 0, gsl_matrix_get(trackerPose, 2, 3) - gsl_matrix_get(Ttip, 2, 3));
	gsl_matrix_set(res, 3, 0, rz_sensed - rz_state);
	while (gsl_matrix_get(res, 3, 0) < -M_PI) gsl_matrix_set(res, 3, 0, gsl_matrix_get(res, 3, 0) + 2*M_PI);
	while (gsl_matrix_get(res, 3, 0) >  M_PI) gsl_matrix_set(res, 3, 0, gsl_matrix_get(res, 3, 0) - 2*M_PI);
	gsl_matrix_set(res, 4, 0, ry_sensed - ry_state);
	while (gsl_matrix_get(res, 4, 0) < -M_PI) gsl_matrix_set(res, 4, 0, gsl_matrix_get(res, 4, 0) + 2*M_PI);
	while (gsl_matrix_get(res, 4, 0) >  M_PI) gsl_matrix_set(res, 4, 0, gsl_matrix_get(res, 4, 0) - 2*M_PI);

	// compute the H matrix
	#define DX (1e-5)
	for (int j = 0; j < (int)H->size2; j++) {
		gsl_matrix_memcpy(X_delta, X_full_iterated);
		gsl_matrix_set(X_delta, j, 0, gsl_matrix_get(X_delta, j, 0) + DX);
		if (!getFullTransformationMatrix(X_delta, TtipDelta, (((int)X_delta->size1)-6)/2)) {
			printf("\tError: steerCorrectFullIterated, something wrong when grabbing TtipDelta.\n\n");
			return false;
		}
		for (int i = 0; i < (int)H->size1; i++) {
			double hElement = 0.0;
			if (i < 3) {
				hElement = gsl_matrix_get(TtipDelta, i, 3) - gsl_matrix_get(Ttip, i, 3);
			} else if (i == 3) {
				hElement = atan2(gsl_matrix_get(TtipDelta, 1, 0), gsl_matrix_get(TtipDelta, 0, 0)) - atan2(gsl_matrix_get(Ttip, 1, 0), gsl_matrix_get(Ttip, 0, 0));
				while (hElement < -M_PI) hElement = hElement + 2.0*M_PI;
				while (hElement >  M_PI) hElement = hElement - 2.0*M_PI;
			} else if (i == 4) {
				hElement = atan2(-gsl_matrix_get(TtipDelta, 2, 0), sqrt(gsl_matrix_get(TtipDelta, 2, 1)*gsl_matrix_get(TtipDelta, 2, 1) + gsl_matrix_get(TtipDelta, 2, 2)*gsl_matrix_get(TtipDelta, 2, 2))) - atan2(-gsl_matrix_get(Ttip, 2, 0), sqrt(gsl_matrix_get(Ttip, 2, 1)*gsl_matrix_get(Ttip, 2, 1) + gsl_matrix_get(Ttip, 2, 2)*gsl_matrix_get(Ttip, 2, 2)));
				while (hElement < -M_PI) hElement = hElement + 2.0*M_PI;
				while (hElement >  M_PI) hElement = hElement - 2.0*M_PI;
			} else {
				printf("\tError: steerCorrectFullIterated, something went wrong computing H.\n\n");
				return false;
			}
			gsl_matrix_set(H, i, j, (hElement/DX));
		}
	}
		
	// compute the innovation matrix
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, H, P_full_iterated, 0.0, HP);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, HP, H, 0.0, S);
	for (int i = 0; i < 3; i++) gsl_matrix_set(S, i, i, gsl_matrix_get(S, i, i) + MEASUREMENT_SPATIAL_UNCERTAINTY);
	for (int i = 3; i < 5; i++) gsl_matrix_set(S, i, i, gsl_matrix_get(S, i, i) + MEASUREMENT_ANGULAR_UNCERTAINTY);

	// invert the innovation matrix
	int s = 0;
	gsl_matrix_memcpy(work, S);
	gsl_linalg_LU_decomp(work, p, &s);
	gsl_linalg_LU_invert(work, p, iS);

	// compute the Kalman gain
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, P_full_iterated, H, 0.0, PHt);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, PHt, iS, 0.0, K);

	// compute the updated state as the EKF when simplified
	if (simplifiedUpdateFullIterated) {
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, res, 0.0, deltaX);
		for (int i = 0; i < (int)X_full_iterated->size1; i++) gsl_matrix_set(X_full_iterated, i, 0, gsl_matrix_get(X_full_iterated, i, 0) + gsl_matrix_get(deltaX, i, 0));
	}

	// compute the updated covariance matrix
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, HP, 0.0, KHP);
	for (int i = 0; i < (int)X_full_iterated->size1; i++) {
		for (int j = i; j < (int)X_full_iterated->size1; j++) {
			double val = gsl_matrix_get(P_full_iterated, i, j) - gsl_matrix_get(KHP, i, j);
			gsl_matrix_set(P_full_iterated, i, j, val);
			gsl_matrix_set(P_full_iterated, j, i, val);
		}
	}

	// delete all of the extra allocated space
	if (res) gsl_matrix_free(res);
	if (deltaX) gsl_matrix_free(deltaX);
	if (H) gsl_matrix_free(H);
	if (HP) gsl_matrix_free(HP);
	if (S) gsl_matrix_free(S);
	if (work) gsl_matrix_free(work);
	if (iS) gsl_matrix_free(iS);
	if (p) gsl_permutation_free(p);
	if (PHt) gsl_matrix_free(PHt);
	if (K) gsl_matrix_free(K);
	if (KHP) gsl_matrix_free(KHP);
	if (Ttip) gsl_matrix_free(Ttip);
	if (TtipDelta) gsl_matrix_free(TtipDelta);
	if (X_delta) gsl_matrix_free(X_delta);

	QueryPerformanceCounter((_LARGE_INTEGER*)&endTime);
	QueryPerformanceFrequency((_LARGE_INTEGER*)&frequency);
	double endTimeStamp = (double)endTime/(double)frequency;
	if (PRINT_ITERATION_INFO)
		printf("iterated update took: %.10lf secs\n", endTimeStamp - startTimeStamp);

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::steerCorrectFullKalm()                                     *
*                                                                              *
*  Kalman update for steering, updates with the tracker.                       *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::steerCorrectFullKalm(gsl_matrix *trackerPose) {

	// some safety checks
	if (!trackerPose || trackerPose->size1 != 4 || trackerPose->size2 != 4) {
		printf("\tError: steerCorrectFullKalm, something wrong with trackerPose argument.\n\n");
		return false;
	}

	// check if we're supposed to update
	if (!fullKalmUpdating) return false;

	// allocate the matrices needed for the Kalman filter
	gsl_matrix *res = gsl_matrix_alloc(5,1);
	gsl_matrix *deltaX = gsl_matrix_alloc((int)X_full_kalm->size1,1);
	gsl_matrix *H = gsl_matrix_calloc(5, (int)X_full_kalm->size1);
	gsl_matrix *HP = gsl_matrix_alloc(5, (int)X_full_kalm->size1);
	gsl_matrix *S = gsl_matrix_calloc(5, 5);
	gsl_matrix *work = gsl_matrix_alloc(5, 5);
	gsl_matrix *iS = gsl_matrix_alloc(5, 5);
	gsl_permutation *p = gsl_permutation_alloc(5);
	gsl_matrix *PHt = gsl_matrix_alloc((int)X_full_kalm->size1, 5);
	gsl_matrix *K = gsl_matrix_alloc((int)X_full_kalm->size1, 5);
	gsl_matrix *KHP = gsl_matrix_alloc((int)X_full_kalm->size1, (int)X_full_kalm->size1);
	gsl_matrix *Ttip = gsl_matrix_alloc(4,4);
	gsl_matrix *TtipDelta = gsl_matrix_alloc(4,4);
	gsl_matrix *X_delta = gsl_matrix_alloc((int)X_full_kalm->size1,1);

	// get the transformation matrix for the tip of the snake
	if (!getFullTransformationMatrix(X_full_kalm, Ttip, (((int)X_full_kalm->size1)-6)/2)) {
		printf("\tError: steerCorrectFullKalm, something wrong when grabbing Ttip.\n\n");
		return false;
	}

	// calculate the roll and pitch values from the transformation matrix
	double rz_state = atan2(gsl_matrix_get(Ttip, 1, 0), gsl_matrix_get(Ttip, 0, 0));
	double ry_state = atan2(-gsl_matrix_get(Ttip, 2, 0), sqrt(gsl_matrix_get(Ttip, 2, 1)*gsl_matrix_get(Ttip, 2, 1) + gsl_matrix_get(Ttip, 2, 2)*gsl_matrix_get(Ttip, 2, 2)));

	// calculate the roll and pitch values from the transformation matrix
	double rz_sensed = atan2(gsl_matrix_get(trackerPose, 1, 0),gsl_matrix_get(trackerPose, 0, 0));
	double ry_sensed = atan2(-gsl_matrix_get(trackerPose, 2, 0),sqrt(gsl_matrix_get(trackerPose, 2, 1)*gsl_matrix_get(trackerPose, 2, 1) + gsl_matrix_get(trackerPose, 2, 2)*gsl_matrix_get(trackerPose, 2, 2)));

	// set the values in the residual vector
	gsl_matrix_set(res, 0, 0, gsl_matrix_get(trackerPose, 0, 3) - gsl_matrix_get(Ttip, 0, 3));
	gsl_matrix_set(res, 1, 0, gsl_matrix_get(trackerPose, 1, 3) - gsl_matrix_get(Ttip, 1, 3));
	gsl_matrix_set(res, 2, 0, gsl_matrix_get(trackerPose, 2, 3) - gsl_matrix_get(Ttip, 2, 3));
	gsl_matrix_set(res, 3, 0, rz_sensed - rz_state);
	while (gsl_matrix_get(res, 3, 0) < -M_PI) gsl_matrix_set(res, 3, 0, gsl_matrix_get(res, 3, 0) + 2*M_PI);
	while (gsl_matrix_get(res, 3, 0) >  M_PI) gsl_matrix_set(res, 3, 0, gsl_matrix_get(res, 3, 0) - 2*M_PI);
	gsl_matrix_set(res, 4, 0, ry_sensed - ry_state);
	while (gsl_matrix_get(res, 4, 0) < -M_PI) gsl_matrix_set(res, 4, 0, gsl_matrix_get(res, 4, 0) + 2*M_PI);
	while (gsl_matrix_get(res, 4, 0) >  M_PI) gsl_matrix_set(res, 4, 0, gsl_matrix_get(res, 4, 0) - 2*M_PI);

	// compute the H matrix
	#define DX (1e-5)
	for (int j = 0; j < (int)H->size2; j++) {
		gsl_matrix_memcpy(X_delta, X_full_kalm);
		gsl_matrix_set(X_delta, j, 0, gsl_matrix_get(X_delta, j, 0) + DX);
		if (!getFullTransformationMatrix(X_delta, TtipDelta, (((int)X_delta->size1)-6)/2)) {
			printf("\tError: steerCorrectFullKalm, something wrong when grabbing TtipDelta.\n\n");
			return false;
		}
		for (int i = 0; i < (int)H->size1; i++) {
			double hElement = 0.0;
			if (i < 3) {
				hElement = gsl_matrix_get(TtipDelta, i, 3) - gsl_matrix_get(Ttip, i, 3);
			} else if (i == 3) {
				hElement = atan2(gsl_matrix_get(TtipDelta, 1, 0), gsl_matrix_get(TtipDelta, 0, 0)) - atan2(gsl_matrix_get(Ttip, 1, 0), gsl_matrix_get(Ttip, 0, 0));
				while (hElement < -M_PI) hElement = hElement + 2.0*M_PI;
				while (hElement >  M_PI) hElement = hElement - 2.0*M_PI;
			} else if (i == 4) {
				hElement = atan2(-gsl_matrix_get(TtipDelta, 2, 0), sqrt(gsl_matrix_get(TtipDelta, 2, 1)*gsl_matrix_get(TtipDelta, 2, 1) + gsl_matrix_get(TtipDelta, 2, 2)*gsl_matrix_get(TtipDelta, 2, 2))) - atan2(-gsl_matrix_get(Ttip, 2, 0), sqrt(gsl_matrix_get(Ttip, 2, 1)*gsl_matrix_get(Ttip, 2, 1) + gsl_matrix_get(Ttip, 2, 2)*gsl_matrix_get(Ttip, 2, 2)));
				while (hElement < -M_PI) hElement = hElement + 2.0*M_PI;
				while (hElement >  M_PI) hElement = hElement - 2.0*M_PI;
			} else {
				printf("\tError: steerCorrectFullKalm, something went wrong computing H.\n\n");
				return false;
			}
			gsl_matrix_set(H, i, j, (hElement/DX));
		}
	}

	// compute the innovation matrix
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, H, P_full_kalm, 0.0, HP);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, HP, H, 0.0, S);
	for (int i = 0; i < 3; i++) gsl_matrix_set(S, i, i, gsl_matrix_get(S, i, i) + MEASUREMENT_SPATIAL_UNCERTAINTY);
	for (int i = 3; i < 5; i++) gsl_matrix_set(S, i, i, gsl_matrix_get(S, i, i) + MEASUREMENT_ANGULAR_UNCERTAINTY);

	// invert the innovation matrix
	int s = 0;
	gsl_matrix_memcpy(work, S);
	gsl_linalg_LU_decomp(work, p, &s);
	gsl_linalg_LU_invert(work, p, iS);

	// compute the Kalman gain
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, P_full_kalm, H, 0.0, PHt);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, PHt, iS, 0.0, K);

	// compute the updated state as the EKF when simplified
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, res, 0.0, deltaX);
	for (int i = 0; i < (int)X_full_kalm->size1; i++) gsl_matrix_set(X_full_kalm, i, 0, gsl_matrix_get(X_full_kalm, i, 0) + gsl_matrix_get(deltaX, i, 0));

	// compute the updated covariance matrix
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, HP, 0.0, KHP);
	for (int i = 0; i < (int)X_full_kalm->size1; i++) {
		for (int j = i; j < (int)X_full_kalm->size1; j++) {
			double val = gsl_matrix_get(P_full_kalm, i, j) - gsl_matrix_get(KHP, i, j);
			gsl_matrix_set(P_full_kalm, i, j, val);
			gsl_matrix_set(P_full_kalm, j, i, val);
		}
	}

	// delete all of the extra allocated space
	if (res) gsl_matrix_free(res);
	if (deltaX) gsl_matrix_free(deltaX);
	if (H) gsl_matrix_free(H);
	if (HP) gsl_matrix_free(HP);
	if (S) gsl_matrix_free(S);
	if (work) gsl_matrix_free(work);
	if (iS) gsl_matrix_free(iS);
	if (p) gsl_permutation_free(p);
	if (PHt) gsl_matrix_free(PHt);
	if (K) gsl_matrix_free(K);
	if (KHP) gsl_matrix_free(KHP);
	if (Ttip) gsl_matrix_free(Ttip);
	if (TtipDelta) gsl_matrix_free(TtipDelta);
	if (X_delta) gsl_matrix_free(X_delta);

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::steerCorrectDistalKalm()                                   *
*                                                                              *
*  Kalman update for steering, updates with the tracker.                       *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::steerCorrectDistalKalm(gsl_matrix *trackerPose) {

	// some safety checks
	if (!trackerPose || trackerPose->size1 != 4 || trackerPose->size2 != 4) {
		printf("\tError: steerCorrectDistalKalm, something wrong with trackerPose argument.\n\n");
		return false;
	}

	// check if we're supposed to update
	if (!distalKalmUpdating) return false;

	// calculate the roll and pitch values from the transformation matrix
	double rz = atan2(gsl_matrix_get(trackerPose, 1, 0),gsl_matrix_get(trackerPose, 0, 0));
	double ry = atan2(-gsl_matrix_get(trackerPose, 2, 0),sqrt(gsl_matrix_get(trackerPose, 2, 1)*gsl_matrix_get(trackerPose, 2, 1) + gsl_matrix_get(trackerPose, 2, 2)*gsl_matrix_get(trackerPose, 2, 2)));

	// allocate the matrices needed for the Kalman filter
	gsl_matrix *res = gsl_matrix_alloc(5,1);
	gsl_matrix *deltaX = gsl_matrix_alloc(6,1);
	gsl_matrix *H = gsl_matrix_calloc(5, 6);
	gsl_matrix *HP = gsl_matrix_alloc(5, 6);
	gsl_matrix *S = gsl_matrix_alloc(5, 5);
	gsl_matrix *work = gsl_matrix_alloc(5, 5);
	gsl_matrix *iS = gsl_matrix_alloc(5, 5);
	gsl_permutation *p = gsl_permutation_alloc(5);
	gsl_matrix *PHt = gsl_matrix_alloc(6, 5);
	gsl_matrix *K = gsl_matrix_alloc(6, 5);
	gsl_matrix *KHP = gsl_matrix_alloc(6, 6);

	// set the values in the residual vector
	gsl_matrix_set(res, 0, 0, gsl_matrix_get(trackerPose, 0, 3) - gsl_matrix_get(X_distal_kalm, 0, 0));
	gsl_matrix_set(res, 1, 0, gsl_matrix_get(trackerPose, 1, 3) - gsl_matrix_get(X_distal_kalm, 1, 0));
	gsl_matrix_set(res, 2, 0, gsl_matrix_get(trackerPose, 2, 3) - gsl_matrix_get(X_distal_kalm, 2, 0));
	gsl_matrix_set(res, 3, 0, rz - gsl_matrix_get(X_distal_kalm, 3, 0));
	while (gsl_matrix_get(res, 3, 0) < -M_PI) gsl_matrix_set(res, 3, 0, gsl_matrix_get(res, 3, 0) + 2*M_PI);
	while (gsl_matrix_get(res, 3, 0) >  M_PI) gsl_matrix_set(res, 3, 0, gsl_matrix_get(res, 3, 0) - 2*M_PI);
	gsl_matrix_set(res, 4, 0, ry - gsl_matrix_get(X_distal_kalm, 4, 0));
	while (gsl_matrix_get(res, 4, 0) < -M_PI) gsl_matrix_set(res, 4, 0, gsl_matrix_get(res, 4, 0) + 2*M_PI);
	while (gsl_matrix_get(res, 4, 0) >  M_PI) gsl_matrix_set(res, 4, 0, gsl_matrix_get(res, 4, 0) - 2*M_PI);

	// compute the innovation matrix
	for (int i = 0; i < 5; i++) gsl_matrix_set(H, i, i, 1.0);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, H, P_distal_kalm, 0.0, HP);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, HP, H, 0.0, S);
	for (int i = 0; i < 3; i++) gsl_matrix_set(S, i, i, gsl_matrix_get(S, i, i) + MEASUREMENT_SPATIAL_UNCERTAINTY);
	for (int i = 3; i < 5; i++) gsl_matrix_set(S, i, i, gsl_matrix_get(S, i, i) + MEASUREMENT_ANGULAR_UNCERTAINTY);

	// invert the innovation matrix
	int s = 0;
	gsl_matrix_memcpy(work, S);
	gsl_linalg_LU_decomp(work, p, &s);
	gsl_linalg_LU_invert(work, p, iS);

	// compute the Kalman gain
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, P_distal_kalm, H, 0.0, PHt);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, PHt, iS, 0.0, K);

	// compute the updated state
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, res, 0.0, deltaX);
	for (int i = 0; i < 6; i++) gsl_matrix_set(X_distal_kalm, i, 0, gsl_matrix_get(X_distal_kalm, i, 0) + gsl_matrix_get(deltaX, i, 0));

	// compute the updated covariance matrix
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, HP, 0.0, KHP);
	for (int i = 0; i < 6; i++) {
		for (int j = i; j < 6; j++) {
			double val = gsl_matrix_get(P_distal_kalm, i, j) - gsl_matrix_get(KHP, i, j);
			gsl_matrix_set(P_distal_kalm, i, j, val);
			gsl_matrix_set(P_distal_kalm, j, i, val);
		}
	}

	// delete all of the extra allocated space
	if (res) gsl_matrix_free(res);
	if (deltaX) gsl_matrix_free(deltaX);
	if (H) gsl_matrix_free(H);
	if (HP) gsl_matrix_free(HP);
	if (S) gsl_matrix_free(S);
	if (work) gsl_matrix_free(work);
	if (iS) gsl_matrix_free(iS);
	if (p) gsl_permutation_free(p);
	if (PHt) gsl_matrix_free(PHt);
	if (K) gsl_matrix_free(K);
	if (KHP) gsl_matrix_free(KHP);

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::steerPredictDistalKalm()                                   *
*                                                                              *
*  Motion model for steering, updates the pose of the distal estimate.         *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::steerPredictDistalKalm(float phi, float theta) {

	// make sure we've initialized the state
	if (!isInitializedSnakeEstimation())
		return false;

	// allocate matrices we will need
	gsl_matrix *Tpose = gsl_matrix_calloc(4,4);
	gsl_matrix *TundoOldPhiTheta = gsl_matrix_calloc(4,4);
	gsl_matrix *TapplyNewPhiTheta = gsl_matrix_calloc(4,4);
	gsl_matrix *Tintermediate = gsl_matrix_alloc(4,4);
	gsl_matrix *Tresult = gsl_matrix_alloc(4,4);
	gsl_matrix *F = gsl_matrix_calloc(6,6);
	gsl_matrix *FP = gsl_matrix_alloc(6,6);

	//---------------------------------------------------------------------------------
	// first step the estimate back to the point of rotation

	// pull out the roll, pitch, and yaw for convenience
	double rz = gsl_matrix_get(X_distal_kalm, 3, 0);
	double ry = gsl_matrix_get(X_distal_kalm, 4, 0);
	double rx = gsl_matrix_get(X_distal_kalm, 5, 0);

	// move back the position with LINK_LENGTHS_TURNING, keep the orientation the same
	gsl_matrix_set(X_distal_kalm, 0, 0, gsl_matrix_get(X_distal_kalm, 0, 0) - cos(rz)*cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_distal_kalm, 1, 0, gsl_matrix_get(X_distal_kalm, 1, 0) - sin(rz)*cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_distal_kalm, 2, 0, gsl_matrix_get(X_distal_kalm, 2, 0) + sin(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);

	// compute the F matrix for this transformation
	for (int i = 0; i < 6; i++) gsl_matrix_set(F, i, i, 1.0);
	gsl_matrix_set(F, 0, 3,  sin(rz)*cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(F, 0, 4,  cos(rz)*sin(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(F, 1, 3, -cos(rz)*cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(F, 1, 4,  sin(rz)*sin(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(F, 2, 4,  cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);

	// transform P as FPF'
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, F, P_distal_kalm, 0.0, FP);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, FP, F, 0.0, P_distal_kalm);

	//---------------------------------------------------------------------------------
	// then rotate the old phi and theta back to the reference pose

	// rotate the estimate back from the old thetaDistalKalm and phiDistalKalm
	gsl_matrix_set(X_distal_kalm, 3, 0, atan2((cos(phiDistalKalm)*cos(ry)*sin(rz)+sin(phiDistalKalm)*(cos(rx+thetaDistalKalm)*sin(ry)*sin(rz)-cos(rz)*sin(rx+thetaDistalKalm))),(cos(phiDistalKalm)*cos(ry)*cos(rz)+sin(phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))))));
	gsl_matrix_set(X_distal_kalm, 4, 0, atan2((-(cos(rx)*cos(ry)*cos(thetaDistalKalm)*sin(phiDistalKalm))+cos(phiDistalKalm)*sin(ry)+cos(ry)*sin(phiDistalKalm)*sin(rx)*sin(thetaDistalKalm)),sqrt(pow(-(sin(phiDistalKalm)*sin(ry)*sin(thetaDistalKalm))+cos(ry)*sin(rx)*(pow(cos(thetaDistalKalm),2)+cos(phiDistalKalm)*pow(sin(thetaDistalKalm),2))+cos(rx)*cos(ry)*pow(sin(phiDistalKalm/2.0),2)*sin(2*thetaDistalKalm),2)+pow(cos(thetaDistalKalm)*sin(phiDistalKalm)*sin(ry)+cos(rx)*cos(ry)*(cos(phiDistalKalm)*pow(cos(thetaDistalKalm),2)+pow(sin(thetaDistalKalm),2))+cos(ry)*pow(sin(phiDistalKalm/2.0),2)*sin(rx)*sin(2*thetaDistalKalm),2))));
	gsl_matrix_set(X_distal_kalm, 5, 0, atan2((-(sin(phiDistalKalm)*sin(ry)*sin(thetaDistalKalm))+cos(ry)*sin(rx)*(pow(cos(thetaDistalKalm),2)+cos(phiDistalKalm)*pow(sin(thetaDistalKalm),2))+cos(rx)*cos(ry)*pow(sin(phiDistalKalm/2.),2)*sin(2*thetaDistalKalm)),(cos(thetaDistalKalm)*sin(phiDistalKalm)*sin(ry)+cos(rx)*cos(ry)*(cos(phiDistalKalm)*pow(cos(thetaDistalKalm),2)+pow(sin(thetaDistalKalm),2))+cos(ry)*pow(sin(phiDistalKalm/2.0),2)*sin(rx)*sin(2*thetaDistalKalm))));
		
	// compute F for this transformation
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++) gsl_matrix_set(F, i, j, 0.0);
	for (int i = 0; i < 4; i++) gsl_matrix_set(F, i, i, 1.0);
	gsl_matrix_set(F, 3, 4, (sin(phiDistalKalm)*(cos(rx)*cos(ry)*cos(thetaDistalKalm)*sin(phiDistalKalm)-cos(phiDistalKalm)*sin(ry)-cos(ry)*sin(phiDistalKalm)*sin(rx)*sin(thetaDistalKalm))*sin(rx+thetaDistalKalm))/(pow(cos(phiDistalKalm)*cos(ry)*cos(rz)+sin(phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))),2)*(1+pow(cos(phiDistalKalm)*cos(ry)*sin(rz)+sin(phiDistalKalm)*(cos(rx+thetaDistalKalm)*sin(ry)*sin(rz)-cos(rz)*sin(rx+thetaDistalKalm)),2)/pow(cos(phiDistalKalm)*cos(ry)*cos(rz)+sin(phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))),2))));
	gsl_matrix_set(F, 4, 4, (4*(cos(phiDistalKalm)*cos(ry)+cos(rx+thetaDistalKalm)*sin(phiDistalKalm)*sin(ry)))/sqrt(10-2*cos(2*rx)*cos(2*thetaDistalKalm)+cos(2*phiDistalKalm)*(-2+6*cos(2*ry)+2*cos(2*(rx+thetaDistalKalm))+cos(2*(rx-ry+thetaDistalKalm))+cos(2*(rx+ry+thetaDistalKalm)))+8*cos(rx)*cos(thetaDistalKalm)*sin(2*phiDistalKalm)*sin(2*ry)-8*sin(2*phiDistalKalm)*sin(rx)*sin(2*ry)*sin(thetaDistalKalm)+2*sin(2*rx)*sin(2*thetaDistalKalm)+4*cos(2*ry)*pow(sin(rx+thetaDistalKalm),2)));
	gsl_matrix_set(F, 5, 4, (-32*sin(phiDistalKalm)*sin(rx+thetaDistalKalm))/(20-4*cos(2*phiDistalKalm)+6*cos(2*(phiDistalKalm-ry))+4*cos(2*ry)+6*cos(2*(phiDistalKalm+ry))+2*cos(2*(phiDistalKalm-rx-thetaDistalKalm))+4*cos(2*phiDistalKalm-rx-2*ry-thetaDistalKalm)+cos(2*(phiDistalKalm-rx-ry-thetaDistalKalm))+cos(2*(phiDistalKalm-rx+ry-thetaDistalKalm))-4*cos(2*phiDistalKalm-rx+2*ry-thetaDistalKalm)-4*cos(2*(rx+thetaDistalKalm))+2*cos(2*(phiDistalKalm+rx+thetaDistalKalm))+4*cos(2*phiDistalKalm+rx-2*ry+thetaDistalKalm)-2*cos(2*(rx-ry+thetaDistalKalm))+cos(2*(phiDistalKalm+rx-ry+thetaDistalKalm))-2*cos(2*(rx+ry+thetaDistalKalm))+cos(2*(phiDistalKalm+rx+ry+thetaDistalKalm))-4*cos(2*phiDistalKalm+rx+2*ry+thetaDistalKalm)));
	gsl_matrix_set(F, 3, 5, -((sin(phiDistalKalm)*(cos(phiDistalKalm)*cos(ry)*cos(rx+thetaDistalKalm)+sin(phiDistalKalm)*sin(ry)))/(pow(cos(phiDistalKalm)*cos(ry)*cos(rz)+sin(phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))),2)*(1+pow(cos(phiDistalKalm)*cos(ry)*sin(rz)+sin(phiDistalKalm)*(cos(rx+thetaDistalKalm)*sin(ry)*sin(rz)-cos(rz)*sin(rx+thetaDistalKalm)),2)/pow(cos(phiDistalKalm)*cos(ry)*cos(rz)+sin(phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))),2)))));
	gsl_matrix_set(F, 4, 5, (4*cos(ry)*sin(phiDistalKalm)*sin(rx+thetaDistalKalm))/sqrt(10-2*cos(2*rx)*cos(2*thetaDistalKalm)+cos(2*phiDistalKalm)*(-2+6*cos(2*ry)+2*cos(2*(rx+thetaDistalKalm))+cos(2*(rx-ry+thetaDistalKalm))+cos(2*(rx+ry+thetaDistalKalm)))+8*cos(rx)*cos(thetaDistalKalm)*sin(2*phiDistalKalm)*sin(2*ry)-8*sin(2*phiDistalKalm)*sin(rx)*sin(2*ry)*sin(thetaDistalKalm)+2*sin(2*rx)*sin(2*thetaDistalKalm)+4*cos(2*ry)*pow(sin(rx+thetaDistalKalm),2)));
	gsl_matrix_set(F, 5, 5, (32*cos(ry)*(cos(phiDistalKalm)*cos(ry)+cos(rx+thetaDistalKalm)*sin(phiDistalKalm)*sin(ry)))/(20-4*cos(2*phiDistalKalm)+6*cos(2*(phiDistalKalm-ry))+4*cos(2*ry)+6*cos(2*(phiDistalKalm+ry))+2*cos(2*(phiDistalKalm-rx-thetaDistalKalm))+4*cos(2*phiDistalKalm-rx-2*ry-thetaDistalKalm)+cos(2*(phiDistalKalm-rx-ry-thetaDistalKalm))+cos(2*(phiDistalKalm-rx+ry-thetaDistalKalm))-4*cos(2*phiDistalKalm-rx+2*ry-thetaDistalKalm)-4*cos(2*(rx+thetaDistalKalm))+2*cos(2*(phiDistalKalm+rx+thetaDistalKalm))+4*cos(2*phiDistalKalm+rx-2*ry+thetaDistalKalm)-2*cos(2*(rx-ry+thetaDistalKalm))+cos(2*(phiDistalKalm+rx-ry+thetaDistalKalm))-2*cos(2*(rx+ry+thetaDistalKalm))+cos(2*(phiDistalKalm+rx+ry+thetaDistalKalm))-4*cos(2*phiDistalKalm+rx+2*ry+thetaDistalKalm)));
	
	// then transform P according to this transformation, P = FPF'
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, F, P_distal_kalm, 0.0, FP);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, FP, F, 0.0, P_distal_kalm);

	//---------------------------------------------------------------------------------
	// then rotate to the new phi and theta

	// recompute, as a reference, the roll, pitch, and yaw
	rz = gsl_matrix_get(X_distal_kalm, 3, 0);
	ry = gsl_matrix_get(X_distal_kalm, 4, 0);
	rx = gsl_matrix_get(X_distal_kalm, 5, 0);

	// load the new phi and theta to steer to
	thetaDistalKalm = theta;
	phiDistalKalm = phi;

	// rotate the estimate to the new thetaDistalKalm and phiDistalKalm
	gsl_matrix_set(X_distal_kalm, 3, 0, atan2((cos(-phiDistalKalm)*cos(ry)*sin(rz)+sin(-phiDistalKalm)*(cos(rx+thetaDistalKalm)*sin(ry)*sin(rz)-cos(rz)*sin(rx+thetaDistalKalm))),(cos(-phiDistalKalm)*cos(ry)*cos(rz)+sin(-phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))))));
	gsl_matrix_set(X_distal_kalm, 4, 0, atan2((-(cos(rx)*cos(ry)*cos(thetaDistalKalm)*sin(-phiDistalKalm))+cos(-phiDistalKalm)*sin(ry)+cos(ry)*sin(-phiDistalKalm)*sin(rx)*sin(thetaDistalKalm)),sqrt(pow(-(sin(-phiDistalKalm)*sin(ry)*sin(thetaDistalKalm))+cos(ry)*sin(rx)*(pow(cos(thetaDistalKalm),2)+cos(-phiDistalKalm)*pow(sin(thetaDistalKalm),2))+cos(rx)*cos(ry)*pow(sin(-phiDistalKalm/2.0),2)*sin(2*thetaDistalKalm),2)+pow(cos(thetaDistalKalm)*sin(-phiDistalKalm)*sin(ry)+cos(rx)*cos(ry)*(cos(-phiDistalKalm)*pow(cos(thetaDistalKalm),2)+pow(sin(thetaDistalKalm),2))+cos(ry)*pow(sin(-phiDistalKalm/2.0),2)*sin(rx)*sin(2*thetaDistalKalm),2))));
	gsl_matrix_set(X_distal_kalm, 5, 0, atan2((-(sin(-phiDistalKalm)*sin(ry)*sin(thetaDistalKalm))+cos(ry)*sin(rx)*(pow(cos(thetaDistalKalm),2)+cos(-phiDistalKalm)*pow(sin(thetaDistalKalm),2))+cos(rx)*cos(ry)*pow(sin(-phiDistalKalm/2.),2)*sin(2*thetaDistalKalm)),(cos(thetaDistalKalm)*sin(-phiDistalKalm)*sin(ry)+cos(rx)*cos(ry)*(cos(-phiDistalKalm)*pow(cos(thetaDistalKalm),2)+pow(sin(thetaDistalKalm),2))+cos(ry)*pow(sin(-phiDistalKalm/2.0),2)*sin(rx)*sin(2*thetaDistalKalm))));

	// compute F for this transformation
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++) gsl_matrix_set(F, i, j, 0.0);
	for (int i = 0; i < 4; i++) gsl_matrix_set(F, i, i, 1.0);
	gsl_matrix_set(F, 3, 4, (sin(-phiDistalKalm)*(cos(rx)*cos(ry)*cos(thetaDistalKalm)*sin(-phiDistalKalm)-cos(-phiDistalKalm)*sin(ry)-cos(ry)*sin(-phiDistalKalm)*sin(rx)*sin(thetaDistalKalm))*sin(rx+thetaDistalKalm))/(pow(cos(-phiDistalKalm)*cos(ry)*cos(rz)+sin(-phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))),2)*(1+pow(cos(-phiDistalKalm)*cos(ry)*sin(rz)+sin(-phiDistalKalm)*(cos(rx+thetaDistalKalm)*sin(ry)*sin(rz)-cos(rz)*sin(rx+thetaDistalKalm)),2)/pow(cos(-phiDistalKalm)*cos(ry)*cos(rz)+sin(-phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))),2))));
	gsl_matrix_set(F, 4, 4, (4*(cos(-phiDistalKalm)*cos(ry)+cos(rx+thetaDistalKalm)*sin(-phiDistalKalm)*sin(ry)))/sqrt(10-2*cos(2*rx)*cos(2*thetaDistalKalm)+cos(2*-phiDistalKalm)*(-2+6*cos(2*ry)+2*cos(2*(rx+thetaDistalKalm))+cos(2*(rx-ry+thetaDistalKalm))+cos(2*(rx+ry+thetaDistalKalm)))+8*cos(rx)*cos(thetaDistalKalm)*sin(2*-phiDistalKalm)*sin(2*ry)-8*sin(2*-phiDistalKalm)*sin(rx)*sin(2*ry)*sin(thetaDistalKalm)+2*sin(2*rx)*sin(2*thetaDistalKalm)+4*cos(2*ry)*pow(sin(rx+thetaDistalKalm),2)));
	gsl_matrix_set(F, 5, 4, (-32*sin(-phiDistalKalm)*sin(rx+thetaDistalKalm))/(20-4*cos(2*-phiDistalKalm)+6*cos(2*(-phiDistalKalm-ry))+4*cos(2*ry)+6*cos(2*(-phiDistalKalm+ry))+2*cos(2*(-phiDistalKalm-rx-thetaDistalKalm))+4*cos(2*-phiDistalKalm-rx-2*ry-thetaDistalKalm)+cos(2*(-phiDistalKalm-rx-ry-thetaDistalKalm))+cos(2*(-phiDistalKalm-rx+ry-thetaDistalKalm))-4*cos(2*-phiDistalKalm-rx+2*ry-thetaDistalKalm)-4*cos(2*(rx+thetaDistalKalm))+2*cos(2*(-phiDistalKalm+rx+thetaDistalKalm))+4*cos(2*-phiDistalKalm+rx-2*ry+thetaDistalKalm)-2*cos(2*(rx-ry+thetaDistalKalm))+cos(2*(-phiDistalKalm+rx-ry+thetaDistalKalm))-2*cos(2*(rx+ry+thetaDistalKalm))+cos(2*(-phiDistalKalm+rx+ry+thetaDistalKalm))-4*cos(2*-phiDistalKalm+rx+2*ry+thetaDistalKalm)));
	gsl_matrix_set(F, 3, 5, -((sin(-phiDistalKalm)*(cos(-phiDistalKalm)*cos(ry)*cos(rx+thetaDistalKalm)+sin(-phiDistalKalm)*sin(ry)))/(pow(cos(-phiDistalKalm)*cos(ry)*cos(rz)+sin(-phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))),2)*(1+pow(cos(-phiDistalKalm)*cos(ry)*sin(rz)+sin(-phiDistalKalm)*(cos(rx+thetaDistalKalm)*sin(ry)*sin(rz)-cos(rz)*sin(rx+thetaDistalKalm)),2)/pow(cos(-phiDistalKalm)*cos(ry)*cos(rz)+sin(-phiDistalKalm)*(sin(rx)*(cos(thetaDistalKalm)*sin(rz)-cos(rz)*sin(ry)*sin(thetaDistalKalm))+cos(rx)*(cos(rz)*cos(thetaDistalKalm)*sin(ry)+sin(rz)*sin(thetaDistalKalm))),2)))));
	gsl_matrix_set(F, 4, 5, (4*cos(ry)*sin(-phiDistalKalm)*sin(rx+thetaDistalKalm))/sqrt(10-2*cos(2*rx)*cos(2*thetaDistalKalm)+cos(2*-phiDistalKalm)*(-2+6*cos(2*ry)+2*cos(2*(rx+thetaDistalKalm))+cos(2*(rx-ry+thetaDistalKalm))+cos(2*(rx+ry+thetaDistalKalm)))+8*cos(rx)*cos(thetaDistalKalm)*sin(2*-phiDistalKalm)*sin(2*ry)-8*sin(2*-phiDistalKalm)*sin(rx)*sin(2*ry)*sin(thetaDistalKalm)+2*sin(2*rx)*sin(2*thetaDistalKalm)+4*cos(2*ry)*pow(sin(rx+thetaDistalKalm),2)));
	gsl_matrix_set(F, 5, 5, (32*cos(ry)*(cos(-phiDistalKalm)*cos(ry)+cos(rx+thetaDistalKalm)*sin(-phiDistalKalm)*sin(ry)))/(20-4*cos(2*-phiDistalKalm)+6*cos(2*(-phiDistalKalm-ry))+4*cos(2*ry)+6*cos(2*(-phiDistalKalm+ry))+2*cos(2*(-phiDistalKalm-rx-thetaDistalKalm))+4*cos(2*-phiDistalKalm-rx-2*ry-thetaDistalKalm)+cos(2*(-phiDistalKalm-rx-ry-thetaDistalKalm))+cos(2*(-phiDistalKalm-rx+ry-thetaDistalKalm))-4*cos(2*-phiDistalKalm-rx+2*ry-thetaDistalKalm)-4*cos(2*(rx+thetaDistalKalm))+2*cos(2*(-phiDistalKalm+rx+thetaDistalKalm))+4*cos(2*-phiDistalKalm+rx-2*ry+thetaDistalKalm)-2*cos(2*(rx-ry+thetaDistalKalm))+cos(2*(-phiDistalKalm+rx-ry+thetaDistalKalm))-2*cos(2*(rx+ry+thetaDistalKalm))+cos(2*(-phiDistalKalm+rx+ry+thetaDistalKalm))-4*cos(2*-phiDistalKalm+rx+2*ry+thetaDistalKalm)));
	
	// then transform P according to this transformation, P = FPF'
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, F, P_distal_kalm, 0.0, FP);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, FP, F, 0.0, P_distal_kalm);

	//---------------------------------------------------------------------------------
	// then step the estimate forward to the new desired pose

	// move forward the position with LINK_LENGTHS_TURNING, keep the orientation the same
	rz = gsl_matrix_get(X_distal_kalm, 3, 0); ry = gsl_matrix_get(X_distal_kalm, 4, 0);
	gsl_matrix_set(X_distal_kalm, 0, 0, gsl_matrix_get(X_distal_kalm, 0, 0) + cos(rz)*cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_distal_kalm, 1, 0, gsl_matrix_get(X_distal_kalm, 1, 0) + sin(rz)*cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_distal_kalm, 2, 0, gsl_matrix_get(X_distal_kalm, 2, 0) - sin(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);

	// compute the F matrix for this transformation
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++) gsl_matrix_set(F, i, j, 0.0);
	for (int i = 0; i < 6; i++) gsl_matrix_set(F, i, i, 1.0);
	gsl_matrix_set(F, 0, 3, -sin(rz)*cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(F, 1, 3,  cos(rz)*cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(F, 0, 4, -cos(rz)*sin(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(F, 1, 4, -sin(rz)*sin(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(F, 2, 4, -cos(ry)*LINK_LENGTHS_TURNING*SNAKELIB_LINK_LENGTH);

	// transform P as FPF'
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, F, P_distal_kalm, 0.0, FP);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, FP, F, 0.0, P_distal_kalm);

	// add some extra noise each time we evolve the covariance with steering
	for (int i = 0; i < 3; i++) gsl_matrix_set(P_distal_kalm, i, i, gsl_matrix_get(P_distal_kalm, i, i) + STEER_SPATIAL_UNCERTAINTY_DISTAL_KALM); 
	for (int i = 3; i < 6; i++) gsl_matrix_set(P_distal_kalm, i, i, gsl_matrix_get(P_distal_kalm, i, i) + STEER_ANGULAR_UNCERTAINTY_DISTAL_KALM); 

	// release the allocated memory
	if (Tpose) gsl_matrix_free(Tpose);
	if (TundoOldPhiTheta) gsl_matrix_free(TundoOldPhiTheta);
	if (TapplyNewPhiTheta) gsl_matrix_free(TapplyNewPhiTheta);
	if (Tintermediate) gsl_matrix_free(Tintermediate);
	if (Tresult) gsl_matrix_free(Tresult);

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::steerPredictFullIterated()                                 *
*                                                                              *
*  Motion model for steering, updates the pose of the full iterated estimate.  *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::steerPredictFullIterated(float phi, float theta) {

	// make sure we've initialized the state
	if (!isInitializedSnakeEstimation())
		return false;

	// check that we have enough initialized links
	int numLinks = (X_full_iterated->size1 - 6)/2;
	if (numLinks < LINK_LENGTHS_TURNING) {
		printf("\tError: steer predict full iterated, not enough initialized links for steering.\n\n");
		return false;
	}

	// turn each link a fraction of the total phi value in the direction of theta
	for (int i = 0; i < LINK_LENGTHS_TURNING; i++) {
		gsl_matrix_set(X_full_iterated, X_full_iterated->size1-2*i-2, 0, phi/((double)LINK_LENGTHS_TURNING));
		gsl_matrix_set(X_full_iterated, X_full_iterated->size1-2*i-1, 0, theta);
		gsl_matrix_set(P_full_iterated, P_full_iterated->size1-2*i-2, P_full_iterated->size1-2*i-2, gsl_matrix_get(P_full_iterated, P_full_iterated->size1-2*i-2, P_full_iterated->size1-2*i-2) + STEER_ANGULAR_UNCERTAINTY_FULL_PHI);
		gsl_matrix_set(P_full_iterated, P_full_iterated->size1-2*i-1, P_full_iterated->size1-2*i-1, gsl_matrix_get(P_full_iterated, P_full_iterated->size1-2*i-1, P_full_iterated->size1-2*i-1) + STEER_ANGULAR_UNCERTAINTY_FULL_THETA);
	}

	// add a little noise to the other dimensions
	int upperLimitLink = numLinks-1;
	int lowerLimitLink = numLinks-3;
	if (lowerLimitLink <= 0) lowerLimitLink = 1;
	for (int i = upperLimitLink; i >= lowerLimitLink; i--) {
		gsl_matrix_set(P_full_iterated, 5+2*i-1, 5+2*i-1, gsl_matrix_get(P_full_iterated, 5+2*i-1, 5+2*i-1) + (1.0 - ((double)(upperLimitLink-i))/((double)(upperLimitLink-lowerLimitLink)))*(0.02*M_PI/180.0)*(0.02*M_PI/180.0));
		gsl_matrix_set(P_full_iterated, 5+2*i, 5+2*i, gsl_matrix_get(P_full_iterated, 5+2*i, 5+2*i) + (1.0 - ((double)(upperLimitLink-i))/((double)(upperLimitLink-lowerLimitLink)))*(0.02*M_PI/180.0)*(0.02*M_PI/180.0));
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::steerPredictFullKalm()                                     *
*                                                                              *
*  Motion model for steering, updates the pose of the full kalm estimate.      *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::steerPredictFullKalm(float phi, float theta) {

	// make sure we've initialized the state
	if (!isInitializedSnakeEstimation())
		return false;

	// check that we have enough initialized links
	int numLinks = (X_full_kalm->size1 - 6)/2;
	if (numLinks < LINK_LENGTHS_TURNING) {
		printf("\tError: steer predict full kalm, not enough initialized links for steering.\n\n");
		return false;
	}

	// turn each link a fraction of the total phi value in the direction of theta
	for (int i = 0; i < LINK_LENGTHS_TURNING; i++) {
		gsl_matrix_set(X_full_kalm, X_full_kalm->size1-2*i-2, 0, phi/((double)LINK_LENGTHS_TURNING));
		gsl_matrix_set(X_full_kalm, X_full_kalm->size1-2*i-1, 0, theta);
		gsl_matrix_set(P_full_kalm, P_full_kalm->size1-2*i-2, P_full_kalm->size1-2*i-2, gsl_matrix_get(P_full_kalm, P_full_kalm->size1-2*i-2, P_full_kalm->size1-2*i-2) + STEER_ANGULAR_UNCERTAINTY_FULL_PHI);
		gsl_matrix_set(P_full_kalm, P_full_kalm->size1-2*i-1, P_full_kalm->size1-2*i-1, gsl_matrix_get(P_full_kalm, P_full_kalm->size1-2*i-1, P_full_kalm->size1-2*i-1) + STEER_ANGULAR_UNCERTAINTY_FULL_THETA);
	}

	// add a little noise to the other dimensions
	/*int upperLimitLink = numLinks-1;
	int lowerLimitLink = numLinks-3;
	if (lowerLimitLink <= 0) lowerLimitLink = 1;
	for (int i = upperLimitLink; i >= lowerLimitLink; i--) {
		gsl_matrix_set(P_full_kalm, 5+2*i-1, 5+2*i-1, gsl_matrix_get(P_full_kalm, 5+2*i-1, 5+2*i-1) + (1.0 - ((double)(upperLimitLink-i))/((double)(upperLimitLink-lowerLimitLink)))*(0.3*M_PI/180.0)*(0.3*M_PI/180.0));
		gsl_matrix_set(P_full_kalm, 5+2*i, 5+2*i, gsl_matrix_get(P_full_kalm, 5+2*i, 5+2*i) + (1.0 - ((double)(upperLimitLink-i))/((double)(upperLimitLink-lowerLimitLink)))*(0.3*M_PI/180.0)*(0.3*M_PI/180.0));
	}*/

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::steerPredictFullDead()                                     *
*                                                                              *
*  Motion model for steering, updates the pose of the full kalm estimate.      *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::steerPredictFullDead(float phi, float theta) {

	// make sure we've initialized the state
	if (!isInitializedSnakeEstimation())
		return false;

	// check that we have enough initialized links
	int numLinks = (X_full_dead->size1 - 6)/2;
	if (numLinks < LINK_LENGTHS_TURNING) {
		printf("\tError: steer predict full dead, not enough initialized links for steering.\n\n");
		return false;
	}

	// turn each link a little bit
	for (int i = 0; i < LINK_LENGTHS_TURNING; i++) {
		gsl_matrix_set(X_full_dead, X_full_dead->size1-2*i-2, 0, phi/(double)LINK_LENGTHS_TURNING);
		gsl_matrix_set(X_full_dead, X_full_dead->size1-2*i-1, 0, theta);
	}

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::retractDistalKalm()                                        *
*                                                                              *
*  Retracts the distal kalm estimate back one link.                            *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::retractDistalKalm() {

	// check if we're doing estimation on the distal link
	if (!isInitializedSnakeEstimation())
		return false;

	// make sure we reset the reference for steering
	phiDistalKalm = 0.0;
	thetaDistalKalm = 0.0;

	// pull out the roll pitch and yaw
	double rz = gsl_matrix_get(X_distal_kalm, 3, 0);
	double ry = gsl_matrix_get(X_distal_kalm, 4, 0);

	// move back the position, keep the orientation the same
	gsl_matrix_set(X_distal_kalm, 0, 0, gsl_matrix_get(X_distal_kalm, 0, 0) - cos(rz)*cos(ry)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_distal_kalm, 1, 0, gsl_matrix_get(X_distal_kalm, 1, 0) - sin(rz)*cos(ry)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_distal_kalm, 2, 0, gsl_matrix_get(X_distal_kalm, 2, 0) + sin(ry)*SNAKELIB_LINK_LENGTH);

	// add some noise when retracting
	for (int i = 0; i < 3; i++) gsl_matrix_set(P_distal_kalm, i, i, gsl_matrix_get(P_distal_kalm, i, i) + RETRACT_SPATIAL_UNCERTAINTY_DISTAL_KALM);
	for (int i = 3; i < 6; i++) gsl_matrix_set(P_distal_kalm, i, i, gsl_matrix_get(P_distal_kalm, i, i) + RETRACT_ANGULAR_UNCERTAINTY_DISTAL_KALM);

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::retractFullIterated()                                      *
*                                                                              *
*  Retracts the full iterated estimate back one link.                          *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::retractFullIterated() {

	// check if we're doing estimation on the snake
	if (!isInitializedSnakeEstimation())
		return false;

	// remove a link
	if (X_full_iterated->size1 >= 8) {
		gsl_matrix *X_full_iterated_new = gsl_matrix_alloc(X_full_iterated->size1-2,1);
		for (int i = 0; i < (int)X_full_iterated_new->size1; i++) gsl_matrix_set(X_full_iterated_new, i, 0, gsl_matrix_get(X_full_iterated, i, 0));
		gsl_matrix_free(X_full_iterated);
		X_full_iterated = X_full_iterated_new;
		gsl_matrix *P_full_iterated_new = gsl_matrix_calloc(P_full_iterated->size1-2,P_full_iterated->size2-2);
		for (int i = 0; i < (int)P_full_iterated_new->size1; i++) {
			for (int j = 0; j < (int)P_full_iterated_new->size2; j++) {
				gsl_matrix_set(P_full_iterated_new, i, j, gsl_matrix_get(P_full_iterated, i, j));
			}
		}
		gsl_matrix_free(P_full_iterated);
		P_full_iterated = P_full_iterated_new;
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::retractFullKalm()                                          *
*                                                                              *
*  Retracts the full kalm estimate back one link.                              *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::retractFullKalm() {

	// check if we're doing estimation on the snake
	if (!isInitializedSnakeEstimation())
		return false;

	// remove a link
	if (X_full_kalm->size1 >= 8) {
		gsl_matrix *X_full_kalm_new = gsl_matrix_alloc(X_full_kalm->size1-2,1);
		for (int i = 0; i < (int)X_full_kalm_new->size1; i++) gsl_matrix_set(X_full_kalm_new, i, 0, gsl_matrix_get(X_full_kalm, i, 0));
		gsl_matrix_free(X_full_kalm);
		X_full_kalm = X_full_kalm_new;
		gsl_matrix *P_full_kalm_new = gsl_matrix_calloc(P_full_kalm->size1-2,P_full_kalm->size2-2);
		for (int i = 0; i < (int)P_full_kalm_new->size1; i++) {
			for (int j = 0; j < (int)P_full_kalm_new->size2; j++) {
				gsl_matrix_set(P_full_kalm_new, i, j, gsl_matrix_get(P_full_kalm, i, j));
			}
		}
		gsl_matrix_free(P_full_kalm);
		P_full_kalm = P_full_kalm_new;
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::retractFullDead()                                          *
*                                                                              *
*  Retracts the full dead estimate back one link.                              *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::retractFullDead() {

	// check if we're doing estimation on the snake
	if (!isInitializedSnakeEstimation())
		return false;

	// remove a link
	if (X_full_dead->size1 >= 8) {
		gsl_matrix *X_full_dead_new = gsl_matrix_alloc(X_full_dead->size1-2,1);
		for (int i = 0; i < (int)X_full_dead_new->size1; i++) gsl_matrix_set(X_full_dead_new, i, 0, gsl_matrix_get(X_full_dead, i, 0));
		gsl_matrix_free(X_full_dead);
		X_full_dead = X_full_dead_new;
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::advanceDistalKalm()                                        *
*                                                                              *
*  Advances the distal kalm estimate forward one link.                         *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::advanceDistalKalm(float phi, float theta) {

	// check if we're doing estimation on the distal link
	if (!isInitializedSnakeEstimation())
		return false;

	// make sure we reset the reference for steering
	phiDistalKalm = 0.0;
	thetaDistalKalm = 0.0;

	// pull out the roll pitch and yaw
	double rz = gsl_matrix_get(X_distal_kalm, 3, 0);
	double ry = gsl_matrix_get(X_distal_kalm, 4, 0);

	// move forward the position, keep the orientation the same
	gsl_matrix_set(X_distal_kalm, 0, 0, gsl_matrix_get(X_distal_kalm, 0, 0) + cos(rz)*cos(ry)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_distal_kalm, 1, 0, gsl_matrix_get(X_distal_kalm, 1, 0) + sin(rz)*cos(ry)*SNAKELIB_LINK_LENGTH);
	gsl_matrix_set(X_distal_kalm, 2, 0, gsl_matrix_get(X_distal_kalm, 2, 0) - sin(ry)*SNAKELIB_LINK_LENGTH);

	// add some noise when advancing
	for (int i = 0; i < 3; i++) gsl_matrix_set(P_distal_kalm, i, i, gsl_matrix_get(P_distal_kalm, i, i) + ADVANCE_SPATIAL_UNCERTAINTY_DISTAL_KALM);
	for (int i = 3; i < 6; i++) gsl_matrix_set(P_distal_kalm, i, i, gsl_matrix_get(P_distal_kalm, i, i) + ADVANCE_ANGULAR_UNCERTAINTY_DISTAL_KALM);

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::advanceFullIterated()                                      *
*                                                                              *
*  Advances the full iterated estimate forward one link.                       *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::advanceFullIterated(float phi, float theta) {

	// check if we're doing estimation on the snake
	if (!isInitializedSnakeEstimation())
		return false;

	// add the phi and theta to the state
	gsl_matrix *X_full_iterated_new = gsl_matrix_alloc(X_full_iterated->size1+2,1);
	for (int i = 0; i < (int)X_full_iterated->size1; i++) gsl_matrix_set(X_full_iterated_new, i, 0, gsl_matrix_get(X_full_iterated, i, 0));
	gsl_matrix_set(X_full_iterated_new, X_full_iterated->size1, 0, phi);
	gsl_matrix_set(X_full_iterated_new, X_full_iterated->size1+1, 0, theta);
	gsl_matrix_free(X_full_iterated);
	X_full_iterated = X_full_iterated_new;

	// add to the covariance matrix when advancing
	gsl_matrix *P_full_iterated_new = gsl_matrix_calloc(P_full_iterated->size1+2,P_full_iterated->size2+2);
	for (int i = 0; i < (int)P_full_iterated->size1; i++) {
		for (int j = 0; j < (int)P_full_iterated->size2; j++) {
			gsl_matrix_set(P_full_iterated_new, i, j, gsl_matrix_get(P_full_iterated, i, j));
		}
	}
	gsl_matrix_set(P_full_iterated_new, P_full_iterated->size1, P_full_iterated->size1, ADVANCE_ANGULAR_UNCERTAINTY_FULL);
	gsl_matrix_set(P_full_iterated_new, P_full_iterated->size1+1, P_full_iterated->size1+1, ADVANCE_ANGULAR_UNCERTAINTY_FULL);
	gsl_matrix_free(P_full_iterated);
	P_full_iterated = P_full_iterated_new;

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::advanceFullKalm()                                          *
*                                                                              *
*  Advances the full kalm estimate forward one link.                           *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::advanceFullKalm(float phi, float theta) {

	// check if we're doing estimation on the snake
	phi = phi/((float)LINK_LENGTHS_TURNING);
	if (!isInitializedSnakeEstimation())
		return false;

	// add the phi and theta to the state
	gsl_matrix *X_full_kalm_new = gsl_matrix_alloc(X_full_kalm->size1+2,1);
	for (int i = 0; i < (int)X_full_kalm->size1; i++) gsl_matrix_set(X_full_kalm_new, i, 0, gsl_matrix_get(X_full_kalm, i, 0));
	gsl_matrix_set(X_full_kalm_new, X_full_kalm->size1, 0, phi);
	gsl_matrix_set(X_full_kalm_new, X_full_kalm->size1+1, 0, theta);
	gsl_matrix_free(X_full_kalm);
	X_full_kalm = X_full_kalm_new;

	// add to the covariance matrix when advancing
	gsl_matrix *P_full_kalm_new = gsl_matrix_calloc(P_full_kalm->size1+2,P_full_kalm->size2+2);
	for (int i = 0; i < (int)P_full_kalm->size1; i++) {
		for (int j = 0; j < (int)P_full_kalm->size2; j++) {
			gsl_matrix_set(P_full_kalm_new, i, j, gsl_matrix_get(P_full_kalm, i, j));
		}
	}
	gsl_matrix_set(P_full_kalm_new, P_full_kalm->size1, P_full_kalm->size1, ADVANCE_ANGULAR_UNCERTAINTY_FULL);
	gsl_matrix_set(P_full_kalm_new, P_full_kalm->size1+1, P_full_kalm->size1+1, ADVANCE_ANGULAR_UNCERTAINTY_FULL);
	gsl_matrix_free(P_full_kalm);
	P_full_kalm = P_full_kalm_new;
	
	// add some uncertainty to the last few links
	for (int i = (int)P_full_kalm->size1-8; i < (int)P_full_kalm->size1; i++) {
		gsl_matrix_set(P_full_kalm, i, i, gsl_matrix_get(P_full_kalm, i, i) + (0.08*M_PI/180)*(0.08*M_PI/180));
	}

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::advanceFullDead()                                          *
*                                                                              *
*  Advances the full dead estimate forward one link.                           *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::advanceFullDead(float phi, float theta) {

	// check if we're doing estimation on the snake
	if (!isInitializedSnakeEstimation())
		return false;

	// add the phi and theta to the state
	gsl_matrix *X_full_dead_new = gsl_matrix_alloc(X_full_dead->size1+2,1);
	for (int i = 0; i < (int)X_full_dead->size1; i++) gsl_matrix_set(X_full_dead_new, i, 0, gsl_matrix_get(X_full_dead, i, 0));
	gsl_matrix_set(X_full_dead_new, X_full_dead->size1, 0, phi);
	gsl_matrix_set(X_full_dead_new, X_full_dead->size1+1, 0, theta);
	gsl_matrix_free(X_full_dead);
	X_full_dead = X_full_dead_new;

	return true;
}

/******************************************************************************\
*                                                                              *
*  snakeEstimation::drawLink()                                                 *
*                                                                              *
*  Draws a link based on the transformation matrix                             *
*                                                                              *
\******************************************************************************/
bool snakeEstimation::drawLink(gsl_matrix *T, float *colorTop3f, float *colorBottom3f, float alpha, float radius, float linkLength) {

	GLfloat model_matrix[16];

	gsl_matrix *modelview = gsl_matrix_calloc(4,4);
	gsl_matrix *modelviewTransformed = gsl_matrix_calloc(4,4);

	// push the old matrix so we dont screw anything up
	glPushMatrix();

	// make a new GLfloat array to hold the modelview matrix for modification
	glGetFloatv(GL_MODELVIEW_MATRIX, model_matrix);

	int i = 0;
	for(int c = 0; c < 4; c++) {
		for(int r = 0; r < 4; r++) {
			gsl_matrix_set(modelview, r, c, model_matrix[i++]);
		}
	}

	// postmultiply the modelview matrix by the global link matrix
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, modelview, T, 0.0, modelviewTransformed);

	// convert the new modelview matrix back into a GLfloat array
	i = 0;
	for(int c = 0; c < 4; c++) {
		for(int r = 0; r < 4; r++) {
			model_matrix[i++] = (GLfloat)gsl_matrix_get(modelviewTransformed, r, c);
		}
	}
	// and set it as the actual modelview matrix
	glLoadMatrixf(model_matrix);
	gsl_matrix_free(modelview);
	gsl_matrix_free(modelviewTransformed);

  	glBegin(GL_QUADS);

	// loop through in polar coords and draw all the polygons
	#define NUM_THETAS 18
	#define NUM_PHIS   18
	for (int i = 0; i < NUM_THETAS; i++) {
		float theta = (float)(2*M_PI*((float)i)/((float)NUM_THETAS));
		if (i == 4)
			glColor4f(colorTop3f[0], colorTop3f[1], colorTop3f[2], alpha);
		else
			glColor4f(colorBottom3f[0], colorBottom3f[1], colorBottom3f[2], alpha);
		
		// draw the side cylinder polygon
		glNormal3f(0.0, cos(theta), sin(theta)); glVertex3d(0.0, radius*cos(theta), radius*sin(theta));
		glNormal3f(0.0, cos(theta+(float)(2*M_PI/NUM_THETAS)), sin(theta+(float)(2*M_PI/NUM_THETAS))); glVertex3d(0.0, radius*cos(theta+(float)(2*M_PI/NUM_THETAS)), radius*sin(theta+(float)(2*M_PI/NUM_THETAS)));
		glNormal3f(0.0, cos(theta+(float)(2*M_PI/NUM_THETAS)), sin(theta+(float)(2*M_PI/NUM_THETAS))); glVertex3d(-linkLength, radius*cos(theta+(float)(2*M_PI/NUM_THETAS)), radius*sin(theta+(float)(2*M_PI/NUM_THETAS)));
		glNormal3f(0.0, cos(theta), sin(theta)); glVertex3d(-linkLength, radius*cos(theta), radius*sin(theta));

		// draw the half sphere polygons for the top and bottom of the link
		for (int j = 0; j < NUM_PHIS; j++) {
			float phi = (float)(((M_PI/2)*j)/NUM_PHIS);
			glNormal3f(sin(phi), cos(theta)*cos(phi), sin(theta)*cos(phi));
			glVertex3d(radius*sin(phi), radius*cos(theta)*cos(phi), radius*sin(theta)*cos(phi));
			glNormal3f(sin(phi), cos(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi), sin(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi));
			glVertex3d(radius*sin(phi), radius*cos(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi), radius*sin(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi));
			glNormal3f(sin(phi+(float)((M_PI/2)/NUM_PHIS)), cos(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi+(float)((M_PI/2)/NUM_PHIS)), sin(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi+(float)((M_PI/2)/NUM_PHIS)));
			glVertex3d(radius*sin(phi+(float)((M_PI/2)/NUM_PHIS)),radius*cos(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi+(float)((M_PI/2)/NUM_PHIS)), radius*sin(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi+(float)((M_PI/2)/NUM_PHIS)));
			glNormal3f(sin(phi+(float)((M_PI/2)/NUM_PHIS)), cos(theta)*cos(phi+(float)((M_PI/2)/NUM_PHIS)), sin(theta)*cos(phi+(float)((M_PI/2)/NUM_PHIS)));		
			glVertex3d(radius*sin(phi+(float)((M_PI/2)/NUM_PHIS)), radius*cos(theta)*cos(phi+(float)((M_PI/2)/NUM_PHIS)), radius*sin(theta)*cos(phi+(float)((M_PI/2)/NUM_PHIS)));
		}

		// draw a cap on the bottom
		glNormal3f(-1.0, 0.0, 0.0); glVertex3d(-linkLength, radius*cos(theta), radius*sin(theta));
		glNormal3f(-1.0, 0.0, 0.0); glVertex3d(-linkLength, radius*cos(theta+(float)(2*M_PI/NUM_THETAS)), radius*sin(theta+(float)(2*M_PI/NUM_THETAS)));
		glNormal3f(-1.0, 0.0, 0.0); glVertex3d(-linkLength, 0.0, 0.0);
		glNormal3f(-1.0, 0.0, 0.0); glVertex3d(-linkLength, 0.0, 0.0);

	}
	glEnd();
	glPopMatrix();
	return true;
}




































// This is from steerPredictDistalKalm.....

/*
	gsl_matrix_set(Tpose, 0, 0, cos(rz)*cos(ry)); gsl_matrix_set(Tpose, 0, 1, -sin(rz)*cos(rx)+cos(rz)*sin(ry)*sin(rx)); gsl_matrix_set(Tpose, 0, 2, sin(rz)*sin(rx)+cos(rz)*sin(ry)*cos(rx));
	gsl_matrix_set(Tpose, 0, 3, gsl_matrix_get(X_distal_kalm, 0, 0)); gsl_matrix_set(Tpose, 1, 0, sin(rz)*cos(ry)); gsl_matrix_set(Tpose, 1, 1, cos(rz)*cos(rx)+sin(rz)*sin(ry)*sin(rx));
	gsl_matrix_set(Tpose, 1, 2, -cos(rz)*sin(rx)+sin(rz)*sin(ry)*cos(rx)); gsl_matrix_set(Tpose, 1, 3, gsl_matrix_get(X_distal_kalm, 1, 0)); gsl_matrix_set(Tpose, 2, 0, -sin(ry));
	gsl_matrix_set(Tpose, 2, 1, cos(ry)*sin(rx)); gsl_matrix_set(Tpose, 2, 2, cos(ry)*cos(rx)); gsl_matrix_set(Tpose, 2, 3, gsl_matrix_get(X_distal_kalm, 2, 0));
	gsl_matrix_set(Tpose, 3, 3, 1.0);

	gsl_matrix_set(TundoOldPhiTheta, 0, 0,  cos(-phiDistalKalm));
	gsl_matrix_set(TundoOldPhiTheta, 0, 1, -sin(-phiDistalKalm)*cos(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TundoOldPhiTheta, 0, 2, -sin(-phiDistalKalm)*sin(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TundoOldPhiTheta, 1, 0,  sin(-phiDistalKalm)*cos(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TundoOldPhiTheta, 1, 1,  cos(-phiDistalKalm)*cos(thetaDistalKalm-M_PI/2.0)*cos(thetaDistalKalm-M_PI/2.0)+sin(thetaDistalKalm-M_PI/2.0)*sin(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TundoOldPhiTheta, 1, 2,  cos(-phiDistalKalm)*cos(thetaDistalKalm-M_PI/2.0)*sin(thetaDistalKalm-M_PI/2.0)-sin(thetaDistalKalm-M_PI/2.0)*cos(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TundoOldPhiTheta, 2, 0,  sin(-phiDistalKalm)*sin(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TundoOldPhiTheta, 2, 1,  cos(-phiDistalKalm)*sin(thetaDistalKalm-M_PI/2.0)*cos(thetaDistalKalm-M_PI/2.0)-cos(thetaDistalKalm-M_PI/2.0)*sin(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TundoOldPhiTheta, 2, 2,  cos(-phiDistalKalm)*sin(thetaDistalKalm-M_PI/2.0)*sin(thetaDistalKalm-M_PI/2.0)+cos(thetaDistalKalm-M_PI/2.0)*cos(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TundoOldPhiTheta, 3, 3,  1.0);

	thetaDistalKalm = theta;
	phiDistalKalm = phi;

	gsl_matrix_set(TapplyNewPhiTheta, 0, 0,  cos(phiDistalKalm));
	gsl_matrix_set(TapplyNewPhiTheta, 0, 1, -sin(phiDistalKalm)*cos(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TapplyNewPhiTheta, 0, 2, -sin(phiDistalKalm)*sin(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TapplyNewPhiTheta, 1, 0,  sin(phiDistalKalm)*cos(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TapplyNewPhiTheta, 1, 1,  cos(phiDistalKalm)*cos(thetaDistalKalm-M_PI/2.0)*cos(thetaDistalKalm-M_PI/2.0)+sin(thetaDistalKalm-M_PI/2.0)*sin(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TapplyNewPhiTheta, 1, 2,  cos(phiDistalKalm)*cos(thetaDistalKalm-M_PI/2.0)*sin(thetaDistalKalm-M_PI/2.0)-sin(thetaDistalKalm-M_PI/2.0)*cos(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TapplyNewPhiTheta, 2, 0,  sin(phiDistalKalm)*sin(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TapplyNewPhiTheta, 2, 1,  cos(phiDistalKalm)*sin(thetaDistalKalm-M_PI/2.0)*cos(thetaDistalKalm-M_PI/2.0)-cos(thetaDistalKalm-M_PI/2.0)*sin(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TapplyNewPhiTheta, 2, 2,  cos(phiDistalKalm)*sin(thetaDistalKalm-M_PI/2.0)*sin(thetaDistalKalm-M_PI/2.0)+cos(thetaDistalKalm-M_PI/2.0)*cos(thetaDistalKalm-M_PI/2.0));
	gsl_matrix_set(TapplyNewPhiTheta, 3, 3,  1.0);

	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Tpose, TundoOldPhiTheta, 0.0, Tintermediate);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Tintermediate, TapplyNewPhiTheta, 0.0, Tresult);

	// calculate the roll and pitch values from the transformation matrix
	gsl_matrix_set(X_distal_kalm, 0, 0, gsl_matrix_get(Tresult, 0, 3));
	gsl_matrix_set(X_distal_kalm, 1, 0, gsl_matrix_get(Tresult, 1, 3));
	gsl_matrix_set(X_distal_kalm, 2, 0, gsl_matrix_get(Tresult, 2, 3));
	gsl_matrix_set(X_distal_kalm, 3, 0, atan2(gsl_matrix_get(Tresult, 1, 0),gsl_matrix_get(Tresult, 0, 0)));
	gsl_matrix_set(X_distal_kalm, 4, 0, atan2(-gsl_matrix_get(Tresult, 2, 0),sqrt(gsl_matrix_get(Tresult, 2, 1)*gsl_matrix_get(Tresult, 2, 1) + gsl_matrix_get(Tresult, 2, 2)*gsl_matrix_get(Tresult, 2, 2))));
	gsl_matrix_set(X_distal_kalm, 5, 0, atan2(gsl_matrix_get(Tresult, 2, 1),gsl_matrix_get(Tresult, 2, 2)));
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::getNumLinksAdvanced()                                      *
*                                                                              *
*  This function returns the number of advanced links.                         *
*                                                                              *
\******************************************************************************/
/*
int snakeEstimation::getNumLinksAdvanced(void) {
	return numLinksAdvanced;
}
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::scoreKalman()                                              *
*                                                                              *
*  Computes a score to tell how much it agrees with the measurement and the    *
*      old state.                                                              *
*                                                                              *
\******************************************************************************/
/*
float snakeEstimation::scoreKalman(float phi, float theta, float x, float y, float z, gsl_matrix *X_kalm_new) {

		gsl_matrix *linkMatrix = getLinkTransformationMatrix(init_yaw, init_pitch, init_x, init_y, init_z, numLinksAdvanced, numLinksAdvanced-1, X_kalm_new);

		gsl_matrix *tZ = createTranslateZmatrix(SNAKELIB_LINK_LENGTH);
		gsl_matrix *newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, tZ, 0.0, newLinkMatrix);
		gsl_matrix_free(tZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		gsl_matrix *rZ = createRotateZmatrix(theta);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rZ, 0.0, newLinkMatrix);
		gsl_matrix_free(rZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		gsl_matrix *rY = createRotateYmatrix(phi);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rY, 0.0, newLinkMatrix);
		gsl_matrix_free(rY);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		rZ = createRotateZmatrix(-theta);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rZ, 0.0, newLinkMatrix);
		gsl_matrix_free(rZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		tZ = createTranslateZmatrix(SNAKELIB_LINK_LENGTH);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, tZ, 0.0, newLinkMatrix);
		gsl_matrix_free(tZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;
		
		float PARAM1 = 1.0;
		float PARAM2 = 20.0;

		float squaredError = ((float)gsl_matrix_get(linkMatrix, 0, 3) - x)*((float)gsl_matrix_get(linkMatrix, 0, 3) - x) +
							 ((float)gsl_matrix_get(linkMatrix, 1, 3) - y)*((float)gsl_matrix_get(linkMatrix, 1, 3) - y) +
							 ((float)gsl_matrix_get(linkMatrix, 2, 3) - z)*((float)gsl_matrix_get(linkMatrix, 2, 3) - z);
		squaredError += PARAM1*((float)numLinksAdvanced*(float)numLinksAdvanced*((float)gsl_matrix_get(X_kalm_new, 0, 0) - (float)gsl_matrix_get(X_saved, 0, 0))*((float)gsl_matrix_get(X_kalm_new, 0, 0) - (float)gsl_matrix_get(X_saved, 0, 0)));
		for (int i = 0; i < numLinksAdvanced; i++) {
			squaredError += PARAM2*((float)gsl_matrix_get(X_kalm_new, 2*i+1, 0) - (float)gsl_matrix_get(X_saved, 2*i+1, 0))*((float)gsl_matrix_get(X_kalm_new, 2*i+1, 0) - (float)gsl_matrix_get(X_saved, 2*i+1, 0));
			squaredError += PARAM2*((float)gsl_matrix_get(X_kalm_new, 2*i+2, 0) - (float)gsl_matrix_get(X_saved, 2*i+2, 0))*((float)gsl_matrix_get(X_kalm_new, 2*i+2, 0) - (float)gsl_matrix_get(X_saved, 2*i+2, 0));
		}
		
		return squaredError;
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::kalmanCorrect()                                            *
*                                                                              *
*  Corrects the Kalman estimate with the specified sensor location.            *
*                                                                              *
\******************************************************************************/
/*
bool snakeEstimation::kalmanCorrect(float phi, float theta, float x, float y, float z) {

	if (numLinksAdvanced > 0) {

		gsl_matrix *X_kalm_best = gsl_matrix_calloc(2*numLinksAdvanced+1,1);
		gsl_matrix *X_kalm_new = gsl_matrix_alloc(2*numLinksAdvanced+1,1);
		gsl_matrix *X_kalm_try = gsl_matrix_calloc(2*numLinksAdvanced+1,1);

		gsl_matrix_memcpy(X_kalm_new, X_kalm);
		float bestScore = scoreKalman(phi, theta, x, y, z, X_kalm_new);

		//-----------------------------------------------
		gsl_matrix_memcpy(X_kalm_best, X_kalm_new);
		gsl_matrix_memcpy(X_kalm_try, X_kalm_new);
		for (float changeRoll = (float)-0.015; changeRoll <= (float)0.01; changeRoll += (float)0.015) {
			gsl_matrix_set(X_kalm_try, 0, 0, gsl_matrix_get(X_kalm_new, 0, 0)+changeRoll);
			float testScore = scoreKalman(phi, theta, x, y, z, X_kalm_try);
			if (testScore < bestScore) {
				gsl_matrix_memcpy(X_kalm_best, X_kalm_try);
				bestScore = testScore;
			}
		}
		gsl_matrix_memcpy(X_kalm_new, X_kalm_best);
		//-----------------------------------------------

		gsl_matrix_memcpy(X_kalm_best, X_kalm_new);
		gsl_matrix_memcpy(X_kalm_try, X_kalm_new);
		for (int i = 0; i < numLinksAdvanced; i++) {
			for (int j = 0; j <= 1; j++) {
				for (float change = (float)-0.015; change <= (float)0.015; change += (float)0.01) {
					gsl_matrix_set(X_kalm_try, 2*i+1+j, 0, gsl_matrix_get(X_kalm_new, 2*i+1+j, 0)+change);
					float testScore = scoreKalman(phi, theta, x, y, z, X_kalm_try);
					if (testScore < bestScore) {
						gsl_matrix_memcpy(X_kalm_best, X_kalm_try);
						bestScore = testScore;
					}
				}
				gsl_matrix_memcpy(X_kalm_new, X_kalm_best);
			}
		}
		//-----------------------------------------------

		gsl_matrix_memcpy(X_kalm, X_kalm_new);
		gsl_matrix_free(X_kalm_best);
		gsl_matrix_free(X_kalm_try);
		gsl_matrix_free(X_kalm_new);
	}
	return true;
}
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::drawSnakes()                                               *
*                                                                              *
*  Draws the snake robot estimates.                                            *
*                                                                              *
\******************************************************************************/
/*
bool snakeEstimation::drawSnakes(float phi, float theta) {

	gsl_matrix *linkMatrix = NULL;
	gsl_matrix *rY = NULL;
	gsl_matrix *rZ = NULL;
	gsl_matrix *tZ = NULL;

	if (!initializedFlag) return true;

	// draw advanced links
	for (int i = 0; i < numLinksAdvanced; i++) {
		if (visibleKalm) {
			glColor4f(0.0f, 1.0f, 0.0f, alphaKalm);
			linkMatrix = getLinkTransformationMatrix(init_yaw, init_pitch, init_x, init_y, init_z, numLinksAdvanced, i, X_kalm);
			drawLink(linkMatrix);
			gsl_matrix_free(linkMatrix);
		}
		if (visibleDead) {
			glColor4f(1.0f, 0.0f, 0.0f, alphaDead);
			linkMatrix = getLinkTransformationMatrix(init_yaw, init_pitch, init_x, init_y, init_z, numLinksAdvanced, i, X_dead);
			drawLink(linkMatrix);
			gsl_matrix_free(linkMatrix);		
		}
	}
	if (visibleKalm) {
		glColor4f(0.0f, 1.0f, 0.0f, alphaKalm);
		if (numLinksAdvanced > 0) {
			linkMatrix = getLinkTransformationMatrix(init_yaw, init_pitch, init_x, init_y, init_z, numLinksAdvanced, numLinksAdvanced-1, X_kalm);
		} else {
			linkMatrix = gsl_matrix_alloc(4,4);
			gsl_matrix_set(linkMatrix, 0, 0, -cos(init_yaw)*sin(init_pitch)*cos(0.0)-sin(init_yaw)*sin(0.0));
			gsl_matrix_set(linkMatrix, 0, 1, cos(init_yaw)*sin(init_pitch)*sin(0.0)-sin(init_yaw)*cos(0.0));
			gsl_matrix_set(linkMatrix, 0, 2, cos(init_yaw)*cos(init_pitch));
			gsl_matrix_set(linkMatrix, 0, 3, init_x);
			gsl_matrix_set(linkMatrix, 1, 0, -sin(init_yaw)*sin(init_pitch)*cos(0.0)+cos(init_yaw)*sin(0.0));
			gsl_matrix_set(linkMatrix, 1, 1, sin(init_yaw)*sin(init_pitch)*sin(0.0)+cos(init_yaw)*cos(0.0));
			gsl_matrix_set(linkMatrix, 1, 2, sin(init_yaw)*cos(init_pitch));
			gsl_matrix_set(linkMatrix, 1, 3, init_y);
			gsl_matrix_set(linkMatrix, 2, 0, -cos(init_pitch)*cos(0.0));
			gsl_matrix_set(linkMatrix, 2, 1, cos(init_pitch)*sin(0.0));
			gsl_matrix_set(linkMatrix, 2, 2, -sin(init_pitch));
			gsl_matrix_set(linkMatrix, 2, 3, init_z);
			gsl_matrix_set(linkMatrix, 3, 0, 0.0);
			gsl_matrix_set(linkMatrix, 3, 1, 0.0);
			gsl_matrix_set(linkMatrix, 3, 2, 0.0);
			gsl_matrix_set(linkMatrix, 3, 3, 1.0);

			tZ = createTranslateZmatrix(-3*SNAKELIB_LINK_LENGTH);
			gsl_matrix *newLinkMatrix = gsl_matrix_alloc(4,4);
			gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, tZ, 0.0, newLinkMatrix);
			gsl_matrix_free(tZ);
			gsl_matrix_free(linkMatrix);
			linkMatrix = newLinkMatrix;
		}

		tZ = createTranslateZmatrix(SNAKELIB_LINK_LENGTH);
		gsl_matrix *newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, tZ, 0.0, newLinkMatrix);
		gsl_matrix_free(tZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		rZ = createRotateZmatrix(theta);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rZ, 0.0, newLinkMatrix);
		gsl_matrix_free(rZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		rY = createRotateYmatrix(phi);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rY, 0.0, newLinkMatrix);
		gsl_matrix_free(rY);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		rZ = createRotateZmatrix(-theta);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rZ, 0.0, newLinkMatrix);
		gsl_matrix_free(rZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		drawLink(linkMatrix);
		gsl_matrix_free(linkMatrix);

	}
	if (visibleDead) {
		glColor4f(1.0f, 0.0f, 0.0f, alphaDead);
		if (numLinksAdvanced > 0) {
			linkMatrix = getLinkTransformationMatrix(init_yaw, init_pitch, init_x, init_y, init_z, numLinksAdvanced, numLinksAdvanced-1, X_dead);
		} else {
			linkMatrix = gsl_matrix_alloc(4,4);
			gsl_matrix_set(linkMatrix, 0, 0, -cos(init_yaw)*sin(init_pitch)*cos(0.0)-sin(init_yaw)*sin(0.0));
			gsl_matrix_set(linkMatrix, 0, 1, cos(init_yaw)*sin(init_pitch)*sin(0.0)-sin(init_yaw)*cos(0.0));
			gsl_matrix_set(linkMatrix, 0, 2, cos(init_yaw)*cos(init_pitch));
			gsl_matrix_set(linkMatrix, 0, 3, init_x);
			gsl_matrix_set(linkMatrix, 1, 0, -sin(init_yaw)*sin(init_pitch)*cos(0.0)+cos(init_yaw)*sin(0.0));
			gsl_matrix_set(linkMatrix, 1, 1, sin(init_yaw)*sin(init_pitch)*sin(0.0)+cos(init_yaw)*cos(0.0));
			gsl_matrix_set(linkMatrix, 1, 2, sin(init_yaw)*cos(init_pitch));
			gsl_matrix_set(linkMatrix, 1, 3, init_y);
			gsl_matrix_set(linkMatrix, 2, 0, -cos(init_pitch)*cos(0.0));
			gsl_matrix_set(linkMatrix, 2, 1, cos(init_pitch)*sin(0.0));
			gsl_matrix_set(linkMatrix, 2, 2, -sin(init_pitch));
			gsl_matrix_set(linkMatrix, 2, 3, init_z);
			gsl_matrix_set(linkMatrix, 3, 0, 0.0);
			gsl_matrix_set(linkMatrix, 3, 1, 0.0);
			gsl_matrix_set(linkMatrix, 3, 2, 0.0);
			gsl_matrix_set(linkMatrix, 3, 3, 1.0);

			tZ = createTranslateZmatrix(-3*SNAKELIB_LINK_LENGTH);
			gsl_matrix *newLinkMatrix = gsl_matrix_alloc(4,4);
			gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, tZ, 0.0, newLinkMatrix);
			gsl_matrix_free(tZ);
			gsl_matrix_free(linkMatrix);
			linkMatrix = newLinkMatrix;
		}

		tZ = createTranslateZmatrix(SNAKELIB_LINK_LENGTH);
		gsl_matrix *newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, tZ, 0.0, newLinkMatrix);
		gsl_matrix_free(tZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		rZ = createRotateZmatrix(theta);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rZ, 0.0, newLinkMatrix);
		gsl_matrix_free(rZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		rY = createRotateYmatrix(phi);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rY, 0.0, newLinkMatrix);
		gsl_matrix_free(rY);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		rZ = createRotateZmatrix(-theta);
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rZ, 0.0, newLinkMatrix);
		gsl_matrix_free(rZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		drawLink(linkMatrix);
		gsl_matrix_free(linkMatrix);

	}

	return true;
}
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::getLinkTransformationMatrix()                              *
*                                                                              *
*  This function draws a snake with the state given as an argument.            *
*                                                                              *
\******************************************************************************/
/*
gsl_matrix *snakeEstimation::getLinkTransformationMatrix(float init_yaw, float init_pitch, float init_x, float init_y, float init_z, int numLinks, int link_index, gsl_matrix *state) {
	
	if (link_index < 0 || link_index >= numLinks) return NULL;
	if (!state) return NULL;
	if (numLinks <= 0) return NULL;

	gsl_matrix *linkMatrix = gsl_matrix_alloc(4,4);
	gsl_matrix_set(linkMatrix, 0, 0, -cos(init_yaw)*sin(init_pitch)*cos(gsl_matrix_get(state, 0, 0))-sin(init_yaw)*sin(gsl_matrix_get(state, 0, 0)));
	gsl_matrix_set(linkMatrix, 0, 1, cos(init_yaw)*sin(init_pitch)*sin(gsl_matrix_get(state, 0, 0))-sin(init_yaw)*cos(gsl_matrix_get(state, 0, 0)));
	gsl_matrix_set(linkMatrix, 0, 2, cos(init_yaw)*cos(init_pitch));
	gsl_matrix_set(linkMatrix, 0, 3, init_x);
	gsl_matrix_set(linkMatrix, 1, 0, -sin(init_yaw)*sin(init_pitch)*cos(gsl_matrix_get(state, 0, 0))+cos(init_yaw)*sin(gsl_matrix_get(state, 0, 0)));
	gsl_matrix_set(linkMatrix, 1, 1, sin(init_yaw)*sin(init_pitch)*sin(gsl_matrix_get(state, 0, 0))+cos(init_yaw)*cos(gsl_matrix_get(state, 0, 0)));
	gsl_matrix_set(linkMatrix, 1, 2, sin(init_yaw)*cos(init_pitch));
	gsl_matrix_set(linkMatrix, 1, 3, init_y);
	gsl_matrix_set(linkMatrix, 2, 0, -cos(init_pitch)*cos(gsl_matrix_get(state, 0, 0)));
	gsl_matrix_set(linkMatrix, 2, 1, cos(init_pitch)*sin(gsl_matrix_get(state, 0, 0)));
	gsl_matrix_set(linkMatrix, 2, 2, -sin(init_pitch));
	gsl_matrix_set(linkMatrix, 2, 3, init_z);
	gsl_matrix_set(linkMatrix, 3, 0, 0.0);
	gsl_matrix_set(linkMatrix, 3, 1, 0.0);
	gsl_matrix_set(linkMatrix, 3, 2, 0.0);
	gsl_matrix_set(linkMatrix, 3, 3, 1.0);

	gsl_matrix *tZ = createTranslateZmatrix(-2*SNAKELIB_LINK_LENGTH);
	gsl_matrix *newLinkMatrix = gsl_matrix_alloc(4,4);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, tZ, 0.0, newLinkMatrix);
	gsl_matrix_free(tZ);
	gsl_matrix_free(linkMatrix);
	linkMatrix = newLinkMatrix;
	
	for (int i = 0; i <= link_index; i++) {
		gsl_matrix *rZ = createRotateZmatrix(gsl_matrix_get(state,2*i+2,0));
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rZ, 0.0, newLinkMatrix);
		gsl_matrix_free(rZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		gsl_matrix *rY = createRotateYmatrix(gsl_matrix_get(state,2*i+1,0));
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rY, 0.0, newLinkMatrix);
		gsl_matrix_free(rY);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		rZ = createRotateZmatrix(-gsl_matrix_get(state,2*i+2,0));
		newLinkMatrix = gsl_matrix_alloc(4,4);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, rZ, 0.0, newLinkMatrix);
		gsl_matrix_free(rZ);
		gsl_matrix_free(linkMatrix);
		linkMatrix = newLinkMatrix;

		if (i < link_index) {
			tZ = createTranslateZmatrix(SNAKELIB_LINK_LENGTH);
			newLinkMatrix = gsl_matrix_alloc(4,4);
			gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, linkMatrix, tZ, 0.0, newLinkMatrix);
			gsl_matrix_free(tZ);
			gsl_matrix_free(linkMatrix);
			linkMatrix = newLinkMatrix;
		}
	}
	return linkMatrix;
}
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::destroyEstimation()                                        *
*                                                                              *
*  This function destroys the estimation.                                      *
*                                                                              *
\******************************************************************************/
/*
bool snakeEstimation::destroyEstimation(void) {
	if (X_kalm) gsl_matrix_free(X_kalm); X_kalm = NULL;
	if (X_saved) gsl_matrix_free(X_saved); X_saved = NULL;
	if (X_dead) gsl_matrix_free(X_dead); X_dead = NULL;
	if (P_kalm) gsl_matrix_free(P_kalm); P_kalm = NULL;
	numLinksAdvanced = 0;
	init_x = init_y = init_z = init_pitch = init_yaw = 0;
	initializedFlag = false;
	return true;
}
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::initializeEstimation()                                     *
*                                                                              *
*  This function initializes the origin of the estimation problem.             *
*                                                                              *
\******************************************************************************/
/*
bool snakeEstimation::initializeEstimation(float x, float y, float z, float pitch, float yaw) {
	init_x = x; init_y = y; init_z = z;
	init_pitch = pitch; init_yaw = yaw;
	initializedFlag = true;
	return true;
}
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::isInitialized()                                            *
*                                                                              *
*  This function returns the state of initializedFlag.                         *
*                                                                              *
\******************************************************************************/
/*
bool snakeEstimation::isInitialized(void) {
	return initializedFlag;
}
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::removeLinkFromState()                                      *
*                                                                              *
*  This function removes a link from the state.                                *
*                                                                              *
\******************************************************************************/
/*
bool snakeEstimation::removeLinkFromState(void) {
	if (numLinksAdvanced > 1) {
		gsl_matrix *X_kalm_new = gsl_matrix_alloc(2*(numLinksAdvanced-1)+1,1);
		gsl_matrix *X_saved_new = gsl_matrix_alloc(2*(numLinksAdvanced-1)+1,1);
		gsl_matrix *X_dead_new = gsl_matrix_alloc(2*(numLinksAdvanced-1)+1,1);
		gsl_matrix *P_kalm_new = gsl_matrix_alloc(2*(numLinksAdvanced-1)+1,2*(numLinksAdvanced-1)+1);
		memcpy(X_kalm_new->data, X_kalm->data, sizeof(double)*(2*(numLinksAdvanced-1)+1));
		memcpy(X_saved_new->data, X_saved->data, sizeof(double)*(2*(numLinksAdvanced-1)+1));
		memcpy(X_dead_new->data, X_dead->data, sizeof(double)*(2*(numLinksAdvanced-1)+1));
		for (int i = 0; i < 2*(numLinksAdvanced-1)+1; i++) {
			for (int j = 0; j < 2*(numLinksAdvanced-1)+1; j++) {
				gsl_matrix_set(P_kalm_new, i, j, gsl_matrix_get(P_kalm, i, j));
			}
		}		
		if (X_kalm) gsl_matrix_free(X_kalm);
		if (X_saved) gsl_matrix_free(X_saved);
		if (P_kalm) gsl_matrix_free(P_kalm);
		if (X_dead) gsl_matrix_free(X_dead);
		X_kalm = X_kalm_new; X_dead = X_dead_new; P_kalm = P_kalm_new;
		X_saved = X_saved_new;
		numLinksAdvanced--;
	} else if (numLinksAdvanced == 1) {
		if (X_kalm) gsl_matrix_free(X_kalm); X_kalm = NULL;
		if (X_saved) gsl_matrix_free(X_saved); X_saved = NULL;
		if (P_kalm) gsl_matrix_free(P_kalm); P_kalm = NULL;
		if (X_dead) gsl_matrix_free(X_dead); X_dead = NULL;
		numLinksAdvanced = 0;
	} else {
		printf("\tError: retracting snake estimation, tried to remove too many links.\n");
		return false;
	}
	return true;
}
*/

/******************************************************************************\
*                                                                              *
*  snakeEstimation::addLinkToState()                                          *
*                                                                              *
*  This function adds a link with the angles provided.                         *
*                                                                              *
\******************************************************************************/
/*
bool snakeEstimation::addLinkToState(float phi, float theta) {

	if (!initializedFlag) {
		printf("\tError: could not add link to snake, not initialized.\n\n");
		return false;
	} else if (numLinksAdvanced == 0) {
		X_kalm = gsl_matrix_calloc(3,1); X_dead = gsl_matrix_calloc(3,1); X_saved = gsl_matrix_calloc(3,1);
		gsl_matrix_set(X_kalm, 0, 0, 0.0); gsl_matrix_set(X_kalm, 1, 0, (double)phi); gsl_matrix_set(X_kalm, 2, 0, (double)theta);
		gsl_matrix_set(X_dead, 0, 0, 0.0); gsl_matrix_set(X_dead, 1, 0, (double)phi); gsl_matrix_set(X_dead, 2, 0, (double)theta);
		gsl_matrix_set(X_saved, 0, 0, 0.0); gsl_matrix_set(X_saved, 1, 0, (double)phi); gsl_matrix_set(X_saved, 2, 0, (double)theta);
		P_kalm = gsl_matrix_calloc(3,3);
	} else {
		gsl_matrix *X_kalm_new = gsl_matrix_alloc(2*(numLinksAdvanced+1)+1,1);
		gsl_matrix *X_dead_new = gsl_matrix_alloc(2*(numLinksAdvanced+1)+1,1);
		gsl_matrix *X_saved_new = gsl_matrix_alloc(2*(numLinksAdvanced+1)+1,1);
		memcpy(X_kalm_new->data, X_kalm->data, sizeof(double)*(2*numLinksAdvanced+1));
		memcpy(X_dead_new->data, X_dead->data, sizeof(double)*(2*numLinksAdvanced+1));
		memcpy(X_saved_new->data, X_kalm->data, sizeof(double)*(2*numLinksAdvanced+1));
		gsl_matrix_set(X_kalm_new, 2*numLinksAdvanced+1, 0, (double)phi); gsl_matrix_set(X_kalm_new, 2*numLinksAdvanced+2, 0, (double)theta);
		gsl_matrix_set(X_saved_new, 2*numLinksAdvanced+1, 0, (double)phi); gsl_matrix_set(X_saved_new, 2*numLinksAdvanced+2, 0, (double)theta);
		gsl_matrix_set(X_dead_new, 2*numLinksAdvanced+1, 0, (double)phi); gsl_matrix_set(X_dead_new, 2*numLinksAdvanced+2, 0, (double)theta);
		gsl_matrix_free(X_kalm); gsl_matrix_free(X_dead); gsl_matrix_free(X_saved);
		X_kalm = X_kalm_new; X_dead = X_dead_new; X_saved = X_saved_new;
		gsl_matrix *P_kalm_new = gsl_matrix_calloc(2*(numLinksAdvanced+1)+1,2*(numLinksAdvanced+1)+1);
		gsl_matrix_free(P_kalm);
		P_kalm = P_kalm_new;
	}
	numLinksAdvanced++;
	return true;
}
*/
