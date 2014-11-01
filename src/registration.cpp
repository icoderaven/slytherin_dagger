/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  registration.cpp                                                                     |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions dealing with registration between 3D models and the sensor.   |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#include <windows.h>
#include <math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include "snakelibgl.h"
#include "registration.h"

/******************************************************************************\
*                                                                              *
*  registration::logData()                                                     *
*                                                                              *
*  This function logs the important data.                                      *
*                                                                              *
\******************************************************************************/
bool registration::logData(FILE *fptr, double timeStamp, int dataLabel) {
	if (loggingFlag) {
		fprintf_s(fptr, "%d %lf registration numModelPoints %d numTrackerPoints %d avgRegistrationError %f", dataLabel, timeStamp, numModelPoints, numTrackerPoints, avgRegistrationError);
		if (numTrackerPoints > 0) {
			fprintf_s(fptr, " trackerPoints");
			for (int i = 0; i < numTrackerPoints; i++) {
				fprintf_s(fptr, " %lf %lf %lf", gsl_matrix_get(trackerPoints, 0, i), gsl_matrix_get(trackerPoints, 1, i), gsl_matrix_get(trackerPoints, 2, i));
			}
		}
		if (numModelPoints > 0) {
			fprintf_s(fptr, " modelPoints");
			for (int i = 0; i < numModelPoints; i++) {
				fprintf_s(fptr, " %lf %lf %lf", gsl_matrix_get(modelPoints, 0, i), gsl_matrix_get(modelPoints, 1, i), gsl_matrix_get(modelPoints, 2, i));
			}
		}
		if (T_tracker2model) {
			fprintf_s(fptr, " T_tracker2model");
			for (int j = 0; j < (int)T_tracker2model->size1; j++) {
				for (int k = 0; k < (int)T_tracker2model->size2; k++) {
					fprintf_s(fptr, " %lf", gsl_matrix_get(T_tracker2model, j, k));
				}
			}
		} fprintf_s(fptr, "\n");
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::setLoggingFlag()                                              *
*                                                                              *
*  This function sets the logging flag.                                        *
*                                                                              *
\******************************************************************************/
void registration::setLoggingFlag(bool val) {
	loggingFlag = val;
}

/******************************************************************************\
*                                                                              *
*  registration::logTransformationMatrix()                                     *
*                                                                              *
*  This function logs the transformation matrix to file.                       *
*                                                                              *
\******************************************************************************/
bool registration::logTransformationMatrix() {
	if (!T_tracker2model) {
		printf("Error: registration, could not save transformation matrix to file.\n\n");
		return false;
	}
	FILE *myFP = NULL;
	fopen_s(&myFP, "T_tracker2model.dat", "w");
	if (!myFP) {
		printf("Error: registration, could not save transformation matrix to file.\n\n");
		return false;
	}
	if (T_tracker2model->size1 != 4 || T_tracker2model->size2 != 4) {
		printf("Error: registration, logging transformation matrix failed, matrix size wrong.\n\n");
		return false;
	}
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			fprintf_s(myFP, "%f ", (float)gsl_matrix_get(T_tracker2model,i,j));
		} fprintf_s(myFP, "\n");
	}
	fclose(myFP);
	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::get_T_tracker2model()                                         *
*                                                                              *
*  This function returns the transformation matrix.                            *
*                                                                              *
\******************************************************************************/
bool registration::get_T_tracker2model(gsl_matrix *T_return) {
	if (!T_return || !T_tracker2model) {
		return false;
	} else if (T_return->size1 != 4 || T_return->size2 != 4) {
		printf("Error: registration, getting transformation matrix failed, return matrix size wrong.\n\n");
		return false;
	} else if (T_tracker2model->size1 != 4 || T_tracker2model->size2 != 4) {
		printf("Error: registration, getting transformation matrix failed, source matrix size wrong.\n\n");
		return false;
	} else {
		gsl_matrix_memcpy(T_return, T_tracker2model);
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::get_numTrackerPoints()                                         *
*                                                                              *
*  This function returns the number of tracker points that have been added.    *
*                                                                              *
\******************************************************************************/
int registration::get_numTrackerPoints() {
	return numTrackerPoints;
}

/******************************************************************************\
*                                                                              *
*  registration::get_numModelPoints()                                           *
*                                                                              *
*  This function returns the number of model points that have been loaded.     *
*                                                                              *
\******************************************************************************/
int registration::get_numModelPoints() {
	return numModelPoints;
}

/******************************************************************************\
*                                                                              *
*  registration::get_avgRegistrationError()                                    *
*                                                                              *
*  This function returns the average registration error.                       *
*                                                                              *
\******************************************************************************/
float registration::get_avgRegistrationError() {
	return avgRegistrationError;
}

/******************************************************************************\
*                                                                              *
*  registration::clearModelPoints()                                            *
*                                                                              *
*  This function clears the model points that have been added to this class    *
*     for registration.                                                        *
*                                                                              *
\******************************************************************************/
bool registration::clearModelPoints() {
	if (T_tracker2model) gsl_matrix_free(T_tracker2model);
	T_tracker2model = NULL;
	if (modelPoints) gsl_matrix_free(modelPoints);
	modelPoints = NULL;
	numModelPoints = 0;
	avgRegistrationError = 0.0;
	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::clearTrackerPoints()                                          *
*                                                                              *
*  This function clears the tracker points that have been added to this class  *
*     for registration.                                                        *
*                                                                              *
\******************************************************************************/
bool registration::clearTrackerPoints() {
	if (T_tracker2model) gsl_matrix_free(T_tracker2model);
	T_tracker2model = NULL;
	if (trackerPoints) gsl_matrix_free(trackerPoints);
	trackerPoints = NULL;
	numTrackerPoints = 0;
	avgRegistrationError = 0.0;
	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::registration()                                                *
*                                                                              *
*  This function is the constructor for the registration class.                *
*                                                                              *
\******************************************************************************/
registration::registration() {
	T_tracker2model = NULL;
	modelPoints = NULL;
	trackerPoints = NULL;
	numModelPoints = 0;
	numTrackerPoints = 0;
	avgRegistrationError = 0.0;
	loggingFlag = true;
}

/******************************************************************************\
*                                                                              *
*  registration::~registration()                                               *
*                                                                              *
*  This function is the destructor for the registration class.                 *
*                                                                              *
\******************************************************************************/
registration::~registration() {
	if (T_tracker2model) gsl_matrix_free(T_tracker2model);
	if (modelPoints) gsl_matrix_free(modelPoints);
	if (trackerPoints) gsl_matrix_free(trackerPoints);
	T_tracker2model = NULL;
	modelPoints = NULL;
	trackerPoints = NULL;
	numModelPoints = 0;
	numTrackerPoints = 0;
	avgRegistrationError = 0.0;
}

/******************************************************************************\
*                                                                              *
*  registration::registerPoints()                                              *
*                                                                              *
*  This function registers a set of points with another set and returns        *
*     the transformation matrix that transforms set 1 into set 2. In this case *
*     we are transforming the trackerPoints into the trackerPoints.            *
*                                                                              *
\******************************************************************************/
bool registration::registerPoints() {

	// check that the arguments are valid
	if (numModelPoints < 3 || numTrackerPoints < 3) {
		printf("\tError: registration, either not enough model points or not enough tracker points.\n\n");
		return false;
	} else if (numTrackerPoints != numModelPoints) {
		printf("\tError: registration, num model points unequal to num tracker points.\n\n");
		return false;
	} else if (modelPoints == NULL || trackerPoints == NULL) {
		printf("\tError: registration, either no model point given or no tracker points given.\n\n");		
		return false;
	}

	gsl_matrix *modelPoints_centered = gsl_matrix_alloc(3,numModelPoints);		// centered version of the modelPoints set
	gsl_matrix *trackerPoints_centered = gsl_matrix_alloc(3,numTrackerPoints);	// centered version of the trackerPoints set
	gsl_matrix *H = gsl_matrix_calloc(3,3);										// the points covariance
	gsl_matrix *q1 = gsl_matrix_alloc(3,1);										// one instance centered trackerPoints point
	gsl_matrix *q2 = gsl_matrix_alloc(3,1);										// one instance centered modelPoints point
	gsl_matrix *q1q2T = gsl_matrix_alloc(3,3);									// q1 x q2^T
	gsl_matrix *T = gsl_matrix_calloc(4,4);										// the transformation matrix we want
	gsl_matrix *V = gsl_matrix_alloc(3,3);										// V in the SVD: USV^T for matrix H
	gsl_vector *S = gsl_vector_alloc(3);										// S in the SVD: USV^T for matrix H
	gsl_vector *work = gsl_vector_alloc(3);										// extra work space for the SVD
	gsl_matrix *R = gsl_matrix_alloc(3,3);										// the rotation matrix VU^T
	gsl_matrix *t = gsl_matrix_calloc(3,1);										// the translation component of T
	gsl_matrix *RtrackerPoints = gsl_matrix_alloc(3,numTrackerPoints);			// temp matrix for R * modelPoints_centered
	gsl_matrix *modelPoints_copy = gsl_matrix_alloc(3,numModelPoints);			// a copy of the uncentered trackerPoints

	memcpy(modelPoints_centered->data, modelPoints->data, sizeof(double)*numModelPoints*3);
	memcpy(trackerPoints_centered->data, trackerPoints->data, sizeof(double)*numTrackerPoints*3);
	double modelPoints_centroid[3] = {0, 0, 0};
	double trackerPoints_centroid[3] = {0, 0, 0};

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < numModelPoints; i++) {
			modelPoints_centroid[j] += gsl_matrix_get(modelPoints_centered, j, i);
			trackerPoints_centroid[j] += gsl_matrix_get(trackerPoints_centered, j, i);
		}
		modelPoints_centroid[j] /= (double)numModelPoints;
		trackerPoints_centroid[j] /= (double)numTrackerPoints;
	}

	for (int i = 0; i < numModelPoints; i++) {
		for (int j = 0; j < 3; j++) {
			gsl_matrix_set(modelPoints_centered,j,i,gsl_matrix_get(modelPoints_centered,j,i)-modelPoints_centroid[j]);
			gsl_matrix_set(trackerPoints_centered,j,i,gsl_matrix_get(trackerPoints_centered,j,i)-trackerPoints_centroid[j]);
		}
	}

	for (int i = 0; i < numModelPoints; i++) {
		for (int j = 0; j < 3; j++) {
			gsl_matrix_set(q1,j,0,gsl_matrix_get(trackerPoints_centered,j,i));
			gsl_matrix_set(q2,j,0,gsl_matrix_get(modelPoints_centered,j,i));
		}
		gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, q1, q2, 0.0, q1q2T);
		gsl_matrix_add(H,q1q2T);
	}

	gsl_linalg_SV_decomp(H, V, S, work);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, V, H, 0.0, R);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, R, trackerPoints, 0.0, RtrackerPoints);
	memcpy(modelPoints_copy->data, modelPoints->data, sizeof(double)*numModelPoints*3);
	gsl_matrix_sub(modelPoints_copy, RtrackerPoints);
	
	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < numModelPoints; i++) {
			t->data[j] += gsl_matrix_get(modelPoints_copy, j, i);
		}
		t->data[j] /= numModelPoints;
		gsl_matrix_set(T,j,3, t->data[j]);
	}

	avgRegistrationError = 0.0;
	for (int i = 0; i < numModelPoints; i++) {
		avgRegistrationError += (float)sqrt((gsl_matrix_get(modelPoints_copy, 0, i) - t->data[0])*(gsl_matrix_get(modelPoints_copy, 0, i) - t->data[0]) +
									 (gsl_matrix_get(modelPoints_copy, 1, i) - t->data[1])*(gsl_matrix_get(modelPoints_copy, 1, i) - t->data[1]) +
									 (gsl_matrix_get(modelPoints_copy, 2, i) - t->data[2])*(gsl_matrix_get(modelPoints_copy, 2, i) - t->data[2]));
	}
	avgRegistrationError /= numModelPoints;

	gsl_matrix_set(T,3,3,1.0);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			gsl_matrix_set(T,i,j,gsl_matrix_get(R,i,j));
		}
	}

	gsl_matrix_free(modelPoints_centered);
	gsl_matrix_free(trackerPoints_centered);
	gsl_matrix_free(H);
	gsl_matrix_free(q1);
	gsl_matrix_free(q2);
	gsl_matrix_free(q1q2T);
	gsl_matrix_free(V);
	gsl_vector_free(S);
	gsl_vector_free(work);
	gsl_matrix_free(R);
	gsl_matrix_free(t);
	gsl_matrix_free(RtrackerPoints);
	gsl_matrix_free(modelPoints_copy);

	if (T_tracker2model) gsl_matrix_free(T_tracker2model);
	T_tracker2model = T;

	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::saveTrackerPointsToFile()                                     *
*                                                                              *
*  This function saves to file points that correspond to the registration      *
*     landmarks in the coordinate frame of the tracker.                        *
*                                                                              *
\******************************************************************************/
bool registration::saveTrackerPointsToFile(char *filename) {

	FILE *myFP = NULL;
	fopen_s(&myFP, filename, "w");
	if (!myFP) {
		printf("\tError: registration, could not open tracker registration filename for writing.\n\n");
		return false;
	}
	fprintf_s(myFP, "%d\n", numTrackerPoints);
	for (int i = 0; i < numTrackerPoints; i++) {
		fprintf_s(myFP, "%f %f %f\n", gsl_matrix_get(trackerPoints, 0, i), gsl_matrix_get(trackerPoints, 1, i), gsl_matrix_get(trackerPoints, 2, i));
	}
	fclose(myFP);
	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::loadTrackerPointsFromFile()                                   *
*                                                                              *
*  This function loads from file points that correspond to the registration    *
*     landmarks in the coordinate frame of the tracker.                        *
*                                                                              *
\******************************************************************************/
bool registration::loadTrackerPointsFromFile(char *filename) {

	gsl_matrix *points = NULL;
	int numPointsToRead = 0;
	FILE *myFP;
	char *context, *tok, lineArray[MAX_TEXT_LENGTH];
	fopen_s(&myFP, filename, "r");
	if (!myFP) {
		printf("\tError: registration, could not open tracker registration file for reading.\n\n");
		return false;
	}
	if (!fgets(lineArray, MAX_TEXT_LENGTH, myFP)) {
		printf("\tError: registration, something went wrong reading num points from tracker registration file.\n\n");
		return false;
	}
	numPointsToRead = atoi(lineArray);
	points = gsl_matrix_calloc(3,numPointsToRead);
	if (!points) {
		printf("\tError: registration, could not allocate memory for points loading tracker points.\n\n");
		return false;
	}
	for (int i = 0; i < numPointsToRead; i++) {
		if (!fgets(lineArray, MAX_TEXT_LENGTH, myFP)) {
			gsl_matrix_free(points);
			printf("\tError: registration, something went wrong reading tracker registration file (1).\n\n");
			fclose(myFP); return false;
		}
		if (!(tok = strtok_s(lineArray, " \r\n", &context))) {
			gsl_matrix_free(points);
			printf("\tError: registration, something went wrong reading tracker registration file (2).\n\n");
			fclose(myFP); return false;
		}
		gsl_matrix_set(points, 0, i, atof(tok));
		if (!(tok = strtok_s(NULL, " \r\n", &context))) {
			gsl_matrix_free(points);
			printf("\tError: registration, something went wrong reading tracker registration file (3).\n\n");
			fclose(myFP); return false;
		}
		gsl_matrix_set(points, 1, i, atof(tok));
		if (!(tok = strtok_s(NULL, " \r\n", &context))) {
			gsl_matrix_free(points);
			printf("\tError: registration, something went wrong reading tracker registration file (4).\n\n");
			fclose(myFP); return false;
		}
		gsl_matrix_set(points, 2, i, atof(tok));
	}
	if (trackerPoints) gsl_matrix_free(trackerPoints);
	trackerPoints = points;
	numTrackerPoints = numPointsToRead;
	fclose(myFP);
	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::loadModelPointsFromFile()                                     *
*                                                                              *
*  This function loads from file points that correspond to the registration    *
*     landmarks in the coordinate frame of the 3D models.                      *
*                                                                              *
\******************************************************************************/
bool registration::loadModelPointsFromFile(char *filename) {

	gsl_matrix *points = NULL;
	int numPointsToRead = 0;
	FILE *myFP;
	char *context, *tok, lineArray[MAX_TEXT_LENGTH];
	fopen_s(&myFP, filename, "r");
	if (!myFP) {
		printf("\tError: registration, could not open model registration file for reading.\n\n");
		return false;
	}
	if (!fgets(lineArray, MAX_TEXT_LENGTH, myFP)) {
		printf("\tError: registration, something went wrong reading num points from model registration file.\n\n");
		return false;
	}
	numPointsToRead = atoi(lineArray);
	points = gsl_matrix_calloc(3,numPointsToRead);
	if (!points) {
		printf("\tError: registration, could not allocate memory for points loading model points.\n\n");
		return false;
	}
	for (int i = 0; i < numPointsToRead; i++) {
		if (!fgets(lineArray, MAX_TEXT_LENGTH, myFP)) {
			gsl_matrix_free(points);
			printf("\tError: registration, something went wrong reading model registration file (1).\n\n");
			fclose(myFP); return false;
		}
		if (!(tok = strtok_s(lineArray, " \r\n", &context))) {
			gsl_matrix_free(points);
			printf("\tError: registration, something went wrong reading model registration file (2).\n\n");
			fclose(myFP); return false;
		}
		gsl_matrix_set(points, 0, i, atof(tok));
		if (!(tok = strtok_s(NULL, " \r\n", &context))) {
			gsl_matrix_free(points);
			printf("\tError: registration, something went wrong reading model registration file (3).\n\n");
			fclose(myFP); return false;
		}
		gsl_matrix_set(points, 1, i, atof(tok));
		if (!(tok = strtok_s(NULL, " \r\n", &context))) {
			gsl_matrix_free(points);
			printf("\tError: registration, something went wrong reading model registration file (4).\n\n");
			fclose(myFP); return false;
		}
		gsl_matrix_set(points, 2, i, atof(tok));
	}
	if (modelPoints) gsl_matrix_free(modelPoints);
	modelPoints = points;
	numModelPoints = numPointsToRead;
	fclose(myFP);
	return true;
}

/******************************************************************************\
*                                                                              *
*  registration::addPointToTrackerPoints()                                     *
*                                                                              *
*  This function adds a point (from the argument) to the list of points        *
*     corresponding to the landmarks in the frame of the tracker.              *
*                                                                              *
\******************************************************************************/
bool registration::addPointToTrackerPoints(double *point) {

	if (!point) {
		printf("\tError: registration, point to add was NULL.\n\n");
		return false;
	}
	numTrackerPoints++;
	gsl_matrix *points = gsl_matrix_calloc(3,numTrackerPoints);
	if (!points) {
		numTrackerPoints--;
		printf("\tError: registration, could not allocate memory for points.\n\n");
		return false;
	}
	for (int i = 0; i < numTrackerPoints-1; i++) {
		for (int j = 0; j < 3; j++) {
			gsl_matrix_set(points, j, i, gsl_matrix_get(trackerPoints, j, i));
		}
	}
	for (int j = 0; j < 3; j++) {
		gsl_matrix_set(points, j, numTrackerPoints-1, point[j]);
	}
	if (trackerPoints) gsl_matrix_free(trackerPoints);
	trackerPoints = points;
	return true;
}


