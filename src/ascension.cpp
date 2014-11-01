/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  ascension.cpp                                                                        |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Custom functions to interface with the Ascension trakSTAR tracker.      |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#include <windows.h>
#include <math.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include "ATC3DGM.h"
#include <gsl/gsl_blas.h>
#include "snakelibgl.h"
#include "ascension.h"

/******************************************************************************\
*                                                                              *
*  ascensionTracker::logData()                                                 *
*                                                                              *
*  This function logs the important data.                                      *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::logData(FILE *fptr, double timeStamp, int dataLabel) {
	if (loggingFlag) {
		fprintf_s(fptr, "%d %lf ascensionTracker numSensors %d", dataLabel, timeStamp, numSensors);
		if (numSensors > 0) {
			for (int i = 0; i < numSensors; i++) {
				if (isSensorAttached(i)) {
					fprintf_s(fptr, " sensorIndex %d Tsensor", i);
					for (int j = 0; j < (int)Ts[i]->size1; j++) {
						for (int k = 0; k < (int)Ts[i]->size2; k++) {
							fprintf_s(fptr, " %lf", gsl_matrix_get(Ts[i], j, k));
						}
					}
				}
			}
		} fprintf_s(fptr, "\n");
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::getSensorTrailsTriggerOnlyFlag()                          *
*                                                                              *
*  This function returns the sensorTrailsTriggerOnlyFlag.                      *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::getSensorTrailsTriggerOnlyFlag() {
	return sensorTrailsTriggerOnlyFlag;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setSensorTrailsTriggerOnlyFlag()                          *
*                                                                              *
*  This function sets the sensorTrailsTriggerOnlyFlag.                         *
*                                                                              *
\******************************************************************************/
void ascensionTracker::setSensorTrailsTriggerOnlyFlag(bool val) {
	sensorTrailsTriggerOnlyFlag = val;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setLoggingFlag()                                          *
*                                                                              *
*  This function sets the logging flag.                                        *
*                                                                              *
\******************************************************************************/
void ascensionTracker::setLoggingFlag(bool val) {
	loggingFlag = val;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setContinuouslyLogging()                                  *
*                                                                              *
*  This function sets either NULL to cancel continuously logging, or           *
*     adds a filename to the array store privately here to enable it.          *
*                                                                              *
\******************************************************************************/
void ascensionTracker::setContinuouslyLogging(char *filename) {
	if (continuouslyLoggingFilename) free(continuouslyLoggingFilename);
	continuouslyLoggingFilename = NULL;
	if (filename == NULL) {
		continuouslyLoggingFilename = NULL;
	} else if (numSensors > 0) {
		int numSensorsAttached = 0;
		for (int i = 0; i < numSensors; i++) {
			if (sensorAttachedFlags[i]) numSensorsAttached++;
		}
		if (numSensorsAttached > 0) {
			continuouslyLoggingFilename = (char *)malloc(sizeof(char)*(strlen(filename)+1));
			strcpy_s(continuouslyLoggingFilename, sizeof(char)*(strlen(filename)+1), filename);
		} else {
			printf("\tError: setting continuous ascension log, no attached sensors.\n\n");
		}
	} else {
		printf("\tError: setting continuous ascension log, no ascension tracker.\n\n");
	}
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::isContinuouslyLogging()                                   *
*                                                                              *
*  This function returns whether we are continuously logging the tracker.      *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::isContinuouslyLogging() {
	if (continuouslyLoggingFilename) return true;
	else return false;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setVisibleTracker()                                       *
*                                                                              *
*  This function sets the visible tracker flag.                                *
*                                                                              *
\******************************************************************************/
void ascensionTracker::setVisibleTracker(bool val) {
	visibleTracker = val;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setVisibleTrails()                                        *
*                                                                              *
*  This function sets the visible trails flag.                                 *
*                                                                              *
\******************************************************************************/
void ascensionTracker::setVisibleTrails(bool val) {
	visibleTrails = val;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::getTrailCapturingFlag()                                   *
*                                                                              *
*  This function gets the trail capturing flag.                                *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::getTrailCapturingFlag() {
	return trailCapturingFlag;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setTrailCapturingFlag()                                   *
*                                                                              *
*  This function sets the trail capturing flag.                                *
*                                                                              *
\******************************************************************************/
void ascensionTracker::setTrailCapturingFlag(bool val) {
	trailCapturingFlag = val;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::saveTrailsData()                                          *
*                                                                              *
*  This function draws the tracker points collected for the trails.            *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::saveTrailsData(char *filename) {
	if (numTrailPosesAdvance > 0 || numTrailPosesRetract > 0) {
		FILE *myFP = NULL;
		fopen_s(&myFP, filename, "w");
		if (!myFP) {
			printf("\tError: saving trails, could not open file.\n\n");
			return false;
		}
		for (int i = 0; i < numTrailPosesAdvance; i++) {
			fprintf_s(myFP, "advance %d ", i);
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					fprintf_s(myFP, "%f ", (float)(gsl_matrix_get(Ts_trail_advance[i], j, k)));
				}
			} fprintf_s(myFP, "\n");
		}
		for (int i = 0; i < numTrailPosesRetract; i++) {
			fprintf_s(myFP, "retract %d ", i);
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					fprintf_s(myFP, "%f ", (float)(gsl_matrix_get(Ts_trail_retract[i], j, k)));
				}
			} fprintf_s(myFP, "\n");
		}
		fclose(myFP);
	} else {
		printf("\tError: saving trails, no data to save.\n\n");
		return false;
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::drawTrails()                                              *
*                                                                              *
*  This function draws the tracker points collected for the trails.            *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::drawTrails(gsl_matrix *Ttransform) {	
	if (!visibleTrails) return true;
	float colorAdvance[3] = {0.0, 1.0, 1.0};
	for (int i = 0; i < numTrailPosesAdvance; i++)
		drawPose(Ts_trail_advance[i], Ttransform, colorAdvance, alphaTrails, trailsDrawingSize);
	float colorRetract[3] = {1.0, 0.0, 1.0};
	for (int i = 0; i < numTrailPosesRetract; i++)
		drawPose(Ts_trail_retract[i], Ttransform, colorRetract, alphaTrails, trailsDrawingSize);
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::clearTrails()                                             *
*                                                                              *
*  This function clears the data from the trail arrays.                        *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::clearTrails() {
	for (int i = 0; i < numTrailPosesAdvance; i++)
		if (Ts_trail_advance[i]) gsl_matrix_free(Ts_trail_advance[i]);
	for (int i = 0; i < numTrailPosesRetract; i++)
		if (Ts_trail_retract[i]) gsl_matrix_free(Ts_trail_retract[i]);
	if (Ts_trail_advance) free(Ts_trail_advance);
	Ts_trail_advance = NULL;
	if (Ts_trail_retract) free(Ts_trail_retract);
	Ts_trail_retract = NULL;
	numTrailPosesAdvance = 0;
	numTrailPosesRetract = 0;
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::addToTrail()                                              *
*                                                                              *
*  This function adds a data point to the trail arrays.                        *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::addToTrail(bool advanceFlag) {
	if (numSensors <= 0) return false;
	gsl_matrix *T = NULL;
	for (int i = 0; i < numSensors; i++) {
		if (sensorAttachedFlags[i]) {
			T = Ts[i];
			break;
		}
	}
	if (!T) return false;
	if (advanceFlag) {
		gsl_matrix **Ts_trail_advance_new = (gsl_matrix **)malloc(sizeof(gsl_matrix *)*(numTrailPosesAdvance+1));
		if (numTrailPosesAdvance > 0) memcpy_s(Ts_trail_advance_new, sizeof(gsl_matrix *)*numTrailPosesAdvance, Ts_trail_advance, sizeof(gsl_matrix *)*numTrailPosesAdvance);
		Ts_trail_advance_new[numTrailPosesAdvance] = gsl_matrix_alloc(4,4);
		gsl_matrix_memcpy(Ts_trail_advance_new[numTrailPosesAdvance],T);
		if (Ts_trail_advance) free(Ts_trail_advance);
		Ts_trail_advance = Ts_trail_advance_new;
		numTrailPosesAdvance++;
	} else {
		gsl_matrix **Ts_trail_retract_new = (gsl_matrix **)malloc(sizeof(gsl_matrix *)*(numTrailPosesRetract+1));
		if (numTrailPosesRetract > 0) memcpy_s(Ts_trail_retract_new, sizeof(gsl_matrix *)*numTrailPosesRetract, Ts_trail_retract, sizeof(gsl_matrix *)*numTrailPosesRetract);
		Ts_trail_retract_new[numTrailPosesRetract] = gsl_matrix_alloc(4,4);
		gsl_matrix_memcpy(Ts_trail_retract_new[numTrailPosesRetract],T);
		if (Ts_trail_retract) free(Ts_trail_retract);
		Ts_trail_retract = Ts_trail_retract_new;
		numTrailPosesRetract++;
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setAlphaTrails()                                          *
*                                                                              *
*  This function sets the transparency alpha for drawing the trails.           *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::setAlphaTrails(float val) {
	if (val < 0.0) return false;
	else if (val > 1.0) return false;
	else alphaTrails = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setTrailsDrawingSize()                                    *
*                                                                              *
*  This function sets the size of the trails diameter for drawing, in mm.      *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::setTrailsDrawingSize(float size) {
	if (size <= 0.0) return false;
	else trailsDrawingSize = size;
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setAlphaTracker()                                         *
*                                                                              *
*  This function sets the transparency alpha for drawing the tracker.          *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::setAlphaTracker(float val) {
	if (val < 0.0) return false;
	else if (val > 1.0) return false;
	else alphaTracker = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::setTrackerDrawingSize()                                   *
*                                                                              *
*  This function sets the size of the tracker diameter for drawing, in mm.     *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::setTrackerDrawingSize(float size) {
	if (size <= 0.0) return false;
	else trackerDrawingSize = size;
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::getNumSensors()                                           *
*                                                                              *
*  This function returns the number of active tracking sensors                 *
*                                                                              *
\******************************************************************************/
int ascensionTracker::getNumSensors(void) {
	return numSensors;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::logAppendTrackerPose()                                    *
*                                                                              *
*  This function is loads the provided filename for appending and adds a       *
*     line to the file that specifies the pose of the tracker.                 *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::logAppendTrackerPose(char *filename, double timeStamp) {

	if (numSensors <= 0) {
		printf("\tError: ascensionTracker, could not log tracker points, no tracker.\n\n");
		return false;
	}

	FILE *myFP = NULL;
	fopen_s(&myFP, filename, "a+");
	if (!myFP) {
		printf("\tError: ascensionTracker, could not open file for appending tracker pose.\n\n");
		return false;
	}

	for (int i = 0; i < numSensors; i++) {
		if (sensorAttachedFlags[i]) {
			fprintf_s(myFP, "%d %f ", i, timeStamp);
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					fprintf_s(myFP, "%f ", ((float)gsl_matrix_get(Ts[i], j, k)));
				}
			}
			fprintf_s(myFP, "\n");
		}
	}
	fclose(myFP);
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::ascensionTracker()                                        *
*                                                                              *
*  This function is the constructor for the ascensionTracker class. The        *
*     function calls the init() function to intialize variables.               *
*                                                                              *
\******************************************************************************/
ascensionTracker::ascensionTracker() {
	init();
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::~ascensionTracker()                                       *
*                                                                              *
*  This function is the destructor for the ascensionTracker class. The         *
*     function calls the destroyMemory() function to clear allocated memory.   *
*                                                                              *
\******************************************************************************/
ascensionTracker::~ascensionTracker() {
	destroyMemory();
	numSensors = 0;
	trackerDrawingSize = 4.5;
	trailsDrawingSize = 4.5;
	alphaTracker = 1.0;
	alphaTrails = 1.0;
	visibleTracker = true;
	visibleTrails = true;
	numTrailPosesAdvance = 0;
	numTrailPosesRetract = 0;
	trailCapturingFlag = false;
	continuouslyLoggingFilename = NULL;
	sensorTrailsTriggerOnlyFlag = true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::destroyMemory()                                           *
*                                                                              *
*  This function clears the allocated memory that was created for this class.  *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::destroyMemory() {

	for (int i = 0; i < numTrailPosesAdvance; i++)
		if (Ts_trail_advance[i]) gsl_matrix_free(Ts_trail_advance[i]);
	for (int i = 0; i < numTrailPosesRetract; i++)
		if (Ts_trail_retract[i]) gsl_matrix_free(Ts_trail_retract[i]);
	if (Ts_trail_advance) free(Ts_trail_advance);
	Ts_trail_advance = NULL;
	if (Ts_trail_retract) free(Ts_trail_retract);
	Ts_trail_retract = NULL;
	if (sensorAttachedFlags) {
		for (int i = 0; i < numSensors; i++) {
			if (sensorAttachedFlags[i] && Ts[i]) 
				gsl_matrix_free(Ts[i]);
		}
		free(sensorAttachedFlags);
	}
	sensorAttachedFlags = NULL;
	if (Ts) free(Ts);
	Ts = NULL;
	trailCapturingFlag = false;
	if (continuouslyLoggingFilename) free(continuouslyLoggingFilename);
	continuouslyLoggingFilename = NULL;
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::init()                                                    *
*                                                                              *
*  This function initializes the communication with the ascension device.      *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::init() {

	int numTransmitters;
	SYSTEM_CONFIGURATION tracker;
	SENSOR_CONFIGURATION *sensorConfigs;
	TRANSMITTER_CONFIGURATION *transmitterConfigs;

	// initialize some variables
	numSensors = 0;
	trackerDrawingSize = 4.5;
	trailsDrawingSize = 4.5;
	visibleTracker = true;
	alphaTracker = 1.0;
	visibleTrails = true;
	alphaTrails = 1.0;
	numTrailPosesAdvance = 0;
	numTrailPosesRetract = 0;
	trailCapturingFlag = false;
	continuouslyLoggingFilename = NULL;
	sensorTrailsTriggerOnlyFlag = true;
	
	// initialize pointers to NULL
	Ts = NULL;
	sensorAttachedFlags = NULL;
	Ts_trail_advance = NULL;
	Ts_trail_retract = NULL;

	// initialize the ascension system
	if (InitializeBIRDSystem() != BIRD_ERROR_SUCCESS) {
		printf("\tError: ascensionTracker, did not find ascension tracker on USB.\n\n");
		return false;
	}

	// get the configuration of the system
	if (GetBIRDSystemConfiguration(&tracker) != BIRD_ERROR_SUCCESS) {
		printf("\tError: ascensionTracker, could not get configuration from tracker.\n\n");
		return false;
	}

	// read the number of transmitters and sensors
	numSensors = tracker.numberSensors;
	numTransmitters = tracker.numberTransmitters;

	// initialize the data values
	Ts = (gsl_matrix **)malloc(sizeof(gsl_matrix *)*numSensors);
	for (int i = 0; i < numSensors; i++)
		Ts[i] = NULL;

	// allocate memory for the sensorsAttachedFlags variable
	sensorAttachedFlags = (bool *)malloc(sizeof(bool)*numSensors);
	
	// get the configuration of the sensors
	sensorConfigs = (SENSOR_CONFIGURATION *)malloc(sizeof(SENSOR_CONFIGURATION)*numSensors);
	for (int i = 0; i < numSensors; i++) {
		if (GetSensorConfiguration(i, &sensorConfigs[i]) != BIRD_ERROR_SUCCESS) {
			printf("\tError: ascensionTracker, could not get sensor config for one of the tracker sensors.\n\n");
			if (sensorConfigs) free(sensorConfigs);
			destroyMemory(); numSensors = 0; return false;
		}
	}

	// store which sensors are attached
	for (int i = 0; i < numSensors; i++) {
		if (sensorConfigs[i].attached) {
			sensorAttachedFlags[i] = true;
			Ts[i] = gsl_matrix_calloc(4,4);
		} else {
			sensorAttachedFlags[i] = false;
		}
	}

	// get the configuration for the transmitters
	transmitterConfigs = (TRANSMITTER_CONFIGURATION *)malloc(sizeof(TRANSMITTER_CONFIGURATION)*numTransmitters);
	for (int i = 0; i < numTransmitters; i++) {
		if (GetTransmitterConfiguration(i, &transmitterConfigs[i]) != BIRD_ERROR_SUCCESS) {
			printf("\tError: ascensionTracker, could not get transmitter config for one of the tracker transmitters.\n\n");
			if (sensorConfigs) free(sensorConfigs);
			if (transmitterConfigs) free(transmitterConfigs);
			destroyMemory(); numSensors = 0; return false;			
		}
	}

	// set system parameters
	for (short i = 0; i < numTransmitters; i++) {
		if (transmitterConfigs[i].attached) {
			if (SetSystemParameter(SELECT_TRANSMITTER, &i, sizeof(i)) != BIRD_ERROR_SUCCESS) {
				printf("\tError: ascensionTracker, could not set transmitter parameters.\n\n");
				if (sensorConfigs) free(sensorConfigs);
				if (transmitterConfigs) free(transmitterConfigs);
				destroyMemory(); numSensors = 0; return false;						
			}
		}
	}

	// set the data format for the sensors
	DATA_FORMAT_TYPE mode = DOUBLE_POSITION_ANGLES;
	for (int i = 0; i < numSensors; i++) {	
		if (SetSensorParameter(i, DATA_FORMAT, &mode, sizeof(mode)) != BIRD_ERROR_SUCCESS) {
				printf("\tError: ascensionTracker, had trouble setting sensor data format.\n\n");
				if (sensorConfigs) free(sensorConfigs);
				if (transmitterConfigs) free(transmitterConfigs);
				destroyMemory(); numSensors = 0; return false;						
		}
	}

	if (sensorConfigs) free(sensorConfigs);
	if (transmitterConfigs) free(transmitterConfigs);
	loggingFlag = true;

	return true;

}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::isSensorAttached()                                        *
*                                                                              *
*  This function requests whether a sensor is attached at the given index.     *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::isSensorAttached(int sensorIndex) {
	if (!sensorAttachedFlags) {
		printf("\tError: ascensionTracker, tried to request sensor attached flag from array when not allocated.\n\n");
		return false;
	} else if (sensorIndex < 0 || sensorIndex >= numSensors) {
		printf("\tError: ascensionTracker, tried to request sensor attached flag with index out of range.\n\n");
		return false;
	}
	return sensorAttachedFlags[sensorIndex];
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::updatePoseMatrix()                                        *
*                                                                              *
*  This function updates the pose variables by reading from the device.        *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::updatePoseMatrix(double timeStamp) {

	DOUBLE_POSITION_ANGLES_RECORD record, *pRecord = &record;
	for (int i = 0; i < numSensors; i++) {
		if (sensorAttachedFlags[i]) {
			if (GetAsynchronousRecord(i, pRecord, sizeof(record)) != BIRD_ERROR_SUCCESS) {
				printf("\tError: ascensionTracker, could not access tracker pose from API.\n\n");
				return false;
			} else {
				if (!Ts[i] || Ts[i]->size1 != 4 || Ts[i]->size2 != 4) {
					printf("\tError: ascensionTracker, when updating pose matrix, Ts[i] was bad.\n\n");
					return false;
				}

				double rx = M_PI*record.r/180.0;
				double ry = -M_PI*record.e/180.0;
				double rz = -M_PI*record.a/180.0;

				gsl_matrix_set(Ts[i], 0, 0, cos(rz)*cos(ry)); gsl_matrix_set(Ts[i], 0, 1, -sin(rz)*cos(rx)+cos(rz)*sin(ry)*sin(rx));
				gsl_matrix_set(Ts[i], 0, 2, sin(rz)*sin(rx)+cos(rz)*sin(ry)*cos(rx)); gsl_matrix_set(Ts[i], 1, 0, sin(rz)*cos(ry));
				gsl_matrix_set(Ts[i], 1, 1, cos(rz)*cos(rx)+sin(rz)*sin(ry)*sin(rx)); gsl_matrix_set(Ts[i], 1, 2, -cos(rz)*sin(rx)+sin(rz)*sin(ry)*cos(rx));
				gsl_matrix_set(Ts[i], 2, 0, -sin(ry)); gsl_matrix_set(Ts[i], 2, 1, cos(ry)*sin(rx)); gsl_matrix_set(Ts[i], 2, 2, cos(ry)*cos(rx));
				gsl_matrix_set(Ts[i], 0, 3,  25.4*record.x); gsl_matrix_set(Ts[i], 1, 3, -25.4*record.y); gsl_matrix_set(Ts[i], 2, 3, -25.4*record.z);
				gsl_matrix_set(Ts[i], 3, 3, 1.0); gsl_matrix_set(Ts[i], 3, 0, 0.0); gsl_matrix_set(Ts[i], 3, 1, 0.0); gsl_matrix_set(Ts[i], 3, 2, 0.0);
			}
		}
	}

	// log if we're supposed to
	if (continuouslyLoggingFilename) {
		if (numSensors <= 0) return true;
		FILE *myFP = NULL; fopen_s(&myFP, continuouslyLoggingFilename, "a+");
		if (!myFP) return false;
		for (int i = 0; i < numSensors; i++) {
			if (sensorAttachedFlags[i]) {
				fprintf_s(myFP, "%d %f ", i, timeStamp);
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						fprintf_s(myFP, "%f ", ((float)gsl_matrix_get(Ts[i], j, k)));
					}
				}
				fprintf_s(myFP, "\n");
			}
		}
		fclose(myFP);
	}
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::getPoseMatrix()                                           *
*                                                                              *
*  Returns the transformation matrix for the tracker pose.                     *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::getPoseMatrix(gsl_matrix *Ttracker, int sensorIndex) {
	if (!Ttracker) {
		printf("\tError: ascensionTracker, return matrix not allocated.\n\n");
		return false;
	} else if (Ttracker->size1 != 4 || Ttracker->size2 != 4) {
		printf("\tError: ascensionTracker, return matrix not the right size.\n\n");
		return false;
	} else if (sensorIndex >= numSensors || sensorIndex < 0) {
		printf("\tError: ascensionTracker, requested pose matrix from sensor index out of range.\n\n");
		return false;
	} else if (!Ts[sensorIndex] || Ts[sensorIndex]->size1 != 4 || Ts[sensorIndex]->size2 != 4) {
		printf("\tError: ascensionTracker, requested pose matrix from sensor index but Ts[sensorIndex] is bad.\n\n");
		return false;
	}
	gsl_matrix_memcpy(Ttracker, Ts[sensorIndex]);
	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::drawTracker()                                             *
*                                                                              *
*  Draws the tracker pose in its own coordinate frame unless a transformation  *
*     matrix is passed in, so then the pose is transformed appropriately by T. *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::drawTracker(gsl_matrix *Ttransform, int sensorIndex) {

	if (!visibleTracker) return true;

	if (!Ts[sensorIndex] || Ts[sensorIndex]->size1 != 4 || Ts[sensorIndex]->size2 != 4) {
		printf("\tError: ascensionTracker, tried to draw tracker that was bad.\n\n");
		return false;
	}
	float color[3];
	if (sensorIndex == 0) {
		color[0] = 1.0f; color[1] = 1.0f; color[2] = 0.0f;
	} else {
		color[0] = 0.0f; color[1] = 1.0f; color[2] = 1.0f;
	}
	if (!drawPose(Ts[sensorIndex], Ttransform, color, alphaTracker, trackerDrawingSize))
		return false;

	return true;
}

/******************************************************************************\
*                                                                              *
*  ascensionTracker::drawPose()                                                *
*                                                                              *
*  Draws the pose given a transformation matrix.                               *
*                                                                              *
\******************************************************************************/
bool ascensionTracker::drawPose(gsl_matrix *Tpose, gsl_matrix *Ttransform, float *color3f, float alpha, float size) {

	GLfloat model_matrix[16];

	// allocate memory
	gsl_matrix *modelview = gsl_matrix_calloc(4,4);
	gsl_matrix *modelviewTransformed = gsl_matrix_calloc(4,4);
	gsl_matrix *poseMatrix = gsl_matrix_alloc(4,4);

	if (!Tpose) {
		printf("\tError: ascensionTracker, could not get tracker transformation matrix to draw pose.\n\n");
		gsl_matrix_free(modelview);
		gsl_matrix_free(modelviewTransformed);
		gsl_matrix_free(poseMatrix);
		return false;
	} else if (Ttransform) {
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Ttransform, Tpose, 0.0, poseMatrix);
	} else {
		gsl_matrix_memcpy(poseMatrix, Tpose);
	}

	// push the old matrix so we dont screw anything up
	glPushMatrix();

	// make a new GLfloat array to hold the modelview matrix for modification
	glGetFloatv(GL_MODELVIEW_MATRIX, model_matrix);

	// copy it to a gsl matrix
	int i = 0;
	for(int c = 0; c < 4; c++) {
		for(int r = 0; r < 4; r++) {
			gsl_matrix_set(modelview, r, c, model_matrix[i++]);
		}
	}

	// postmultiply the modelview matrix by the global link matrix
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, modelview, poseMatrix, 0.0, modelviewTransformed);

	// convert the new modelview matrix back into a GLfloat array
	i = 0;
	for(int c = 0; c < 4; c++) {
		for(int r = 0; r < 4; r++) {
			model_matrix[i++] = (GLfloat)gsl_matrix_get(modelviewTransformed, r, c);
		}
	}

	// and set it as the actual modelview matrix
	glLoadMatrixf(model_matrix);

	// free the memory we created
	gsl_matrix_free(modelview);
	gsl_matrix_free(modelviewTransformed);
	gsl_matrix_free(poseMatrix);

	// begin drawing
  	glBegin(GL_QUADS);
	glColor4f(color3f[0], color3f[1], color3f[2], alpha);

	// loop through in polar coords and draw all the polygons
	#define NUM_THETAS 36
	#define NUM_PHIS   18
	for (int i = 0; i < NUM_THETAS; i++) {
		float theta = (float)(2*M_PI*i/NUM_THETAS);
		
		// draw the side cylinder polygon
		glNormal3f(0.0, cos(theta), sin(theta));
		glVertex3d(0.0, size*cos(theta), size*sin(theta));
		glNormal3f(0.0, cos(theta+(float)(2*M_PI/NUM_THETAS)), sin(theta+(float)(2*M_PI/NUM_THETAS)));
		glVertex3d(0.0, size*cos(theta+(float)(2*M_PI/NUM_THETAS)), size*sin(theta+(float)(2*M_PI/NUM_THETAS)));
		glNormal3f(0.0, cos(theta+(float)(2*M_PI/NUM_THETAS)), sin(theta+(float)(2*M_PI/NUM_THETAS)));
		glVertex3d(-size, size*cos(theta+(float)(2*M_PI/NUM_THETAS)), size*sin(theta+(float)(2*M_PI/NUM_THETAS)));
		glNormal3f(0.0, cos(theta), sin(theta));
		glVertex3d(-size, size*cos(theta), size*sin(theta));

		// draw the half sphere polygons for the top and bottom of the link
		for (int j = 0; j < NUM_PHIS; j++) {
			float phi = (float)(((M_PI/2)*j)/NUM_PHIS);
			glNormal3f(sin(phi), cos(theta)*cos(phi), sin(theta)*cos(phi));
			glVertex3d(size*sin(phi), size*cos(theta)*cos(phi), size*sin(theta)*cos(phi));
			glNormal3f(sin(phi), cos(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi), sin(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi));
			glVertex3d(size*sin(phi), size*cos(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi), size*sin(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi));
			glNormal3f(sin(phi+(float)((M_PI/2)/NUM_PHIS)), cos(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi+(float)((M_PI/2)/NUM_PHIS)), sin(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi+(float)((M_PI/2)/NUM_PHIS)));
			glVertex3d(size*sin(phi+(float)((M_PI/2)/NUM_PHIS)),size*cos(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi+(float)((M_PI/2)/NUM_PHIS)), size*sin(theta+(float)(2*M_PI/NUM_THETAS))*cos(phi+(float)((M_PI/2)/NUM_PHIS)));
			glNormal3f(sin(phi+(float)((M_PI/2)/NUM_PHIS)), cos(theta)*cos(phi+(float)((M_PI/2)/NUM_PHIS)), sin(theta)*cos(phi+(float)((M_PI/2)/NUM_PHIS)));		
			glVertex3d(size*sin(phi+(float)((M_PI/2)/NUM_PHIS)), size*cos(theta)*cos(phi+(float)((M_PI/2)/NUM_PHIS)), size*sin(theta)*cos(phi+(float)((M_PI/2)/NUM_PHIS)));
		}

		// draw a cap on the bottom
		glNormal3f(-1.0, 0.0, 0.0);
		glVertex3d(-size, size*cos(theta), size*sin(theta));
		glNormal3f(-1.0, 0.0, 0.0);
		glVertex3d(-size, size*cos(theta+(float)(2*M_PI/NUM_THETAS)), size*sin(theta+(float)(2*M_PI/NUM_THETAS)));
		glNormal3f(-1.0, 0.0, 0.0);
		glVertex3d(-size, 0.0, 0.0);
		glNormal3f(-1.0, 0.0, 0.0);
		glVertex3d(-size, 0.0, 0.0);

	}

	// end drawing
	glEnd();

	glPopMatrix();
	return true;
}