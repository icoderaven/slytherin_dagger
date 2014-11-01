/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  ascension.h                                                                          |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Custom functions to interface with the Ascension trakSTAR tracker.      |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#ifndef ASCENSION_H
#define ASCENSION_H

#ifndef M_PI
#define M_PI 3.14159265
#endif

class ascensionTracker {
public:

	ascensionTracker();
	~ascensionTracker();

	void setVisibleTracker(bool val);
	void setVisibleTrails(bool val);
	bool getTrailCapturingFlag(void);
	void setTrailCapturingFlag(bool val);
	bool addToTrail(bool advanceFlag);
	bool isSensorAttached(int sensorIndex);
	bool updatePoseMatrix(double timeStamp);
	bool drawTracker(gsl_matrix *Ttransform, int sensorIndex);
	bool logAppendTrackerPose(char *filename, double timeStamp);
	int getNumSensors(void);
	bool setTrackerDrawingSize(float size);
	bool setAlphaTracker(float val);
	bool setTrailsDrawingSize(float size);
	bool setAlphaTrails(float val);
	bool clearTrails(void);
	bool drawTrails(gsl_matrix *Ttransform);
	bool saveTrailsData(char *filename);
	bool getPoseMatrix(gsl_matrix *Ttracker, int sensorIndex);
	bool isContinuouslyLogging(void);
	void setContinuouslyLogging(char *filename);
	void setLoggingFlag(bool val);
	void setSensorTrailsTriggerOnlyFlag(bool val);
	bool getSensorTrailsTriggerOnlyFlag(void);
	bool logData(FILE *fptr, double timeStamp, int dataLabel);

private:

	bool init();
	bool destroyMemory();
	bool drawPose(gsl_matrix *Tpose, gsl_matrix *Ttransform, float *color3f, float alpha, float size);
	bool *sensorAttachedFlags;
	gsl_matrix **Ts;
	gsl_matrix **Ts_trail_advance;
	gsl_matrix **Ts_trail_retract;
	int numTrailPosesAdvance;
	int numTrailPosesRetract;
	float trackerDrawingSize;
	float trailsDrawingSize;
	int numSensors;
	float alphaTracker;
	float alphaTrails;
	bool trailCapturingFlag;
	bool visibleTracker;
	bool visibleTrails;
	char *continuouslyLoggingFilename;
	bool loggingFlag;
	bool sensorTrailsTriggerOnlyFlag;

};

#endif ASCENSION_H
