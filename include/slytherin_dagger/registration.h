/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  registration.h                                                                       |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions dealing with registration between 3D models and the sensor.   |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#ifndef REGISTRATION_H
#define REGISTRATION_H

class registration {
public:

	registration();
	~registration();

	bool logTransformationMatrix();
	bool clearModelPoints();
	bool clearTrackerPoints();
	bool addPointToTrackerPoints(double *point);
	bool loadModelPointsFromFile(char *filename);
	bool loadTrackerPointsFromFile(char *filename);
	bool saveTrackerPointsToFile(char *filename);
	bool registerPoints();
	int get_numModelPoints();
	int get_numTrackerPoints();
	float get_avgRegistrationError();
	bool get_T_tracker2model(gsl_matrix *T_return);
	void setLoggingFlag(bool val);
	bool logData(FILE *fptr, double timeStamp, int dataLabel);

private:

	int numModelPoints;
	int numTrackerPoints;
	float avgRegistrationError;
	gsl_matrix *modelPoints;
	gsl_matrix *trackerPoints;
	gsl_matrix *T_tracker2model;
	bool loggingFlag;

};

#endif REGISTRATION_H
