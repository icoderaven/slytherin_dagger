/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  snakeEstimation.h                                                                    |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions dealing with snake shape estimation with a Kalman filtering.  |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#ifndef SNAKEESTIMATION_H
#define SNAKEESTIMATION_H

class snakeEstimation {
public:

	snakeEstimation();
	~snakeEstimation();

	bool setAlphaFullIterated(float val);
	bool setVisibleFullIterated(bool val);
	bool initializeFullIterated(gsl_matrix *trackerPose);
	bool advanceFullIterated(float phi, float theta);
	bool steerPredictFullIterated(float phi, float theta);
	bool steerCorrectFullIterated(gsl_matrix *trackerPose);
	bool drawFullIterated(gsl_matrix *Ttransform);
	bool destroyFullIterated(void);
	bool retractFullIterated(void);
	bool getFullIteratedEstimate(gsl_matrix *Xin, gsl_matrix *Pin);
	bool setUpdatingFullIterated(bool val);
	bool setSimplifiedUpdateFullIterated(bool val);

	bool setAlphaFullKalm(float val);
	bool setVisibleFullKalm(bool val);
	bool initializeFullKalm(gsl_matrix *trackerPose);
	bool advanceFullKalm(float phi, float theta);
	bool steerPredictFullKalm(float phi, float theta);
	bool steerCorrectFullKalm(gsl_matrix *trackerPose);
	bool drawFullKalm(gsl_matrix *Ttransform);
	bool destroyFullKalm(void);
	bool retractFullKalm(void);
	bool setUpdatingFullKalm(bool val);

	bool setAlphaFullDead(float val);
	bool setVisibleFullDead(bool val);
	bool initializeFullDead(gsl_matrix *trackerPose);
	bool advanceFullDead(float phi, float theta);
	bool steerPredictFullDead(float phi, float theta);
	bool drawFullDead(gsl_matrix *Ttransform);
	bool destroyFullDead(void);
	bool retractFullDead(void);

	bool setAlphaDistalKalm(float val);
	bool setVisibleDistalKalm(bool val);
	bool initializeDistalKalm(gsl_matrix *trackerPose);
	bool advanceDistalKalm(float phi, float theta);
	bool steerPredictDistalKalm(float phi, float theta);
	bool steerCorrectDistalKalm(gsl_matrix *trackerPose);
	bool drawDistalKalm(gsl_matrix *Ttransform);
	bool destroyDistalKalm(void);
	bool retractDistalKalm(void);
	bool getDistalTransformationMatrix(gsl_matrix *Tin);
	bool setUpdatingDistalKalm(bool val);
	
	bool isInitializedSnakeEstimation(void);
	bool drawLink(gsl_matrix *T, float *colorTop3f, float *colorBottom3f, float alpha, float radius, float linkLength);
	void setLoggingFlag(bool val);
	bool logData(FILE *fptr, double timeStamp, int dataLabel);

private:

	float alphaDistalKalm;			// denotes the transparency level for the distal kalm estimate
	bool visibleDistalKalm;			// determines if the distal kalm estimate is drawn on the screen
	gsl_matrix *X_distal_kalm;		// the state is [x, y, z, r_x, r_y, r_z] where the rotations are applied
	gsl_matrix *P_distal_kalm;		// the covariance matrix for the X_distal_kalm state vector
	float thetaDistalKalm;			// the current theta turning value stored for the distal kalm estimate
	float phiDistalKalm;			// the current phi turning value stored for the distal kalm estimate
	bool distalKalmUpdating;
	float alphaFullIterated;        // denotes the transparency level for the full kalm estimate
	bool visibleFullIterated;       // determines if the full kalm estimate is drawn on the screen
	gsl_matrix *X_full_iterated;	// the state for the full kalm estimate
	gsl_matrix *P_full_iterated;	// the covariance matrix for the X_full_kalm state vector
	bool fullIteratedUpdating;
	bool simplifiedUpdateFullIterated;
	float alphaFullKalm;			// denotes the transparency level for the full kalm estimate
	bool visibleFullKalm;			// determines if the full kalm estimate is drawn on the screen
	gsl_matrix *X_full_kalm;		// the state for the full kalm estimate
	gsl_matrix *P_full_kalm;		// the covariance matrix for the X_full_kalm state vector
	bool fullKalmUpdating;
	float alphaFullDead;			// denotes the transparency level for the full dead estimate
	bool visibleFullDead;			// determines if the full dead estimate is drawn on the screen
	gsl_matrix *X_full_dead;		// the state for the full dead estimate
	bool loggingFlag;				// whether we're logging state estimation stuff or not

};

#endif SNAKEESTIMATION_H
