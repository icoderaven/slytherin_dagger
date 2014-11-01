/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  autonomy.h                                                                           |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions dealing with autonomous driving of the snake robot.           |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#ifndef AUTONOMY_H
#define AUTONOMY_H

#define SPLINE_DIMENSION 5		// e.g. 3 would be cubic spline for z and y as a function of x
								// note: 3 (cubic) would have 4 parameters for each y and z spline curve
								// but we only need to solve for 2 parameters (each, for y and z)
								// this is due to constraint forced for start and end points of spline

#define NUM_SPLINE_PARAMETERS (SPLINE_DIMENSION+1)

class autonomy {
public:

	autonomy();
	~autonomy();

	float getAutonomousJoystickX(void);
	float getAutonomousJoystickY(void);
	void setAutonomousDrivingFlag(bool val);
	bool getAutonomousDrivingFlag(void);
	void setVisiblePreCurvePathPoints(bool val);
	bool getVisiblePreCurvePathPoints(void);
	void setVisibleSplineCurve(bool val);
	bool getVisibleSplineCurve(void);
	void setPickingPointsFlag(bool val);
	bool getPickingPointsFlag(void);
	bool getUpdatePreCurveDrawFlag(void);
	bool getUpdateCurveDrawFlag(void);
	bool clearPath(void);
	bool drawPreCurvePathPoints(int glListIndex);
	bool drawCurvePathPoints(int glListIndex);
	bool addControlRay(double nearx, double neary, double nearz, double farx, double fary, double farz, int numPolygonMeshes, polygonMesh **polygonMeshes);
	bool computePath(void);
	bool updatePreCurvePathPoints(int numPolygonMeshes, polygonMesh **polygonMeshes);
	bool computeControlLookAheadPoint(void);
	bool computeAutonomousJoystickValues(void);
	bool drawControlLookAheadPoint(void);
	void setLoggingFlag(bool val);
	bool logData(FILE *fptr, double timeStamp, int dataLabel);

private:

	bool checkSameClockDir(float *pt1, float *pt2, float *pt3, float *norm);
	bool computeLineTriangleIntersection(float *tri_pt1, float *tri_pt2, float *tri_pt3, float *line_pt1, float *line_pt2, float *intersection_pt);

	bool visiblePreCurvePathPoints;
	bool visibleSplineCurve;
	bool pickingPointsFlag;
	bool updateCurveDrawFlag;
	bool updatePreCurveDrawFlag;
	int numControlPoints;
	double **controlRayNearPoints;
	double **controlRayFarPoints;
	double **controlSurfacePoints;
	double **controlSurfacePointsPushedOut;
	int numSegmentsAddedToPreCurvePathPoints;
	int *numPreCurvePathPointsInEachSegment;
	double ***preCurvePathPoints;
	double ***preCurvePathPointsPushedOut;
	int numSplineSegments;
	double **splineParametersForEachSegment;
	int *numCurvePathPointsInEachSegment;
	double ***curvePathPoints;
	bool autonomousDrivingFlag;
	float autonomousJoystickX;
	float autonomousJoystickY;
	double *controlLookAheadPoint;
	bool loggingFlag;
	
};

#endif AUTONOMY_H
