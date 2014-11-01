/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  snakelibgl_gui.cpp                                                                   |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions to handle the GUI for the snakelibgl files.                   |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#include <windows.h>
#include <gtk/gtk.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include "../probe_control/gui_support.h"
#include "../probe_control/gui_interface.h"
#include "../probe_control/gui_callbacks.h"
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include "snakelibgl.h"
#include "snakeEstimation.h"
#include "registration.h"
#include "ascension.h"
#include "polygonMesh.h"
#include "autonomy.h"
#include "snakelibgl_gui.h"

// extern variables
extern GtkWidget *snakelibgl_window;
extern registration *myRegistration;
extern ascensionTracker *myAscensionTracker;
extern polygonMesh **myPolygonMeshes;
extern snakeEstimation *mySnakeEstimation;
extern autonomy *myAutonomy;
extern int numPolygonMeshes;

GtkWidget *snakelibgl_autonomyStartStopButton;
GtkWidget *snakelibgl_visualizationTable;
GtkWidget *snakelibgl_visualizationPolygonMeshFilenameTextBox;
GtkWidget *snakelibgl_visualizationScreenshotFilenameTextBox;
GtkWidget *snakelibgl_ascensionNumTrackerPtsLabel; 
GtkWidget *snakelibgl_ascensionNumModelPtsLabel;
GtkWidget *snakelibgl_ascensionModelPointsFilenameTextBox;  
GtkWidget *snakelibgl_ascensionTrackerPointsFilenameTextBox;
GtkWidget *snakelibgl_ascensionLogTrackerPointTextBox;
GtkWidget *snakelibgl_ascensionContinuousLogTrackerTextBox;
GtkWidget *snakelibgl_ascensionContinuousLogTrackerButton;
GtkWidget *snakelibgl_ascensionRegistrationErrorLabel;
GtkWidget *snakelibgl_ascensionSensorVisualsSizeSlider;
GtkWidget *snakelibgl_ascensionSensorVisualsCheckBox;
GtkWidget *snakelibgl_ascensionSensorVisualsAlphaSlider;
GtkWidget *snakelibgl_ascensionSensorTrailsStartStopButton;
GtkWidget *snakelibgl_ascensionSensorTrailsCheckBox;  
GtkWidget *snakelibgl_ascensionSensorTrailsAlphaSlider;
GtkWidget *snakelibgl_ascensionSensorTrailsSizeSlider;
GtkWidget *snakelibgl_ascensionSaveTrailDataTextBox;
GtkWidget *snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox;
GtkWidget *snakelibgl_snakeEstimationFullDeadCheckBox;
GtkWidget *snakelibgl_snakeEstimationFullKalmCheckBox;
GtkWidget *snakelibgl_snakeEstimationFullIteratedCheckBox;
GtkWidget *snakelibgl_snakeEstimationDistalKalmCheckBox;
GtkWidget *snakelibgl_snakeEstimationAlphaFullDeadSlider;
GtkWidget *snakelibgl_snakeEstimationAlphaFullKalmSlider;
GtkWidget *snakelibgl_snakeEstimationAlphaFullIteratedSlider;
GtkWidget *snakelibgl_snakeEstimationAlphaDistalKalmSlider;
GtkWidget *snakelibgl_snakeEstimationFullKalmUpdateCheckBox;
GtkWidget *snakelibgl_snakeEstimationFullIteratedUpdateCheckBox;
GtkWidget *snakelibgl_snakeEstimationDistalKalmUpdateCheckBox;
GtkWidget *snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox;
GtkWidget *snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox;
GtkWidget *snakelibgl_autonomyVisibleSplineCurveCheckBox;
GtkWidget *snakelibgl_autonomyEnableDisableButton;
GtkWidget *snakelibgl_loggingAllOnButton;
GtkWidget *snakelibgl_loggingAllOffButton;
GtkWidget *snakelibgl_loggingAscensionTrackerCheckBox;
GtkWidget *snakelibgl_loggingRegistrationCheckBox;
GtkWidget *snakelibgl_loggingSnakeEstimationCheckBox;
GtkWidget *snakelibgl_loggingAutonomyCheckBox;
GtkWidget *snakelibgl_loggingStartNewLogFileTextBox;

void snakelibgl_loggingStartNewLogFileButton_clicked(void)
{
	SnakeLibGL::startNewLogFile((char *)gtk_entry_get_text((GtkEntry *)snakelibgl_loggingStartNewLogFileTextBox));
}

void snakelibgl_loggingAscensionTrackerCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_loggingAscensionTrackerCheckBox)) {
		myAscensionTracker->setLoggingFlag(true);
	} else {
		myAscensionTracker->setLoggingFlag(false);
	}
}

void snakelibgl_loggingRegistrationCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_loggingRegistrationCheckBox)) {
		myRegistration->setLoggingFlag(true);
	} else {
		myRegistration->setLoggingFlag(false);
	}
}

void snakelibgl_loggingSnakeEstimationCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_loggingSnakeEstimationCheckBox)) {
		mySnakeEstimation->setLoggingFlag(true);
	} else {
		mySnakeEstimation->setLoggingFlag(false);
	}
}

void snakelibgl_loggingAutonomyCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_loggingAutonomyCheckBox)) {
		myAutonomy->setLoggingFlag(true);
	} else {
		myAutonomy->setLoggingFlag(false);
	}
}

void snakelibgl_loggingAllOnButton_clicked(void)
{
	gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingAscensionTrackerCheckBox, (gboolean)1);
	gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingRegistrationCheckBox, (gboolean)1);
	gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingAutonomyCheckBox, (gboolean)1);
	gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingSnakeEstimationCheckBox, (gboolean)1);
}

void snakelibgl_loggingAllOffButton_clicked(void)
{
	gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingAscensionTrackerCheckBox, (gboolean)0);
	gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingRegistrationCheckBox, (gboolean)0);
	gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingAutonomyCheckBox, (gboolean)0);
	gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingSnakeEstimationCheckBox, (gboolean)0);
}

void snakelibgl_autonomyClearPathButton_clicked(void)
{
	myAutonomy->clearPath();
	gtk_object_set(GTK_OBJECT(snakelibgl_autonomyStartStopButton), "label", "Start", NULL);
	myAutonomy->setPickingPointsFlag(false);
	gtk_object_set(GTK_OBJECT(snakelibgl_autonomyEnableDisableButton), "label", "Enable Autonomy", NULL);
	myAutonomy->setAutonomousDrivingFlag(false);
}

void snakelibgl_autonomyVisibleSplineCurveCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_autonomyVisibleSplineCurveCheckBox)) {
		myAutonomy->setVisibleSplineCurve(true);
	} else {
		myAutonomy->setVisibleSplineCurve(false);
	}
}

void snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox)) {
		myAutonomy->setVisiblePreCurvePathPoints(true);
	} else {
		myAutonomy->setVisiblePreCurvePathPoints(false);
	}
}

void snakelibgl_autonomyEnableDisableButton_clicked(void)
{
	if (myAutonomy->getAutonomousDrivingFlag()) {
		gtk_object_set(GTK_OBJECT(snakelibgl_autonomyEnableDisableButton), "label", "Enable Autonomy", NULL);
		myAutonomy->setAutonomousDrivingFlag(false);
	} else {
		myAutonomy->setAutonomousDrivingFlag(true);
		if (myAutonomy->getAutonomousDrivingFlag())
			gtk_object_set(GTK_OBJECT(snakelibgl_autonomyEnableDisableButton), "label", "Disable Autonomy", NULL);
	}
}

void snakelibgl_autonomyStartStopButton_clicked(void)
{
	if (myAutonomy->getPickingPointsFlag()) {
		gtk_object_set(GTK_OBJECT(snakelibgl_autonomyStartStopButton), "label", "Start", NULL);
		myAutonomy->setPickingPointsFlag(false);
	} else {
		myAutonomy->setPickingPointsFlag(true);
		if (myAutonomy->getPickingPointsFlag())
			gtk_object_set(GTK_OBJECT(snakelibgl_autonomyStartStopButton), "label", "Stop", NULL);
	}
}

void snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox)) {
		mySnakeEstimation->setSimplifiedUpdateFullIterated(true);
	} else {
		mySnakeEstimation->setSimplifiedUpdateFullIterated(false);
	}
}

void snakelibgl_snakeEstimationFullIteratedUpdateCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_snakeEstimationFullIteratedUpdateCheckBox)) {
		mySnakeEstimation->setUpdatingFullIterated(true);
	} else {
		mySnakeEstimation->setUpdatingFullIterated(false);
	}
}

void snakelibgl_snakeEstimationFullKalmUpdateCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_snakeEstimationFullKalmUpdateCheckBox)) {
		mySnakeEstimation->setUpdatingFullKalm(true);
	} else {
		mySnakeEstimation->setUpdatingFullKalm(false);
	}
}

void snakelibgl_snakeEstimationDistalKalmUpdateCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_snakeEstimationDistalKalmUpdateCheckBox)) {
		mySnakeEstimation->setUpdatingDistalKalm(true);
	} else {
		mySnakeEstimation->setUpdatingDistalKalm(false);
	}
}

void snakelibgl_snakeEstimationAlphaFullDeadSlider_value_changed(void)
{
	GtkAdjustment *adj = gtk_range_get_adjustment((GtkRange *)snakelibgl_snakeEstimationAlphaFullDeadSlider);
	mySnakeEstimation->setAlphaFullDead((float)gtk_adjustment_get_value(adj));
}

void snakelibgl_snakeEstimationAlphaFullKalmSlider_value_changed(void)
{
	GtkAdjustment *adj = gtk_range_get_adjustment((GtkRange *)snakelibgl_snakeEstimationAlphaFullKalmSlider);
	mySnakeEstimation->setAlphaFullKalm((float)gtk_adjustment_get_value(adj));
}

void snakelibgl_snakeEstimationAlphaFullIteratedSlider_value_changed(void)
{
	GtkAdjustment *adj = gtk_range_get_adjustment((GtkRange *)snakelibgl_snakeEstimationAlphaFullIteratedSlider);
	mySnakeEstimation->setAlphaFullIterated((float)gtk_adjustment_get_value(adj));
}

void snakelibgl_snakeEstimationAlphaDistalKalmSlider_value_changed(void)
{
	GtkAdjustment *adj = gtk_range_get_adjustment((GtkRange *)snakelibgl_snakeEstimationAlphaDistalKalmSlider);
	mySnakeEstimation->setAlphaDistalKalm((float)gtk_adjustment_get_value(adj));
}

void snakelibgl_snakeEstimationFullDeadCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_snakeEstimationFullDeadCheckBox)) {
		mySnakeEstimation->setVisibleFullDead(true);
	} else {
		mySnakeEstimation->setVisibleFullDead(false);
	}
}

void snakelibgl_snakeEstimationFullKalmCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_snakeEstimationFullKalmCheckBox)) {
		mySnakeEstimation->setVisibleFullKalm(true);
	} else {
		mySnakeEstimation->setVisibleFullKalm(false);
	}
}

void snakelibgl_snakeEstimationFullIteratedCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_snakeEstimationFullIteratedCheckBox)) {
		mySnakeEstimation->setVisibleFullIterated(true);
	} else {
		mySnakeEstimation->setVisibleFullIterated(false);
	}
}

void snakelibgl_snakeEstimationDistalKalmCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_snakeEstimationDistalKalmCheckBox)) {
		mySnakeEstimation->setVisibleDistalKalm(true);
	} else {
		mySnakeEstimation->setVisibleDistalKalm(false);
	}
}

void snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox)) {
		myAscensionTracker->setSensorTrailsTriggerOnlyFlag(true);
	} else {
		myAscensionTracker->setSensorTrailsTriggerOnlyFlag(false);
	}	
}

void snakelibgl_ascensionSensorTrailsAddSingleButton_clicked(void)
{
	myAscensionTracker->addToTrail(true);
	return;
}

void snakelibgl_ascensionSensorTrailsSaveButton_clicked(void)
{
	char *filename = (char *)gtk_entry_get_text((GtkEntry *)snakelibgl_ascensionSaveTrailDataTextBox);
	myAscensionTracker->saveTrailsData(filename);	
}

void snakelibgl_ascensionSensorTrailsSizeSlider_value_changed(void)
{
	GtkAdjustment *adj = gtk_range_get_adjustment((GtkRange *)snakelibgl_ascensionSensorTrailsSizeSlider);
	myAscensionTracker->setTrailsDrawingSize((float)gtk_adjustment_get_value(adj));
}

void snakelibgl_ascensionSensorTrailsAlphaSlider_value_changed(void)
{
	GtkAdjustment *adj = gtk_range_get_adjustment((GtkRange *)snakelibgl_ascensionSensorTrailsAlphaSlider);
	myAscensionTracker->setAlphaTrails((float)gtk_adjustment_get_value(adj));
}

void snakelibgl_ascensionSensorTrailsCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_ascensionSensorTrailsCheckBox)) {
		myAscensionTracker->setVisibleTrails(true);
	} else {
		myAscensionTracker->setVisibleTrails(false);
	}
}

void snakelibgl_ascensionSensorTrailsClearButton_clicked(void)
{
	myAscensionTracker->clearTrails();
}

void snakelibgl_ascensionSensorTrailsStartStopButton_clicked(void)
{
	if (myAscensionTracker->getTrailCapturingFlag()) {
		gtk_object_set(GTK_OBJECT(snakelibgl_ascensionSensorTrailsStartStopButton), "label", "Start", NULL);
		myAscensionTracker->setTrailCapturingFlag(false);
	} else {
		gtk_object_set(GTK_OBJECT(snakelibgl_ascensionSensorTrailsStartStopButton), "label", "Stop", NULL);
		myAscensionTracker->setTrailCapturingFlag(true);
	}
}

void snakelibgl_ascensionSensorVisualsCheckBox_toggled(void)
{
	if (gtk_toggle_button_get_active((GtkToggleButton *)snakelibgl_ascensionSensorVisualsCheckBox)) {
		myAscensionTracker->setVisibleTracker(true);
	} else {
		myAscensionTracker->setVisibleTracker(false);
	}
}

void snakelibgl_ascensionSensorVisualsSizeSlider_value_changed(void)
{
	GtkAdjustment *adj = gtk_range_get_adjustment((GtkRange *)snakelibgl_ascensionSensorVisualsSizeSlider);
	myAscensionTracker->setTrackerDrawingSize((float)gtk_adjustment_get_value(adj));
}

void snakelibgl_ascensionSensorVisualsAlphaSlider_value_changed(void)
{
	GtkAdjustment *adj = gtk_range_get_adjustment((GtkRange *)snakelibgl_ascensionSensorVisualsAlphaSlider);
	myAscensionTracker->setAlphaTracker((float)gtk_adjustment_get_value(adj));
}

void snakelibgl_ascensionContinuousLogTrackerButton_clicked(void)
{
	if (myAscensionTracker->isContinuouslyLogging()) {
		gtk_widget_set_sensitive(snakelibgl_ascensionContinuousLogTrackerTextBox, true);
		gtk_object_set(GTK_OBJECT(snakelibgl_ascensionContinuousLogTrackerButton), "label", "Continuous Start", NULL);
		myAscensionTracker->setContinuouslyLogging(NULL);
	} else {
		myAscensionTracker->setContinuouslyLogging((char *)gtk_entry_get_text((GtkEntry *)snakelibgl_ascensionContinuousLogTrackerTextBox));
		if (myAscensionTracker->isContinuouslyLogging()) {
			gtk_widget_set_sensitive(snakelibgl_ascensionContinuousLogTrackerTextBox, false);
			gtk_object_set(GTK_OBJECT(snakelibgl_ascensionContinuousLogTrackerButton), "label", "Continuous Stop", NULL);
		}
	}
}

void snakelibgl_ascensionLogTrackerPointButton_clicked(void)
{
	long long currentTime;
	long long frequency;
	QueryPerformanceCounter((_LARGE_INTEGER*)&currentTime);
	QueryPerformanceFrequency((_LARGE_INTEGER*)&frequency);
	double timeStamp = (double)currentTime/(double)frequency;
	char *filename = (char *)gtk_entry_get_text((GtkEntry *)snakelibgl_ascensionLogTrackerPointTextBox);
	myAscensionTracker->logAppendTrackerPose(filename, timeStamp);
}

void snakelibgl_ascensionRegisterButton_clicked(void)
{
	if (myRegistration->registerPoints())
		myRegistration->logTransformationMatrix();
	for (int i = 0; i < numPolygonMeshes; i++)
		myPolygonMeshes[i]->setUpdateDrawFlag(true);
	char label[MAX_TEXT_LENGTH];
	sprintf_s(label, MAX_TEXT_LENGTH, "Avg. Registration Error: %.2f mm", myRegistration->get_avgRegistrationError());
	gtk_label_set_text((GtkLabel *)snakelibgl_ascensionRegistrationErrorLabel, label);
}

void snakelibgl_ascensionClearModelPtsButton_clicked (void)
{
	myRegistration->clearModelPoints();	
	char label[MAX_TEXT_LENGTH];
	sprintf_s(label, MAX_TEXT_LENGTH, "Num Model Pts: %d", myRegistration->get_numModelPoints());
	gtk_label_set_text((GtkLabel *)snakelibgl_ascensionNumModelPtsLabel, label);
	sprintf_s(label, MAX_TEXT_LENGTH, "Avg. Registration Error: %.2f mm", myRegistration->get_avgRegistrationError());
	gtk_label_set_text((GtkLabel *)snakelibgl_ascensionRegistrationErrorLabel, label);
}

void snakelibgl_ascensionSaveTrackerPointsButton_clicked (void)
{
	char *filename = (char *)gtk_entry_get_text((GtkEntry *)snakelibgl_ascensionTrackerPointsFilenameTextBox);
	if (myRegistration->get_numTrackerPoints() > 0) {
		myRegistration->saveTrackerPointsToFile(filename);
	}
}

void snakelibgl_ascensionLoadTrackerPointsButton_clicked (void)
{
	char *filename = (char *)gtk_entry_get_text((GtkEntry *)snakelibgl_ascensionTrackerPointsFilenameTextBox);
	myRegistration->loadTrackerPointsFromFile(filename);
	char label[MAX_TEXT_LENGTH];
	sprintf_s(label, MAX_TEXT_LENGTH, "Num Tracker Pts: %d", myRegistration->get_numTrackerPoints());
	gtk_label_set_text((GtkLabel *)snakelibgl_ascensionNumTrackerPtsLabel, label);
}

void snakelibgl_ascensionLoadModelPointsButton_clicked (void)
{
	char *filename = (char *)gtk_entry_get_text((GtkEntry *)snakelibgl_ascensionModelPointsFilenameTextBox);
	myRegistration->loadModelPointsFromFile(filename);
	char label[MAX_TEXT_LENGTH];
	sprintf_s(label, MAX_TEXT_LENGTH, "Num Model Pts: %d", myRegistration->get_numModelPoints());
	gtk_label_set_text((GtkLabel *)snakelibgl_ascensionNumModelPtsLabel, label);
}

void snakelibgl_ascensionClearTrackerPtsButton_clicked (void)
{
	myRegistration->clearTrackerPoints();	
	char label[MAX_TEXT_LENGTH];
	sprintf_s(label, MAX_TEXT_LENGTH, "Num Tracker Pts: %d", myRegistration->get_numTrackerPoints());
	gtk_label_set_text((GtkLabel *)snakelibgl_ascensionNumTrackerPtsLabel, label);
	sprintf_s(label, MAX_TEXT_LENGTH, "Avg. Registration Error: %.2f mm", myRegistration->get_avgRegistrationError());
	gtk_label_set_text((GtkLabel *)snakelibgl_ascensionRegistrationErrorLabel, label);
}

void snakelibgl_ascensionCaptureTrackerPointButton_clicked (void)
{
	// for registration, we acquire a point from the last sensor index that is attached
	double point[3];
	bool foundPoint = false;
	for (int i = 0; i < myAscensionTracker->getNumSensors(); i++) {
		if (myAscensionTracker->isSensorAttached(i)) {
			gsl_matrix *T = gsl_matrix_alloc(4,4);
			myAscensionTracker->getPoseMatrix(T,i);
			point[0] = gsl_matrix_get(T, 0, 3);
			point[1] = gsl_matrix_get(T, 1, 3);
			point[2] = gsl_matrix_get(T, 2, 3);
			if (T) gsl_matrix_free(T);
			foundPoint = true;
		}
	}
	if (foundPoint) {
		myRegistration->addPointToTrackerPoints(point);
	}
	char label[MAX_TEXT_LENGTH];
	sprintf_s(label, MAX_TEXT_LENGTH, "Num Tracker Pts: %d", myRegistration->get_numTrackerPoints());
	gtk_label_set_text((GtkLabel *)snakelibgl_ascensionNumTrackerPtsLabel, label);
}

void snakelibgl_visualizationPolygonMeshFileLoadButton_clicked (void)
{
	GtkWidget *newHbox;
	GtkWidget *newCheckButton;
	GtkWidget *newAlphaSlider;

	char *filename;
	filename = (char *)gtk_entry_get_text((GtkEntry *)snakelibgl_visualizationPolygonMeshFilenameTextBox);

	polygonMesh *newPolygonMesh = new polygonMesh;
	if (!(newPolygonMesh->loadPolygonMeshFile(filename))) {
		delete newPolygonMesh;
	} else {
		numPolygonMeshes++;
		polygonMesh **newPolygonMeshes = (polygonMesh **)malloc(sizeof(polygonMesh *)*numPolygonMeshes);
		if (numPolygonMeshes > 0) memcpy_s(newPolygonMeshes, sizeof(polygonMesh *)*(numPolygonMeshes-1), myPolygonMeshes, sizeof(polygonMesh *)*(numPolygonMeshes-1));
		if (myPolygonMeshes) free(myPolygonMeshes);
		myPolygonMeshes = newPolygonMeshes;
		newPolygonMeshes[numPolygonMeshes-1] = newPolygonMesh;	

		myPolygonMeshes[numPolygonMeshes-1]->drawPolygonMesh(numPolygonMeshes);
	
		newHbox = gtk_hbox_new(FALSE, 0);
		gtk_container_set_border_width (GTK_CONTAINER (newHbox), 0);
		gtk_box_set_spacing((GtkBox *)newHbox, 10);
		gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), newHbox, 0, 1, 10+numPolygonMeshes, 11+numPolygonMeshes, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
		gtk_widget_show (newHbox);

		newCheckButton = gtk_check_button_new_with_label((gchar *)filename);
		gtk_box_pack_start (GTK_BOX (newHbox), newCheckButton, TRUE, TRUE, 0);
		gtk_widget_show (newCheckButton);
		gtk_toggle_button_set_active((GtkToggleButton *)newCheckButton, (gint)TRUE);
		g_signal_connect ((gpointer) newCheckButton, "toggled", G_CALLBACK (snakelibgl_visualizationPolygonMeshFileCheckButton_toggled), newCheckButton);
		newCheckButton->name = (gchar*)malloc(sizeof(gchar)*100);
		sprintf_s(newCheckButton->name, MAX_TEXT_LENGTH, "%d", numPolygonMeshes-1);   

		newAlphaSlider = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new(1.0, 0.0, 1.0, 0.05, 1, 0)));
		gtk_box_pack_start (GTK_BOX (newHbox), newAlphaSlider, TRUE, TRUE, 0);
		gtk_widget_show (newAlphaSlider);
		gtk_widget_set_size_request (newAlphaSlider, 150, 35);
		gtk_scale_set_digits (GTK_SCALE (newAlphaSlider), 2);
		g_signal_connect ((gpointer) newAlphaSlider, "value_changed", G_CALLBACK (snakelibgl_visualizationPolygonMeshFileAlphaSlider_value_changed), newAlphaSlider);	
		newAlphaSlider->name = (gchar*)malloc(sizeof(gchar)*100);
		sprintf_s(newAlphaSlider->name, MAX_TEXT_LENGTH, "%d", numPolygonMeshes-1);   
	}
}

void snakelibgl_visualizationPolygonMeshFileCheckButton_toggled(GtkWidget *checkButton)
{
	int polygonMeshFileIndex = atoi(checkButton->name);
	if (gtk_toggle_button_get_active ((GtkToggleButton *)checkButton)) {
		myPolygonMeshes[polygonMeshFileIndex]->setVisibleFlag(true);
	} else {
		myPolygonMeshes[polygonMeshFileIndex]->setVisibleFlag(false);
	}
}

void snakelibgl_visualizationPolygonMeshFileAlphaSlider_value_changed(GtkWidget *alphaSlider)
{
	int polygonMeshFileIndex = atoi(alphaSlider->name);
	GtkRange *range = (GtkRange *)alphaSlider;
	GtkAdjustment *adj = gtk_range_get_adjustment(range);
	myPolygonMeshes[polygonMeshFileIndex]->setAlpha((float)gtk_adjustment_get_value(adj));
	myPolygonMeshes[polygonMeshFileIndex]->setUpdateDrawFlag(true);
}

void snakelibgl_visualizationTakeScreenshotButton_clicked(void)
{
	SnakeLibGL::captureScreenshot((char *)gtk_entry_get_text((GtkEntry *)snakelibgl_visualizationScreenshotFilenameTextBox));
}

GtkWidget* create_snakelibgl_window (void) {

  // Setup the GTK window for the visualization GUI
  snakelibgl_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_set_usize (snakelibgl_window, 550, 950);
  gtk_window_set_title (GTK_WINDOW (snakelibgl_window), _("CardioARM Visualization"));
  gtk_window_set_icon_name (GTK_WINDOW (snakelibgl_window), "gtk-about");
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_window), 0);

  // Create a notebook for the visualization GUI
  GtkWidget *snakelibgl_notebook = gtk_notebook_new ();
  gtk_widget_show (snakelibgl_notebook);
  gtk_container_add (GTK_CONTAINER (snakelibgl_window), snakelibgl_notebook);

  // Setup a GTK vbox for visualization
  GtkWidget *snakelibgl_vbox_visualization = gtk_vbox_new (FALSE, 0);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_vbox_visualization), 10);
  gtk_container_add (GTK_CONTAINER (snakelibgl_notebook), snakelibgl_vbox_visualization);
  GtkWidget *snakelibgl_tabLabel_visualization = gtk_label_new (_("Visualization"));
  gtk_notebook_set_tab_label (GTK_NOTEBOOK (snakelibgl_notebook), gtk_notebook_get_nth_page (GTK_NOTEBOOK (snakelibgl_notebook), 0), snakelibgl_tabLabel_visualization);
  gtk_widget_show (snakelibgl_vbox_visualization);

  // Setup a GTK vbox for ascension
  GtkWidget *snakelibgl_vbox_ascension = gtk_vbox_new (FALSE, 0);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_vbox_ascension), 10);
  gtk_container_add (GTK_CONTAINER (snakelibgl_notebook), snakelibgl_vbox_ascension);
  GtkWidget *snakelibgl_tabLabel_ascension = gtk_label_new (_("Ascension Tracker"));
  gtk_notebook_set_tab_label (GTK_NOTEBOOK (snakelibgl_notebook), gtk_notebook_get_nth_page (GTK_NOTEBOOK (snakelibgl_notebook), 1), snakelibgl_tabLabel_ascension);
  gtk_widget_show (snakelibgl_vbox_ascension);

  // Setup a GTK vbox for snake estimation
  GtkWidget *snakelibgl_vbox_snakeEstimation = gtk_vbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_vbox_snakeEstimation), 10);
  gtk_container_add(GTK_CONTAINER(snakelibgl_notebook), snakelibgl_vbox_snakeEstimation);
  GtkWidget *snakelibgl_tabLabel_snakeEstimation = gtk_label_new(_("Snake Estimation"));
  gtk_notebook_set_tab_label(GTK_NOTEBOOK(snakelibgl_notebook), gtk_notebook_get_nth_page(GTK_NOTEBOOK(snakelibgl_notebook), 2), snakelibgl_tabLabel_snakeEstimation);
  gtk_widget_show(snakelibgl_vbox_snakeEstimation);

  // Setup a GTK vbox for autonomy stuff
  GtkWidget *snakelibgl_vbox_autonomy = gtk_vbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_vbox_autonomy), 10);
  gtk_container_add(GTK_CONTAINER(snakelibgl_notebook), snakelibgl_vbox_autonomy);
  GtkWidget *snakelibgl_tabLabel_autonomy = gtk_label_new(_("Autonomy"));
  gtk_notebook_set_tab_label(GTK_NOTEBOOK(snakelibgl_notebook), gtk_notebook_get_nth_page(GTK_NOTEBOOK(snakelibgl_notebook), 3), snakelibgl_tabLabel_autonomy);
  gtk_widget_show(snakelibgl_vbox_autonomy);

  // Setup a GTK vbox for logging stuff
  GtkWidget *snakelibgl_vbox_logging = gtk_vbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_vbox_logging), 10);
  gtk_container_add(GTK_CONTAINER(snakelibgl_notebook), snakelibgl_vbox_logging);
  GtkWidget *snakelibgl_tabLabel_logging = gtk_label_new(_("Logging"));
  gtk_notebook_set_tab_label(GTK_NOTEBOOK(snakelibgl_notebook), gtk_notebook_get_nth_page(GTK_NOTEBOOK(snakelibgl_notebook), 4), snakelibgl_tabLabel_logging);
  gtk_widget_show(snakelibgl_vbox_logging);

  //-----------------------------------------------------------
  // visualization tab stuff

  // Setup a GTK table
  snakelibgl_visualizationTable = gtk_table_new (11, 1, FALSE);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 0, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 1, 5);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 2, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 3, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 4, 30);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 5, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 6, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 7, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 8, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 9, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_visualizationTable), 10, 10);
  gtk_table_set_col_spacing (GTK_TABLE (snakelibgl_visualizationTable), 0, 0);
  gtk_box_pack_start (GTK_BOX (snakelibgl_vbox_visualization), snakelibgl_visualizationTable, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_visualizationTable);

  // Add a text label to designate that this stuff is for loading polygon meshes
  GtkWidget *snakelibgl_visualizationScreenshotTitleLabel = gtk_label_new (_("Screenshot"));
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationScreenshotTitleLabel, 0, 1, 0, 1, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  PangoFontDescription *myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_visualizationScreenshotTitleLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show(snakelibgl_visualizationScreenshotTitleLabel);
  
  // Add a text label to lable the filename text box for the screenshot
  GtkWidget *snakelibgl_visualizationScreenshotFilenameLabel = gtk_label_new (_("Screenshot Filename: "));
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationScreenshotFilenameLabel, 0, 1, 1, 2, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_visualizationScreenshotFilenameLabel);

  // Add an editable text box to the table for the screenshot filename
  snakelibgl_visualizationScreenshotFilenameTextBox = gtk_entry_new();
  gtk_entry_set_width_chars(GTK_ENTRY(snakelibgl_visualizationScreenshotFilenameTextBox), 30);
  gtk_entry_set_text(GTK_ENTRY(snakelibgl_visualizationScreenshotFilenameTextBox), "_screenshot1.ppm");
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationScreenshotFilenameTextBox, 0, 1, 2, 3, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_visualizationScreenshotFilenameTextBox);

  // Add a button to take a screenshot
  GtkWidget *snakelibgl_visualizationTakeScreenshotButton = gtk_button_new_with_label("Capture Screenshot");
  gtk_table_attach(GTK_TABLE(snakelibgl_visualizationTable), snakelibgl_visualizationTakeScreenshotButton, 0, 1, 3, 4, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_set_size_request(snakelibgl_visualizationTakeScreenshotButton, 110, 30);
  gtk_widget_show(snakelibgl_visualizationTakeScreenshotButton);

  // Add a H separator to the visualization box
  GtkWidget *snakelibgl_visualizationScreenshotHseparator = gtk_hseparator_new ();
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationScreenshotHseparator, 0, 1, 4, 5, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_visualizationScreenshotHseparator);

  // Add a text label to designate that this stuff is for loding polygon meshes
  GtkWidget *snakelibgl_visualizationPolygonMeshTitleLabel = gtk_label_new (_("Load Polygon Mesh"));
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationPolygonMeshTitleLabel, 0, 1, 5, 6, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_visualizationPolygonMeshTitleLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show(snakelibgl_visualizationPolygonMeshTitleLabel);

  // Add a text label to lable the filename text box
  GtkWidget *snakelibgl_visualizationPolygonMeshFilenameLabel = gtk_label_new (_("Polygon Mesh Filename: "));
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationPolygonMeshFilenameLabel, 0, 1, 6, 7, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_visualizationPolygonMeshFilenameLabel);

  // Add an editable text box to the table for the polygon mesh filename
  snakelibgl_visualizationPolygonMeshFilenameTextBox = gtk_entry_new();
  gtk_entry_set_width_chars(GTK_ENTRY(snakelibgl_visualizationPolygonMeshFilenameTextBox), 30);
  gtk_entry_set_text(GTK_ENTRY(snakelibgl_visualizationPolygonMeshFilenameTextBox), "pinkBox.ply");
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationPolygonMeshFilenameTextBox, 0, 1, 7, 8, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_visualizationPolygonMeshFilenameTextBox);

  // Add a button to load the polygon mesh filename in the text box
  GtkWidget *snakelibgl_visualizationPolygonMeshLoadButton = gtk_button_new_with_label ("Load Polygon Mesh");
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationPolygonMeshLoadButton, 0, 1, 8, 9, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_set_size_request (snakelibgl_visualizationPolygonMeshLoadButton, 110, 30);
  gtk_widget_show (snakelibgl_visualizationPolygonMeshLoadButton);

  // Add a H separator to the polygon mesh table
  GtkWidget *snakelibgl_visualizationPolygonMeshHseparator = gtk_hseparator_new ();
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationPolygonMeshHseparator, 0, 1, 9, 10, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_visualizationPolygonMeshHseparator);

  // Add a hbox for the polygon mesh list labels
  GtkWidget *snakelibgl_visualizationHboxForListLabels = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_visualizationHboxForListLabels), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_visualizationHboxForListLabels, 10);
  gtk_table_attach (GTK_TABLE (snakelibgl_visualizationTable), snakelibgl_visualizationHboxForListLabels, 0, 1, 10, 11, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_visualizationHboxForListLabels);

  // Add labels to the box
  GtkWidget *snakelibgl_visualizationPolygonMeshListFilenameLabel = gtk_label_new (_("    Filename:"));
  gtk_box_pack_start (GTK_BOX (snakelibgl_visualizationHboxForListLabels), snakelibgl_visualizationPolygonMeshListFilenameLabel, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_visualizationPolygonMeshListFilenameLabel);
  GtkWidget *snakelibgl_visualizationPolygonMeshListTransparencyLabel = gtk_label_new (_("             Transparency:"));
  gtk_box_pack_start (GTK_BOX (snakelibgl_visualizationHboxForListLabels), snakelibgl_visualizationPolygonMeshListTransparencyLabel, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_visualizationPolygonMeshListTransparencyLabel, 150, 15);
  gtk_widget_show (snakelibgl_visualizationPolygonMeshListTransparencyLabel);

  //-----------------------------------------------------------
  // ascension tab stuff

  // Setup a GTK table
  GtkWidget *snakelibgl_ascensionTable = gtk_table_new (26, 1, FALSE);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 0, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 1, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 2, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 3, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 4, 4);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 5, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 6, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 7, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 8, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 9, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 10, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 11, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 12, 30);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 13, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 14, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 15, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 16, 30);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 17, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 18, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 19, 4);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 20, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 21, 30);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 22, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 23, 4);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 24, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_ascensionTable), 25, 20);
  gtk_table_set_col_spacing (GTK_TABLE (snakelibgl_ascensionTable), 0, 0);
  gtk_box_pack_start (GTK_BOX (snakelibgl_vbox_ascension), snakelibgl_ascensionTable, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionTable);

  // Add a text label to designate that these buttons are for registration
  GtkWidget *snakelibgl_ascensionRegistrationLabel = gtk_label_new (_("Registration"));
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionRegistrationLabel, 0, 1, 0, 1, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_ascensionRegistrationLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show (snakelibgl_ascensionRegistrationLabel);

  // Add a text label to label the model registration points filename text box
  GtkWidget *snakelibgl_ascensionModelPointsFilenameLabel = gtk_label_new (_("Model Registration Pts Filename:"));
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionModelPointsFilenameLabel, 0, 1, 1, 2, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionModelPointsFilenameLabel);

  // Add an hbox to hold some widgets for loading the model registration points
  GtkWidget *snakelibgl_ascensionHbox1 = gtk_hbox_new (FALSE, 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionHbox1, 10);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_ascensionHbox1), 0);
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHbox1, 0, 1, 2, 3, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHbox1);

  // Add an editable text box to the hbox for the model registration points filename
  snakelibgl_ascensionModelPointsFilenameTextBox = gtk_entry_new();
  gtk_entry_set_width_chars(GTK_ENTRY(snakelibgl_ascensionModelPointsFilenameTextBox), 40);
  gtk_entry_set_text(GTK_ENTRY(snakelibgl_ascensionModelPointsFilenameTextBox), "pinkBoxModelRegistrationPoints.dat");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox1), snakelibgl_ascensionModelPointsFilenameTextBox, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionModelPointsFilenameTextBox);

  // Add a button to the hbox to load the model registration points from file
  GtkWidget *snakelibgl_ascensionLoadModelPointsButton = gtk_button_new_with_label ("Load Model Registration Points");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox1), snakelibgl_ascensionLoadModelPointsButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionLoadModelPointsButton, 200, 25);
  gtk_widget_show (snakelibgl_ascensionLoadModelPointsButton);

  // Add a text label to lable the tracker registration points filename text box
  GtkWidget *snakelibgl_ascensionTrackerPointsFilenameLabel = gtk_label_new (_("Tracker Registration Pts Filename:"));
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionTrackerPointsFilenameLabel, 0, 1, 3, 4, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionTrackerPointsFilenameLabel);

  // Add an hbox to hold some widgets for saving the tracker registration points
  GtkWidget *snakelibgl_ascensionHbox2 = gtk_hbox_new (FALSE, 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionHbox2, 10);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_ascensionHbox2), 0);
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHbox2, 0, 1, 4, 5, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHbox2);

  // Add an editable text box to the hbox for the tracker registration points filename
  snakelibgl_ascensionTrackerPointsFilenameTextBox = gtk_entry_new();
  gtk_entry_set_width_chars(GTK_ENTRY(snakelibgl_ascensionTrackerPointsFilenameTextBox), 40);
  gtk_entry_set_text(GTK_ENTRY(snakelibgl_ascensionTrackerPointsFilenameTextBox), "pinkBoxTrackerRegistrationPoints.dat");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox2), snakelibgl_ascensionTrackerPointsFilenameTextBox, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionTrackerPointsFilenameTextBox);

  // Add a button to the hbox to save the tracker registration points to file
  GtkWidget *snakelibgl_ascensionSaveTrackerPointsButton = gtk_button_new_with_label ("Save Tracker Registration Points");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox2), snakelibgl_ascensionSaveTrackerPointsButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionSaveTrackerPointsButton, 200, 25);
  gtk_widget_show (snakelibgl_ascensionSaveTrackerPointsButton);

  // Add an hbox to hold a widget for loading the tracker registration points
  GtkWidget *snakelibgl_ascensionHbox3 = gtk_hbox_new (FALSE, 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionHbox3, 10);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_ascensionHbox3), 0);
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHbox3, 0, 1, 5, 6, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHbox3);

  // Add a dummy empty text label to take up space
  GtkWidget *snakelibgl_ascensionLoadTrackerPointsEmptyLabel = gtk_label_new (_("    "));
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox3), snakelibgl_ascensionLoadTrackerPointsEmptyLabel, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionLoadTrackerPointsEmptyLabel, 248, 25);
  gtk_widget_show (snakelibgl_ascensionLoadTrackerPointsEmptyLabel);

  // Add a button to the hbox to load the tracker registration points from file
  GtkWidget *snakelibgl_ascensionLoadTrackerPointsButton = gtk_button_new_with_label ("Load Tracker Registration Points");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox3), snakelibgl_ascensionLoadTrackerPointsButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionLoadTrackerPointsButton, 200, 25);
  gtk_widget_show (snakelibgl_ascensionLoadTrackerPointsButton);

  // Add a H separator to the ascension table
  GtkWidget *snakelibgl_ascensionHseparator1 = gtk_hseparator_new ();
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHseparator1, 0, 1, 6, 7, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHseparator1);

  // Add another hbox to hold a button and a label
  GtkWidget *snakelibgl_ascensionHbox4 = gtk_hbox_new (FALSE, 0);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_ascensionHbox4), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionHbox4, 10);
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHbox4, 0, 1, 7, 8, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHbox4);

  // Add a button to the hbox that clears the tracker registration points
  GtkWidget *snakelibgl_ascensionClearTrackerPtsButton = gtk_button_new_with_label ("Clear Tracker Pts.");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox4), snakelibgl_ascensionClearTrackerPtsButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionClearTrackerPtsButton, 120, 25);
  gtk_widget_show (snakelibgl_ascensionClearTrackerPtsButton);

  // Add a label to the hbox that shows the number of tracker registration points
  snakelibgl_ascensionNumTrackerPtsLabel = gtk_label_new (_("Num Tracker Pts: 0"));
  gtk_widget_set_size_request (snakelibgl_ascensionNumTrackerPtsLabel, 120, 20);
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox4), snakelibgl_ascensionNumTrackerPtsLabel, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionNumTrackerPtsLabel);  

  // Add another hbox to hold a button and a label
  GtkWidget *snakelibgl_ascensionHbox5 = gtk_hbox_new (FALSE, 0);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_ascensionHbox5), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionHbox5, 10);
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHbox5, 0, 1, 8, 9, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHbox5);

  // Add a button to the hbox to clear the model registration points
  GtkWidget *snakelibgl_ascensionClearModelPtsButton = gtk_button_new_with_label ("Clear Model Pts.");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox5), snakelibgl_ascensionClearModelPtsButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionClearModelPtsButton, 120, 25);
  gtk_widget_show (snakelibgl_ascensionClearModelPtsButton);

  // Add a label to the hbox that shows the number of model registration points
  snakelibgl_ascensionNumModelPtsLabel = gtk_label_new (_("Num Model Pts: 0"));
  gtk_widget_set_size_request (snakelibgl_ascensionNumModelPtsLabel, 120, 20);
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox5), snakelibgl_ascensionNumModelPtsLabel, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionNumModelPtsLabel); 

  // Add another H separator to the ascension table
  GtkWidget *snakelibgl_ascensionHseparator2 = gtk_hseparator_new ();
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHseparator2, 0, 1, 9, 10, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHseparator2);

  // Add a button to capture a tracker registration point
  GtkWidget *snakelibgl_ascensionCaptureTrackerPointButton = gtk_button_new_with_label ("Capture Tracker Registration Point");
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionCaptureTrackerPointButton, 0, 1, 10, 11, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionCaptureTrackerPointButton);
  gtk_widget_set_size_request (snakelibgl_ascensionCaptureTrackerPointButton, 200, 25);
 
  // Add an hbox to hold some widgets for performing the registration
  GtkWidget *snakelibgl_ascensionHbox6 = gtk_hbox_new(FALSE, 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionHbox6, 10);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_ascensionHbox6), 0);
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHbox6, 0, 1, 11, 12, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHbox6);

  // Add a button to perform the registration
  GtkWidget *snakelibgl_ascensionRegisterButton = gtk_button_new_with_label("Perform Registration");
  gtk_widget_set_size_request(snakelibgl_ascensionRegisterButton, 120, 25);
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox6), snakelibgl_ascensionRegisterButton, TRUE, TRUE, 0);
  gtk_widget_show(snakelibgl_ascensionRegisterButton);

  // Add a label to the hbox that shows the number of model registration points
  snakelibgl_ascensionRegistrationErrorLabel = gtk_label_new (_("Avg. Registration Error: 0.00 mm"));
  gtk_widget_set_size_request (snakelibgl_ascensionRegistrationErrorLabel, 200, 20);
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox6), snakelibgl_ascensionRegistrationErrorLabel, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionRegistrationErrorLabel); 

  // Add another H separator to the ascension table
  GtkWidget *snakelibgl_ascensionHseparator3 = gtk_hseparator_new();
  gtk_table_attach(GTK_TABLE(snakelibgl_ascensionTable), snakelibgl_ascensionHseparator3, 0, 1, 12, 13, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionHseparator3);

  // Add a text label to designate that these buttons are for registration
  GtkWidget *snakelibgl_ascensionLoggingLabel = gtk_label_new(_("Log Ascension Points"));
  gtk_table_attach(GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionLoggingLabel, 0, 1, 13, 14, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_ascensionLoggingLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show(snakelibgl_ascensionLoggingLabel);

  // Add a text label to label the log tracker point filename text box
  GtkWidget *snakelibgl_ascensionLogTrackerPointFilenameLabel = gtk_label_new (_("Log Tracker Point Filename:"));
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionLogTrackerPointFilenameLabel, 0, 1, 14, 15, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionLogTrackerPointFilenameLabel);

  // Add an hbox to hold some widgets for logging the tracker points
  GtkWidget *snakelibgl_ascensionHbox7 = gtk_hbox_new(FALSE, 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionHbox7, 10);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_ascensionHbox7), 0);
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionHbox7, 0, 1, 15, 16, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionHbox7);

  // Add an editable text box to the hbox for the log tracker point filename
  snakelibgl_ascensionLogTrackerPointTextBox = gtk_entry_new();
  gtk_entry_set_width_chars(GTK_ENTRY(snakelibgl_ascensionLogTrackerPointTextBox), 30);
  gtk_entry_set_text(GTK_ENTRY(snakelibgl_ascensionLogTrackerPointTextBox), "_trackerLog1.dat");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox7), snakelibgl_ascensionLogTrackerPointTextBox, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionLogTrackerPointTextBox);

  // Add a button to the hbox to log the tracker point to file
  GtkWidget *snakelibgl_ascensionLogTrackerPointButton = gtk_button_new_with_label ("Log Tracker Point");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionHbox7), snakelibgl_ascensionLogTrackerPointButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionLogTrackerPointButton, 200, 30);
  gtk_widget_show (snakelibgl_ascensionLogTrackerPointButton);

  // Add an hbox to hold some widgets for continuosly logging the tracker points
  GtkWidget *snakelibgl_ascensionContinousAscensionLogHbox = gtk_hbox_new(FALSE, 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionContinousAscensionLogHbox, 10);
  gtk_container_set_border_width (GTK_CONTAINER (snakelibgl_ascensionContinousAscensionLogHbox), 0);
  gtk_table_attach (GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionContinousAscensionLogHbox, 0, 1, 16, 17, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_ascensionContinousAscensionLogHbox);

  // Add an editable text box to the hbox for the continuous log tracker point filename
  snakelibgl_ascensionContinuousLogTrackerTextBox = gtk_entry_new();
  gtk_entry_set_width_chars(GTK_ENTRY(snakelibgl_ascensionContinuousLogTrackerTextBox), 30);
  gtk_entry_set_text(GTK_ENTRY(snakelibgl_ascensionContinuousLogTrackerTextBox), "_continuousLog1.dat");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionContinousAscensionLogHbox), snakelibgl_ascensionContinuousLogTrackerTextBox, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionContinuousLogTrackerTextBox);

  // Add a button to the hbox to start/stop continuously logging the tracker points to file
  snakelibgl_ascensionContinuousLogTrackerButton = gtk_button_new_with_label ("Continuous Start");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionContinousAscensionLogHbox), snakelibgl_ascensionContinuousLogTrackerButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionContinuousLogTrackerButton, 200, 30);
  gtk_widget_show (snakelibgl_ascensionContinuousLogTrackerButton);

  // Add another H separator to the ascension table
  GtkWidget *snakelibgl_ascensionHseparator4 = gtk_hseparator_new();
  gtk_table_attach(GTK_TABLE(snakelibgl_ascensionTable), snakelibgl_ascensionHseparator4, 0, 1, 17, 18, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionHseparator4);

  // Add a text label to designate that these buttons are for the sensor visuals
  GtkWidget *snakelibgl_ascensionSensorVisualsLabel = gtk_label_new(_("Sensor Visuals"));
  gtk_table_attach(GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionSensorVisualsLabel, 0, 1, 18, 19, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_ascensionSensorVisualsLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show(snakelibgl_ascensionSensorVisualsLabel);

  // Add a text label for the sliders below this
  GtkWidget *snakelibgl_ascensionSensorVisualsSlidersLabel = gtk_label_new(_("                    Alpha:                                             Size:"));
  gtk_table_attach(GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionSensorVisualsSlidersLabel, 0, 1, 19, 20, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionSensorVisualsSlidersLabel);

  // Add a hbox for the sensor visuals stuff
  GtkWidget *snakelibgl_ascensionSensorVisualsHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_ascensionSensorVisualsHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionSensorVisualsHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_ascensionTable), snakelibgl_ascensionSensorVisualsHbox, 0, 1, 20, 21, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionSensorVisualsHbox);

  // Add a checkbox for displaying the sensor
  snakelibgl_ascensionSensorVisualsCheckBox = gtk_check_button_new_with_label("On/Off");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionSensorVisualsHbox), snakelibgl_ascensionSensorVisualsCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_ascensionSensorVisualsCheckBox, (gint)TRUE);
  gtk_widget_show (snakelibgl_ascensionSensorVisualsCheckBox);

  // Add a slider for the alpha value
  snakelibgl_ascensionSensorVisualsAlphaSlider = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new(1.0, 0.0, 1.0, 0.05, 1, 0)));
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionSensorVisualsHbox), snakelibgl_ascensionSensorVisualsAlphaSlider, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionSensorVisualsAlphaSlider);
  gtk_widget_set_size_request (snakelibgl_ascensionSensorVisualsAlphaSlider, 150, 35);
  gtk_scale_set_digits (GTK_SCALE (snakelibgl_ascensionSensorVisualsAlphaSlider), 2);

  // Add a slider for the size value
  snakelibgl_ascensionSensorVisualsSizeSlider = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new(4.5, 0.0, 20.0, 0.05, 1, 0)));
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionSensorVisualsHbox), snakelibgl_ascensionSensorVisualsSizeSlider, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionSensorVisualsSizeSlider);
  gtk_widget_set_size_request (snakelibgl_ascensionSensorVisualsSizeSlider, 150, 35);
  gtk_scale_set_digits (GTK_SCALE (snakelibgl_ascensionSensorVisualsSizeSlider), 2);

  // Add another H separator to the ascension table
  GtkWidget *snakelibgl_ascensionHseparator5 = gtk_hseparator_new();
  gtk_table_attach(GTK_TABLE(snakelibgl_ascensionTable), snakelibgl_ascensionHseparator5, 0, 1, 21, 22, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionHseparator5);

  // Add a text label to designate that these buttons are for showing a trail of tracker points
  GtkWidget *snakelibgl_ascensionSensorTrailsLabel = gtk_label_new(_("Sensor Trails"));
  gtk_table_attach(GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionSensorTrailsLabel, 0, 1, 22, 23, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_ascensionSensorTrailsLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show(snakelibgl_ascensionSensorTrailsLabel);

  // Add a text label for the sliders below this
  GtkWidget *snakelibgl_ascensionSensorTrailsSliderLabel = gtk_label_new(_("Alpha:                             Size:                                                    "));
  gtk_table_attach(GTK_TABLE (snakelibgl_ascensionTable), snakelibgl_ascensionSensorTrailsSliderLabel, 0, 1, 23, 24, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionSensorTrailsSliderLabel);

  // Add an hbox for sensor trail stuff
  GtkWidget *snakelibgl_ascensionSensorTrailsHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_ascensionSensorTrailsHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionSensorTrailsHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_ascensionTable), snakelibgl_ascensionSensorTrailsHbox, 0, 1, 24, 25, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionSensorTrailsHbox);

  // Add a checkbox for displaying the trails
  snakelibgl_ascensionSensorTrailsCheckBox = gtk_check_button_new_with_label("On/Off");
  gtk_box_pack_start(GTK_BOX(snakelibgl_ascensionSensorTrailsHbox), snakelibgl_ascensionSensorTrailsCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_ascensionSensorTrailsCheckBox, (gint)TRUE);
  gtk_widget_show (snakelibgl_ascensionSensorTrailsCheckBox);

  // Add a slider for the alpha value
  snakelibgl_ascensionSensorTrailsAlphaSlider = gtk_hscale_new(GTK_ADJUSTMENT(gtk_adjustment_new(1.0, 0.0, 1.0, 0.05, 1, 0)));
  gtk_box_pack_start(GTK_BOX(snakelibgl_ascensionSensorTrailsHbox), snakelibgl_ascensionSensorTrailsAlphaSlider, TRUE, TRUE, 0);
  gtk_widget_show(snakelibgl_ascensionSensorTrailsAlphaSlider);
  gtk_widget_set_size_request(snakelibgl_ascensionSensorTrailsAlphaSlider, 100, 35);
  gtk_scale_set_digits(GTK_SCALE(snakelibgl_ascensionSensorTrailsAlphaSlider), 2);

  // Add a slider for the trail diameter size
  snakelibgl_ascensionSensorTrailsSizeSlider = gtk_hscale_new(GTK_ADJUSTMENT(gtk_adjustment_new(4.5, 0.0, 20.0, 0.05, 1, 0)));
  gtk_box_pack_start(GTK_BOX(snakelibgl_ascensionSensorTrailsHbox), snakelibgl_ascensionSensorTrailsSizeSlider, TRUE, TRUE, 0);
  gtk_widget_show(snakelibgl_ascensionSensorTrailsSizeSlider);
  gtk_widget_set_size_request(snakelibgl_ascensionSensorTrailsSizeSlider, 100, 35);
  gtk_scale_set_digits(GTK_SCALE(snakelibgl_ascensionSensorTrailsSizeSlider), 2);

  // Add a button to the hbox to start/stop the trail
  snakelibgl_ascensionSensorTrailsStartStopButton = gtk_button_new_with_label("Start");
  gtk_box_pack_start(GTK_BOX(snakelibgl_ascensionSensorTrailsHbox), snakelibgl_ascensionSensorTrailsStartStopButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request(snakelibgl_ascensionSensorTrailsStartStopButton, 100, 25);
  gtk_widget_show (snakelibgl_ascensionSensorTrailsStartStopButton);

  // Add a button to the hbox to clear the trail
  GtkWidget *snakelibgl_ascensionSensorTrailsClearButton = gtk_button_new_with_label("Clear");
  gtk_box_pack_start(GTK_BOX(snakelibgl_ascensionSensorTrailsHbox), snakelibgl_ascensionSensorTrailsClearButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request(snakelibgl_ascensionSensorTrailsClearButton, 100, 25);
  gtk_widget_show (snakelibgl_ascensionSensorTrailsClearButton);

  // Add an hbox for more sensor trail stuff
  GtkWidget *snakelibgl_ascensionSensorTrailsHbox2 = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_ascensionSensorTrailsHbox2), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionSensorTrailsHbox2, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_ascensionTable), snakelibgl_ascensionSensorTrailsHbox2, 0, 1, 25, 26, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionSensorTrailsHbox2);

  // Add an editable text box to the hbox for saving trails data
  snakelibgl_ascensionSaveTrailDataTextBox = gtk_entry_new();
  gtk_entry_set_width_chars(GTK_ENTRY(snakelibgl_ascensionSaveTrailDataTextBox), 30);
  gtk_entry_set_text(GTK_ENTRY(snakelibgl_ascensionSaveTrailDataTextBox), "_trailLog1.dat");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionSensorTrailsHbox2), snakelibgl_ascensionSaveTrailDataTextBox, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_ascensionSaveTrailDataTextBox);

  // Add a button to the hbox to save the trail data
  GtkWidget *snakelibgl_ascensionSensorTrailsSaveButton = gtk_button_new_with_label("Save Trails Data");
  gtk_box_pack_start(GTK_BOX(snakelibgl_ascensionSensorTrailsHbox2), snakelibgl_ascensionSensorTrailsSaveButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request(snakelibgl_ascensionSensorTrailsSaveButton, 200, 25);
  gtk_widget_show (snakelibgl_ascensionSensorTrailsSaveButton);

  // Add another hbox for sensor trail stuff
  GtkWidget *snakelibgl_ascensionSensorTrailsHbox3 = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_ascensionSensorTrailsHbox3), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_ascensionSensorTrailsHbox3, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_ascensionTable), snakelibgl_ascensionSensorTrailsHbox3, 0, 1, 26, 27, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_ascensionSensorTrailsHbox3);

  // Add a checkbox for displaying the trails
  snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox = gtk_check_button_new_with_label("Advance/Retract Only");
  gtk_box_pack_start(GTK_BOX(snakelibgl_ascensionSensorTrailsHbox3), snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox, (gint)TRUE);
  gtk_widget_show (snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox);

  // Add a button to add a single instance to trails
  GtkWidget *snakelibgl_ascensionSensorTrailsAddSingleButton = gtk_button_new_with_label ("Add Single Instance");
  gtk_box_pack_start (GTK_BOX (snakelibgl_ascensionSensorTrailsHbox3), snakelibgl_ascensionSensorTrailsAddSingleButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request (snakelibgl_ascensionSensorTrailsAddSingleButton, 200, 30);
  gtk_widget_show (snakelibgl_ascensionSensorTrailsAddSingleButton);

  //-----------------------------------------------------------
  // snake estimation tab stuff

  // Setup a GTK table
  GtkWidget *snakelibgl_snakeEstimationTable = gtk_table_new (10, 1, FALSE);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 0, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 1, 1);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 2, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 3, 1);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 4, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 5, 1);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 6, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 7, 1);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 8, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 9, 10);
  gtk_table_set_col_spacing (GTK_TABLE (snakelibgl_snakeEstimationTable), 0, 0);
  gtk_box_pack_start (GTK_BOX (snakelibgl_vbox_snakeEstimation), snakelibgl_snakeEstimationTable, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_snakeEstimationTable);

  // Add a text label to designate that these buttons are for snake estimation
  GtkWidget *snakelibgl_snakeEstimationLabel = gtk_label_new (_("Snake Estimation"));
  gtk_table_attach (GTK_TABLE (snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationLabel, 0, 1, 0, 1, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_snakeEstimationLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show (snakelibgl_snakeEstimationLabel);

  // Add a text label to label the alpha for the kalm slider
  GtkWidget *snakelibgl_snakeEstimationAlphaFullKalmLabel = gtk_label_new (_("                                                   Alpha:     "));
  gtk_table_attach (GTK_TABLE (snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationAlphaFullKalmLabel, 0, 1, 1, 2, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_snakeEstimationAlphaFullKalmLabel);

  // Add an hbox for full kalm estimation stuff
  GtkWidget *snakelibgl_snakeEstimationFullKalmHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_snakeEstimationFullKalmHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_snakeEstimationFullKalmHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationFullKalmHbox, 0, 1, 2, 3, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_snakeEstimationFullKalmHbox);

  // Add a label to the hbox that labels that this is kalm stuff
  GtkWidget *snakelibgl_snakeEstimationFullKalmLabel = gtk_label_new (_("Kalm: "));
  gtk_widget_set_size_request (snakelibgl_snakeEstimationFullKalmLabel, 60, 20);
  gtk_box_pack_start (GTK_BOX (snakelibgl_snakeEstimationFullKalmHbox), snakelibgl_snakeEstimationFullKalmLabel, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_snakeEstimationFullKalmLabel); 

  // Add a checkbox for displaying the kalm
  snakelibgl_snakeEstimationFullKalmCheckBox = gtk_check_button_new_with_label("On/Off");
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullKalmHbox), snakelibgl_snakeEstimationFullKalmCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_snakeEstimationFullKalmCheckBox, (gint)FALSE);
  gtk_widget_show (snakelibgl_snakeEstimationFullKalmCheckBox);

  // Add a checkbox for updating
  snakelibgl_snakeEstimationFullKalmUpdateCheckBox = gtk_check_button_new_with_label("Update");
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullKalmHbox), snakelibgl_snakeEstimationFullKalmUpdateCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_snakeEstimationFullKalmUpdateCheckBox, (gint)TRUE);
  gtk_widget_show (snakelibgl_snakeEstimationFullKalmUpdateCheckBox);

  // Add a slider for the alpha value
  snakelibgl_snakeEstimationAlphaFullKalmSlider = gtk_hscale_new(GTK_ADJUSTMENT(gtk_adjustment_new(0.3, 0.0, 1.0, 0.05, 1, 0)));
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullKalmHbox), snakelibgl_snakeEstimationAlphaFullKalmSlider, TRUE, TRUE, 0);
  gtk_widget_show(snakelibgl_snakeEstimationAlphaFullKalmSlider);
  gtk_widget_set_size_request(snakelibgl_snakeEstimationAlphaFullKalmSlider, 150, 35);
  gtk_scale_set_digits(GTK_SCALE(snakelibgl_snakeEstimationAlphaFullKalmSlider), 2);

  // Add a text label to label the alpha for the iterated slider
  GtkWidget *snakelibgl_snakeEstimationAlphaFullIteratedLabel = gtk_label_new (_("                                                   Alpha:     "));
  gtk_table_attach (GTK_TABLE (snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationAlphaFullIteratedLabel, 0, 1, 3, 4, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_snakeEstimationAlphaFullIteratedLabel);

  // Add an hbox for full iterated estimation stuff
  GtkWidget *snakelibgl_snakeEstimationFullIteratedHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_snakeEstimationFullIteratedHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_snakeEstimationFullIteratedHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationFullIteratedHbox, 0, 1, 4, 5, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_snakeEstimationFullIteratedHbox);

  // Add a label to the hbox that labels that this is iterated stuff
  GtkWidget *snakelibgl_snakeEstimationFullIteratedLabel = gtk_label_new (_("Iterated: "));
  gtk_widget_set_size_request (snakelibgl_snakeEstimationFullIteratedLabel, 60, 20);
  gtk_box_pack_start (GTK_BOX (snakelibgl_snakeEstimationFullIteratedHbox), snakelibgl_snakeEstimationFullIteratedLabel, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_snakeEstimationFullIteratedLabel); 

  // Add a checkbox for displaying the iterated
  snakelibgl_snakeEstimationFullIteratedCheckBox = gtk_check_button_new_with_label("On/Off");
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullIteratedHbox), snakelibgl_snakeEstimationFullIteratedCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_snakeEstimationFullIteratedCheckBox, (gint)FALSE);
  gtk_widget_show (snakelibgl_snakeEstimationFullIteratedCheckBox);

  // Add a checkbox for updating
  snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox = gtk_check_button_new_with_label("Simplify");
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullIteratedHbox), snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox, (gint)TRUE);
  gtk_widget_show (snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox);

  // Add a checkbox for updating
  snakelibgl_snakeEstimationFullIteratedUpdateCheckBox = gtk_check_button_new_with_label("Update");
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullIteratedHbox), snakelibgl_snakeEstimationFullIteratedUpdateCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_snakeEstimationFullIteratedUpdateCheckBox, (gint)TRUE);
  gtk_widget_show (snakelibgl_snakeEstimationFullIteratedUpdateCheckBox);

  // Add a slider for the alpha value
  snakelibgl_snakeEstimationAlphaFullIteratedSlider = gtk_hscale_new(GTK_ADJUSTMENT(gtk_adjustment_new(0.3, 0.0, 1.0, 0.05, 1, 0)));
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullIteratedHbox), snakelibgl_snakeEstimationAlphaFullIteratedSlider, TRUE, TRUE, 0);
  gtk_widget_show(snakelibgl_snakeEstimationAlphaFullIteratedSlider);
  gtk_widget_set_size_request(snakelibgl_snakeEstimationAlphaFullIteratedSlider, 150, 35);
  gtk_scale_set_digits(GTK_SCALE(snakelibgl_snakeEstimationAlphaFullIteratedSlider), 2);

  // Add a text label to label the alpha for the dead slider
  GtkWidget *snakelibgl_snakeEstimationAlphaFullDeadLabel = gtk_label_new (_("                                                   Alpha:     "));
  gtk_table_attach (GTK_TABLE (snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationAlphaFullDeadLabel, 0, 1, 5, 6, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_snakeEstimationAlphaFullDeadLabel);

  // Add an hbox for dead estimation stuff
  GtkWidget *snakelibgl_snakeEstimationFullDeadHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_snakeEstimationFullDeadHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_snakeEstimationFullDeadHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationFullDeadHbox, 0, 1, 6, 7, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_snakeEstimationFullDeadHbox);

  // Add a label to the hbox that labels that this is dead stuff
  GtkWidget *snakelibgl_snakeEstimationFullDeadLabel = gtk_label_new (_("Dead: "));
  gtk_widget_set_size_request (snakelibgl_snakeEstimationFullDeadLabel, 60, 20);
  gtk_box_pack_start (GTK_BOX (snakelibgl_snakeEstimationFullDeadHbox), snakelibgl_snakeEstimationFullDeadLabel, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_snakeEstimationFullDeadLabel); 

  // Add a checkbox for displaying the dead
  snakelibgl_snakeEstimationFullDeadCheckBox = gtk_check_button_new_with_label("On/Off");
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullDeadHbox), snakelibgl_snakeEstimationFullDeadCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_snakeEstimationFullDeadCheckBox, (gint)FALSE);
  gtk_widget_show (snakelibgl_snakeEstimationFullDeadCheckBox);

  // Add a slider for the alpha value
  snakelibgl_snakeEstimationAlphaFullDeadSlider = gtk_hscale_new(GTK_ADJUSTMENT(gtk_adjustment_new(0.3, 0.0, 1.0, 0.05, 1, 0)));
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationFullDeadHbox), snakelibgl_snakeEstimationAlphaFullDeadSlider, TRUE, TRUE, 0);
  gtk_widget_show(snakelibgl_snakeEstimationAlphaFullDeadSlider);
  gtk_widget_set_size_request(snakelibgl_snakeEstimationAlphaFullDeadSlider, 150, 35);
  gtk_scale_set_digits(GTK_SCALE(snakelibgl_snakeEstimationAlphaFullDeadSlider), 2);

  // Add a text label to label the alpha for the distal slider
  GtkWidget *snakelibgl_snakeEstimationAlphaDistalKalmLabel = gtk_label_new (_("                                                   Alpha:     "));
  gtk_table_attach (GTK_TABLE (snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationAlphaDistalKalmLabel, 0, 1, 7, 8, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_snakeEstimationAlphaDistalKalmLabel);

  // Add an hbox for distal estimation stuff
  GtkWidget *snakelibgl_snakeEstimationDistalKalmHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_snakeEstimationDistalKalmHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_snakeEstimationDistalKalmHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationDistalKalmHbox, 0, 1, 8, 9, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_snakeEstimationDistalKalmHbox);

  // Add a label to the hbox that labels that this is distal stuff
  GtkWidget *snakelibgl_snakeEstimationDistalKalmLabel = gtk_label_new (_("Distal: "));
  gtk_widget_set_size_request (snakelibgl_snakeEstimationDistalKalmLabel, 60, 20);
  gtk_box_pack_start (GTK_BOX (snakelibgl_snakeEstimationDistalKalmHbox), snakelibgl_snakeEstimationDistalKalmLabel, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_snakeEstimationDistalKalmLabel); 

  // Add a checkbox for displaying the distal
  snakelibgl_snakeEstimationDistalKalmCheckBox = gtk_check_button_new_with_label("On/Off");
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationDistalKalmHbox), snakelibgl_snakeEstimationDistalKalmCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_snakeEstimationDistalKalmCheckBox, (gint)FALSE);
  gtk_widget_show (snakelibgl_snakeEstimationDistalKalmCheckBox);

  // Add a checkbox for updating
  snakelibgl_snakeEstimationDistalKalmUpdateCheckBox = gtk_check_button_new_with_label("Update");
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationDistalKalmHbox), snakelibgl_snakeEstimationDistalKalmUpdateCheckBox, TRUE, TRUE, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_snakeEstimationDistalKalmUpdateCheckBox, (gint)TRUE);
  gtk_widget_show (snakelibgl_snakeEstimationDistalKalmUpdateCheckBox);

  // Add a slider for the alpha value
  snakelibgl_snakeEstimationAlphaDistalKalmSlider = gtk_hscale_new(GTK_ADJUSTMENT(gtk_adjustment_new(0.3, 0.0, 1.0, 0.05, 1, 0)));
  gtk_box_pack_start(GTK_BOX(snakelibgl_snakeEstimationDistalKalmHbox), snakelibgl_snakeEstimationAlphaDistalKalmSlider, TRUE, TRUE, 0);
  gtk_widget_show(snakelibgl_snakeEstimationAlphaDistalKalmSlider);
  gtk_widget_set_size_request(snakelibgl_snakeEstimationAlphaDistalKalmSlider, 150, 35);
  gtk_scale_set_digits(GTK_SCALE(snakelibgl_snakeEstimationAlphaDistalKalmSlider), 2);

  // Add a H separator to the snake estimation
  GtkWidget *snakelibgl_snakeEstimationHseparator = gtk_hseparator_new ();
  gtk_table_attach (GTK_TABLE (snakelibgl_snakeEstimationTable), snakelibgl_snakeEstimationHseparator, 0, 1, 9, 10, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_snakeEstimationHseparator);

  //-----------------------------------------------------------
  // autonomy tab stuff

  // Setup a GTK table
  GtkWidget *snakelibgl_autonomyTable = gtk_table_new(9, 1, FALSE);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 0, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 1, 5);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 2, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 3, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 4, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 5, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 6, 20);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 7, 10);
  gtk_table_set_row_spacing (GTK_TABLE (snakelibgl_autonomyTable), 8, 10);
  gtk_table_set_col_spacing (GTK_TABLE (snakelibgl_autonomyTable), 0, 0);
  gtk_box_pack_start (GTK_BOX (snakelibgl_vbox_autonomy), snakelibgl_autonomyTable, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_autonomyTable);

  // Add a text label to designate that these buttons are for autonomy
  GtkWidget *snakelibgl_autonomyLabel = gtk_label_new (_("Autonomy Control Points"));
  gtk_table_attach (GTK_TABLE (snakelibgl_autonomyTable), snakelibgl_autonomyLabel, 0, 1, 0, 1, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_autonomyLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show (snakelibgl_autonomyLabel);

  // Add a text label to describe picking points
  GtkWidget *snakelibgl_autonomyPickPointsDescriptionLabel = gtk_label_new (_("Pick path points:"));
  gtk_table_attach (GTK_TABLE (snakelibgl_autonomyTable), snakelibgl_autonomyPickPointsDescriptionLabel, 0, 1, 1, 2, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_autonomyPickPointsDescriptionLabel);

  // Add an hbox for point picking stuff
  GtkWidget *snakelibgl_autonomyPointPickingHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_autonomyPointPickingHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_autonomyPointPickingHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_autonomyTable), snakelibgl_autonomyPointPickingHbox, 0, 1, 2, 3, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_autonomyPointPickingHbox);

  // Add a button to the hbox to start/stop point picking
  snakelibgl_autonomyStartStopButton = gtk_button_new_with_label("Start");
  gtk_box_pack_start(GTK_BOX(snakelibgl_autonomyPointPickingHbox), snakelibgl_autonomyStartStopButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request(snakelibgl_autonomyStartStopButton, 150, 25);
  gtk_widget_show (snakelibgl_autonomyStartStopButton);

  // Add a button to the hbox to clear the picked points
  GtkWidget *snakelibgl_autonomyClearPathButton = gtk_button_new_with_label("Clear Points/Path");
  gtk_box_pack_start(GTK_BOX(snakelibgl_autonomyPointPickingHbox), snakelibgl_autonomyClearPathButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request(snakelibgl_autonomyClearPathButton, 150, 25);
  gtk_widget_show (snakelibgl_autonomyClearPathButton);

  // Add a checkbox for visible flag for pre curve path points
  snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox = gtk_check_button_new_with_label("Surface Path Points Visible");
  gtk_table_attach(GTK_TABLE(snakelibgl_autonomyTable), snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox, 0, 1, 3, 4, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox, (gint)TRUE);
  gtk_widget_show(snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox);

  // Add a checkbox for visible flag for pre curve path points
  snakelibgl_autonomyVisibleSplineCurveCheckBox = gtk_check_button_new_with_label("Target Curve Visible");
  gtk_table_attach(GTK_TABLE(snakelibgl_autonomyTable), snakelibgl_autonomyVisibleSplineCurveCheckBox, 0, 1, 4, 5, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_autonomyVisibleSplineCurveCheckBox, (gint)TRUE);
  gtk_widget_show(snakelibgl_autonomyVisibleSplineCurveCheckBox);

  // Add a H separator to the autonomy box
  GtkWidget *snakelibgl_autonomyHseparator = gtk_hseparator_new ();
  gtk_table_attach (GTK_TABLE (snakelibgl_autonomyTable), snakelibgl_autonomyHseparator, 0, 1, 5, 6, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_autonomyHseparator);

  // Add a text label to designate that these buttons are for autonomy
  GtkWidget *snakelibgl_autonomyEnableDisableLabel = gtk_label_new (_("Enable/Disable Autonomous Driving"));
  gtk_table_attach (GTK_TABLE (snakelibgl_autonomyTable), snakelibgl_autonomyEnableDisableLabel, 0, 1, 6, 7, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_autonomyEnableDisableLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show (snakelibgl_autonomyEnableDisableLabel);

  // Add a button to enable autonomy
  snakelibgl_autonomyEnableDisableButton = gtk_button_new_with_label("Enable Autonomy");
  gtk_table_attach (GTK_TABLE (snakelibgl_autonomyTable), snakelibgl_autonomyEnableDisableButton, 0, 1, 7, 8, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_set_size_request(snakelibgl_autonomyEnableDisableButton, 200, 30);
  gtk_widget_show (snakelibgl_autonomyEnableDisableButton);

  // Add a H separator to the autonomy box
  GtkWidget *snakelibgl_autonomyHseparator2 = gtk_hseparator_new ();
  gtk_table_attach (GTK_TABLE (snakelibgl_autonomyTable), snakelibgl_autonomyHseparator2, 0, 1, 8, 9, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_autonomyHseparator2);

  //-----------------------------------------------------------
  // logging tab stuff

  // Setup a GTK table
  GtkWidget *snakelibgl_loggingTable = gtk_table_new(10, 1, FALSE);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 0, 20);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 1, 5);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 2, 5);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 3, 5);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 4, 5);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 5, 5);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 6, 10);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 7, 5);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 8, 10);
  gtk_table_set_row_spacing(GTK_TABLE (snakelibgl_loggingTable), 9, 5);
  gtk_box_pack_start(GTK_BOX(snakelibgl_vbox_logging), snakelibgl_loggingTable, TRUE, TRUE, 0);
  gtk_widget_show(snakelibgl_loggingTable);

  // Add a text label to designate that these buttons are for logging
  GtkWidget *snakelibgl_loggingLabel = gtk_label_new(_("Logging"));
  gtk_table_attach (GTK_TABLE (snakelibgl_loggingTable), snakelibgl_loggingLabel, 0, 1, 0, 1, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  myPangoFontDescription = pango_font_description_new();
  pango_font_description_set_size(myPangoFontDescription, 20);
  gtk_widget_modify_font(snakelibgl_loggingLabel, myPangoFontDescription);
  pango_font_description_free(myPangoFontDescription);
  gtk_widget_show(snakelibgl_loggingLabel);

  // Add an hbox for on/off logging
  GtkWidget *snakelibgl_loggingOnOffHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_loggingOnOffHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_loggingOnOffHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_loggingTable), snakelibgl_loggingOnOffHbox, 0, 1, 1, 2, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_loggingOnOffHbox);

  // Add a button to the hbox to turn logging on
  snakelibgl_loggingAllOnButton = gtk_button_new_with_label("All On");
  gtk_box_pack_start(GTK_BOX(snakelibgl_loggingOnOffHbox), snakelibgl_loggingAllOnButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request(snakelibgl_loggingAllOnButton, 150, 25);
  gtk_widget_show(snakelibgl_loggingAllOnButton);

  // Add a button to the hbox to turn logging off
  snakelibgl_loggingAllOffButton = gtk_button_new_with_label("All Off");
  gtk_box_pack_start(GTK_BOX(snakelibgl_loggingOnOffHbox), snakelibgl_loggingAllOffButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request(snakelibgl_loggingAllOffButton, 150, 25);
  gtk_widget_show(snakelibgl_loggingAllOffButton);

  // Add a checkbox for logging ascension tracker
  snakelibgl_loggingAscensionTrackerCheckBox = gtk_check_button_new_with_label("Log Ascension Tracker");
  gtk_table_attach(GTK_TABLE(snakelibgl_loggingTable), snakelibgl_loggingAscensionTrackerCheckBox, 0, 1, 2, 3, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingAscensionTrackerCheckBox, (gint)TRUE);
  gtk_widget_show(snakelibgl_loggingAscensionTrackerCheckBox);

  // Add a checkbox for logging registration stuff
  snakelibgl_loggingRegistrationCheckBox = gtk_check_button_new_with_label("Log Registration");
  gtk_table_attach(GTK_TABLE(snakelibgl_loggingTable), snakelibgl_loggingRegistrationCheckBox, 0, 1, 3, 4, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingRegistrationCheckBox, (gint)TRUE);
  gtk_widget_show(snakelibgl_loggingRegistrationCheckBox);

  // Add a checkbox for logging snake estimation stuff
  snakelibgl_loggingSnakeEstimationCheckBox = gtk_check_button_new_with_label("Log Snake Estimation");
  gtk_table_attach(GTK_TABLE(snakelibgl_loggingTable), snakelibgl_loggingSnakeEstimationCheckBox, 0, 1, 4, 5, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingSnakeEstimationCheckBox, (gint)TRUE);
  gtk_widget_show(snakelibgl_loggingSnakeEstimationCheckBox);

  // Add a checkbox for logging autonomy stuff
  snakelibgl_loggingAutonomyCheckBox = gtk_check_button_new_with_label("Log Autonomy");
  gtk_table_attach(GTK_TABLE(snakelibgl_loggingTable), snakelibgl_loggingAutonomyCheckBox, 0, 1, 5, 6, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_toggle_button_set_active((GtkToggleButton *)snakelibgl_loggingAutonomyCheckBox, (gint)TRUE);
  gtk_widget_show(snakelibgl_loggingAutonomyCheckBox);

  // Add a H separator to the logging box
  GtkWidget *snakelibgl_loggingHseparator = gtk_hseparator_new();
  gtk_table_attach(GTK_TABLE(snakelibgl_loggingTable), snakelibgl_loggingHseparator, 0, 1, 6, 7, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_loggingHseparator);

  // Add a text label to describe start new log file
  GtkWidget *snakelibgl_loggingStartNewLogFileLabel = gtk_label_new (_("Start New Log File:"));
  gtk_table_attach (GTK_TABLE (snakelibgl_loggingTable), snakelibgl_loggingStartNewLogFileLabel, 0, 1, 7, 8, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show (snakelibgl_loggingStartNewLogFileLabel);

  // Add an hbox for starting new log file stuff
  GtkWidget *snakelibgl_loggingStartNewLogFileHbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(snakelibgl_loggingStartNewLogFileHbox), 0);
  gtk_box_set_spacing((GtkBox *)snakelibgl_loggingStartNewLogFileHbox, 10);
  gtk_table_attach(GTK_TABLE(snakelibgl_loggingTable), snakelibgl_loggingStartNewLogFileHbox, 0, 1, 8, 9, (GtkAttachOptions) 0, (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_loggingStartNewLogFileHbox);

  // Add an editable text box to the hbox for starting new log file (filename)
  snakelibgl_loggingStartNewLogFileTextBox = gtk_entry_new();
  gtk_entry_set_width_chars(GTK_ENTRY(snakelibgl_loggingStartNewLogFileTextBox), 30);
  gtk_entry_set_text(GTK_ENTRY(snakelibgl_loggingStartNewLogFileTextBox), "_defaultVisualizationLog.log");
  gtk_box_pack_start (GTK_BOX (snakelibgl_loggingStartNewLogFileHbox), snakelibgl_loggingStartNewLogFileTextBox, TRUE, TRUE, 0);
  gtk_widget_show (snakelibgl_loggingStartNewLogFileTextBox);

  // Add a button to the hbox to start new log
  GtkWidget *snakelibgl_loggingStartNewLogFileButton = gtk_button_new_with_label("Start New Log");
  gtk_box_pack_start(GTK_BOX(snakelibgl_loggingStartNewLogFileHbox), snakelibgl_loggingStartNewLogFileButton, TRUE, TRUE, 0);
  gtk_widget_set_size_request(snakelibgl_loggingStartNewLogFileButton, 200, 25);
  gtk_widget_show (snakelibgl_loggingStartNewLogFileButton);

  // Add a H separator to the logging box
  GtkWidget *snakelibgl_loggingHseparator2 = gtk_hseparator_new();
  gtk_table_attach(GTK_TABLE(snakelibgl_loggingTable), snakelibgl_loggingHseparator2, 0, 1, 9, 10, (GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) 0, 0, 0);
  gtk_widget_show(snakelibgl_loggingHseparator2);

  //-----------------------------------------------------------

  g_signal_connect ((gpointer) snakelibgl_ascensionLogTrackerPointButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionLogTrackerPointButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionRegisterButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionRegisterButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionClearModelPtsButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionClearModelPtsButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionLoadModelPointsButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionLoadModelPointsButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSaveTrackerPointsButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionSaveTrackerPointsButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionLoadTrackerPointsButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionLoadTrackerPointsButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionClearTrackerPtsButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionClearTrackerPtsButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionContinuousLogTrackerButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionContinuousLogTrackerButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionCaptureTrackerPointButton, "clicked",
                    G_CALLBACK (snakelibgl_ascensionCaptureTrackerPointButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_visualizationPolygonMeshLoadButton, "clicked",
                    G_CALLBACK (snakelibgl_visualizationPolygonMeshFileLoadButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_visualizationTakeScreenshotButton, "clicked",
                    G_CALLBACK (snakelibgl_visualizationTakeScreenshotButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorVisualsSizeSlider, "value_changed",
					G_CALLBACK (snakelibgl_ascensionSensorVisualsSizeSlider_value_changed), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorTrailsSizeSlider, "value_changed",
					G_CALLBACK (snakelibgl_ascensionSensorTrailsSizeSlider_value_changed), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorVisualsAlphaSlider, "value_changed",
					G_CALLBACK (snakelibgl_ascensionSensorVisualsAlphaSlider_value_changed), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorVisualsCheckBox, "toggled",
					G_CALLBACK (snakelibgl_ascensionSensorVisualsCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorTrailsStartStopButton, "clicked",
					G_CALLBACK (snakelibgl_ascensionSensorTrailsStartStopButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorTrailsClearButton, "clicked",
					G_CALLBACK (snakelibgl_ascensionSensorTrailsClearButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorTrailsCheckBox, "toggled",
					G_CALLBACK (snakelibgl_ascensionSensorTrailsCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorTrailsAlphaSlider, "value_changed",
					G_CALLBACK (snakelibgl_ascensionSensorTrailsAlphaSlider_value_changed), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorTrailsSaveButton, "clicked",
					G_CALLBACK (snakelibgl_ascensionSensorTrailsSaveButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorTrailsAddSingleButton, "clicked",
					G_CALLBACK (snakelibgl_ascensionSensorTrailsAddSingleButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox, "toggled",
					G_CALLBACK (snakelibgl_ascensionSensorTrailsTriggerOnlyCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_autonomyStartStopButton, "clicked",
					G_CALLBACK (snakelibgl_autonomyStartStopButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox, "toggled",
					G_CALLBACK (snakelibgl_autonomyVisiblePreCurvePathPointsCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_autonomyVisibleSplineCurveCheckBox, "toggled",
					G_CALLBACK (snakelibgl_autonomyVisibleSplineCurveCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_autonomyClearPathButton, "clicked",
					G_CALLBACK (snakelibgl_autonomyClearPathButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationFullDeadCheckBox, "toggled",
					G_CALLBACK (snakelibgl_snakeEstimationFullDeadCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationFullKalmCheckBox, "toggled",
					G_CALLBACK (snakelibgl_snakeEstimationFullKalmCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationFullIteratedCheckBox, "toggled",
					G_CALLBACK (snakelibgl_snakeEstimationFullIteratedCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox, "toggled",
					G_CALLBACK (snakelibgl_snakeEstimationSimplifiedUpdateFullIteratedCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationDistalKalmCheckBox, "toggled",
					G_CALLBACK (snakelibgl_snakeEstimationDistalKalmCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationAlphaFullDeadSlider, "value_changed",
					G_CALLBACK (snakelibgl_snakeEstimationAlphaFullDeadSlider_value_changed), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationAlphaFullKalmSlider, "value_changed",
					G_CALLBACK (snakelibgl_snakeEstimationAlphaFullKalmSlider_value_changed), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationAlphaFullIteratedSlider, "value_changed",
					G_CALLBACK (snakelibgl_snakeEstimationAlphaFullIteratedSlider_value_changed), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationAlphaDistalKalmSlider, "value_changed",
					G_CALLBACK (snakelibgl_snakeEstimationAlphaDistalKalmSlider_value_changed), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationDistalKalmUpdateCheckBox, "toggled",
					G_CALLBACK (snakelibgl_snakeEstimationDistalKalmUpdateCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationFullKalmUpdateCheckBox, "toggled",
					G_CALLBACK (snakelibgl_snakeEstimationFullKalmUpdateCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_snakeEstimationFullIteratedUpdateCheckBox, "toggled",
					G_CALLBACK (snakelibgl_snakeEstimationFullIteratedUpdateCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_autonomyEnableDisableButton, "clicked",
					G_CALLBACK (snakelibgl_autonomyEnableDisableButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_loggingAllOnButton, "clicked",
					G_CALLBACK (snakelibgl_loggingAllOnButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_loggingAllOffButton, "clicked",
					G_CALLBACK (snakelibgl_loggingAllOffButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_loggingSnakeEstimationCheckBox, "toggled",
					G_CALLBACK (snakelibgl_loggingSnakeEstimationCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_loggingRegistrationCheckBox, "toggled",
					G_CALLBACK (snakelibgl_loggingRegistrationCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_loggingAscensionTrackerCheckBox, "toggled",
					G_CALLBACK (snakelibgl_loggingAscensionTrackerCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_loggingAutonomyCheckBox, "toggled",
					G_CALLBACK (snakelibgl_loggingAutonomyCheckBox_toggled), NULL);
  g_signal_connect ((gpointer) snakelibgl_loggingStartNewLogFileButton, "clicked",
					G_CALLBACK (snakelibgl_loggingStartNewLogFileButton_clicked), NULL);
  g_signal_connect ((gpointer) snakelibgl_window, "delete_event",
                    G_CALLBACK (on_quit), NULL);
  g_signal_connect ((gpointer) snakelibgl_window, "destroy_event",
                    G_CALLBACK (gtk_main_quit), NULL);
  g_signal_connect ((gpointer) snakelibgl_window, "destroy",
					G_CALLBACK (gtk_main_quit), NULL);

  //-----------------------------------------------------------

  return snakelibgl_window;
}

