/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  snakelibgl_gui.h                                                                     |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions to handle the GUI for the snakelibgl files.                   |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#ifndef SNAKELIBGL_GUI_H
#define SNAKELIBGL_GUI_H

void snakelibgl_snakeEstimationDeadAlphaSlider_value_changed(void);
void snakelibgl_snakeEstimationKalmAlphaSlider_value_changed(void);
void snakelibgl_snakeEstimationDeadCheckBox_toggled(void);
void snakelibgl_snakeEstimationKalmCheckBox_toggled(void);
void snakelibgl_ascensionSensorTrailsSaveButton_clicked(void);
void snakelibgl_ascensionSensorTrailsAlphaSlider_value_changed(void);
void snakelibgl_ascensionSensorTrailsCheckBox_toggled(void);
void snakelibgl_ascensionSensorTrailsClearButton_clicked(void);
void snakelibgl_ascensionSensorTrailsStartStopButton_clicked(void);
void snakelibgl_ascensionSensorVisualsCheckBox_toggled(void);
void snakelibgl_ascensionSensorVisualsSizeSlider_value_changed(void);
void snakelibgl_ascensionSensorVisualsAlphaSlider_value_changed(void);
void snakelibgl_ascensionLogTrackerPointButton_clicked(void);
void snakelibgl_ascensionRegisterButton_clicked(void);
void snakelibgl_ascensionClearModelPtsButton_clicked(void);
void snakelibgl_ascensionSaveTrackerPointsButton_clicked(void);
void snakelibgl_ascensionLoadTrackerPointsButton_clicked (void);
void snakelibgl_ascensionLoadModelPointsButton_clicked(void);
void snakelibgl_ascensionClearTrackerPtsButton_clicked (void);
void snakelibgl_ascensionCaptureTrackerPointButton_clicked(void);
void snakelibgl_visualizationPolygonMeshFileLoadButton_clicked(void);
void snakelibgl_visualizationPolygonMeshFileCheckButton_toggled(GtkWidget *checkButton);
void snakelibgl_visualizationPolygonMeshFileAlphaSlider_value_changed(GtkWidget *alphaSlider);
void snakelibgl_visualizationTakeScreenshotButton_clicked(void);
GtkWidget* create_snakelibgl_window (void);

#endif SNAKELIBGL_GUI_H
