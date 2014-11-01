/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  polygonMesh.cpp                                                                      |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Custom functions to parse, store, and draw polygon meshes.              |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <GL/glut.h>
#include <gsl/gsl_blas.h>
#include "snakelibgl.h"
#include "polygonMesh.h"

/******************************************************************************\
*                                                                              *
*  polygonMesh::getVertexValue()                                               *
*                                                                              *
*  This function returns the vertex value based on the input index.            *
*                                                                              *
\******************************************************************************/
GLfloat polygonMesh::getVertexValue(int input) {
	return vertices[input];
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::getIndex()                                                     *
*                                                                              *
*  This function returns the vertex index based on the polygon input index.    *
*                                                                              *
\******************************************************************************/
int polygonMesh::getIndex(int input) {
	return indices[input];
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::setUpdateDrawFlag()                                            *
*                                                                              *
*  This function sets the update draw flag.                                    *
*                                                                              *
\******************************************************************************/
void polygonMesh::setUpdateDrawFlag(bool val) {
	updateDrawFlag = val;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::setVisibleFlag()                                               *
*                                                                              *
*  This function sets the visible flag.                                        *
*                                                                              *
\******************************************************************************/
void polygonMesh::setVisibleFlag(bool val) {
	visible = val;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::getUpdateDrawFlag()                                            *
*                                                                              *
*  This function returns the update draw flag.                                 *
*                                                                              *
\******************************************************************************/
bool polygonMesh::getUpdateDrawFlag() {
	return updateDrawFlag;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::getVisibleFlag()                                               *
*                                                                              *
*  This function returns the visible flag.                                     *
*                                                                              *
\******************************************************************************/
bool polygonMesh::getVisibleFlag() {
	return visible;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::getNumVertices()                                               *
*                                                                              *
*  This function returns the number of vertices.                               *
*                                                                              *
\******************************************************************************/
int polygonMesh::getNumVertices() {
	return numVertices;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::getNumPolygons()                                               *
*                                                                              *
*  This function returns the number of polygons.                               *
*                                                                              *
\******************************************************************************/
int polygonMesh::getNumPolygons() {
	return numPolygons;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::getNumVerticesPerPolygon()                                     *
*                                                                              *
*  This function returns the numver of vertices per polygon.                   *
*                                                                              *
\******************************************************************************/
int polygonMesh::getNumVerticesPerPolygon() {
	return numVerticesPerPolygon;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::setAlpha()                                                     *
*                                                                              *
*  This function sets the transparency alpha for drawing.                      *
*                                                                              *
\******************************************************************************/
bool polygonMesh::setAlpha(float val) {
	if (val < 0.0) return false;
	else if (val > 1.0) return false;
	else alpha = val;
	return true;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::polygonMesh()                                                  *
*                                                                              *
*  This function is the constructor for the polygonMesh class. The function    *
*     calls the init() function to intialize variables.                        *
*                                                                              *
\******************************************************************************/
polygonMesh::polygonMesh() {
	init();
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::~polygonMesh()                                                 *
*                                                                              *
*  This function is the destructor for the polygonMesh class. The function     *
*     calls the destroyMemory() function to clear allocated memory.            *
*                                                                              *
\******************************************************************************/
polygonMesh::~polygonMesh() {
	destroyMemory();
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::init()                                                         *
*                                                                              *
*  This function initializes variables for the class.                          *
*                                                                              *
\******************************************************************************/
bool polygonMesh::init() {
	visible = true;
	updateDrawFlag = false;
	alpha = 1.0;
	numVertices = 0;
	numPolygons = 0;
	numVerticesPerPolygon = 0;
	vertices = NULL;
	normals = NULL;
	colors = NULL;
	indices = NULL;
	return true;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::destroyMemory()                                                *
*                                                                              *
*  This function clears allocated memory for the vertex and faces arrays.      *
*                                                                              *
\******************************************************************************/
bool polygonMesh::destroyMemory() {
	if (vertices) free(vertices);
	if (normals) free(normals);
	if (colors) free(colors);
	if (indices) free(indices);
	init();
	return true;
}

/******************************************************************************\
*                                                                              *
*  polygon::loadPolygonMeshFile()                                              *
*                                                                              *
*  This function calls the appropriate helper function based on the file type  *
*     in order to parse a polygon mesh file and load the data into memory.     *
*                                                                              *
\******************************************************************************/
bool polygonMesh::loadPolygonMeshFile(char *filename) {

	if (!filename) {
		printf("\tError: polygon mesh, empty polygon mesh filename.\n\n");
		return false;
	} else if (strlen(filename) <= 4) {
		printf("\tError: polygon mesh, improper filename given.\n\n");
		return false;
	} else if (!strcmp(&filename[strlen(filename)-4], ".ply")) {
		if (!parsePLY(filename)) {
			printf("\tError: polygon mesh, could not parse PLY file.\n\n");
			return false;
		}
	} else {
		printf("\tError: polygon mesh, filename problem.\n\n");
		return false;
	}
	updateDrawFlag = true;
	return true;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::parsePLY()                                                     *
*                                                                              *
*  This function parses a PLY file and fills in the vertices and faces info    *
*     for the private variables. Memory is created.                            *
*                                                                              *
\******************************************************************************/
bool polygonMesh::parsePLY(char *filename) {

	destroyMemory();
	init();

	char *token, *context;
	FILE *plyFP = NULL;
	fopen_s(&plyFP, filename, "r");
	if (!plyFP) {
		printf("\tError: polygon mesh, could not load polygon mesh file.\n\n");
		return false;
	}

    int x_index = -1, y_index = -1, z_index = -1;
    int nx_index = -1, ny_index = -1, nz_index = -1;
    int red_index = -1, green_index = -1, blue_index = -1;

	char lineArray[MAX_TEXT_LENGTH];
	char *lineArrayPtr = &lineArray[0];
	char lineArrayCopy[MAX_TEXT_LENGTH];
	char *lineArrayCopyPtr = &lineArrayCopy[0];
	lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
	lineArrayPtr[3] = 0;
	if (strcmp("ply", lineArrayPtr)) {
		printf("\tError: polygon mesh, did not see 'ply' header in file.\n\n");
		fclose(plyFP); return false;
	}
	lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
	lineArrayPtr[12] = 0;
	if (strcmp("ascii",&lineArrayPtr[7])) {
		printf("\tError: polygon mesh, ply file needs to be ASCII format.\n\n");
		fclose(plyFP); return false;
	}
	lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
	lineArrayPtr[14] = 0;
	while (strcmp("element vertex", lineArrayPtr)) {
		lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
		lineArrayPtr[14] = 0;
	}
	numVertices = atoi(&lineArrayPtr[15]);
	lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
    memcpy(lineArrayCopyPtr, lineArrayPtr, sizeof(char)*MAX_TEXT_LENGTH);          
	lineArrayCopyPtr[12] = 0;
	int numPropertiesRead = 0;
	while (strcmp("element face", lineArrayCopyPtr)) {
          memcpy(lineArrayCopyPtr, lineArrayPtr, sizeof(char)*MAX_TEXT_LENGTH);          
          lineArrayCopyPtr[8] = 0;
          if (strcmp("property", lineArrayCopyPtr)) {
                printf("\tError: polygon mesh, ply file has unexpected formatting.\n\n");
                fclose(plyFP); destroyMemory(); init(); return false;
          }
		  if ((token = strtok_s(lineArrayPtr," \r\n\t", &context)) == NULL) {
                printf("\tError: polygon mesh, something went wrong loading ply file (1).\n\n");
                fclose(plyFP); destroyMemory(); init(); return false;
		  }
		  if ((token = strtok_s(NULL," \r\n\t", &context)) == NULL) {
                printf("\tError: polygon mesh, something went wrong loading ply file (2).\n\n");
                fclose(plyFP); destroyMemory(); init(); return false;
		  }
		  if ((token = strtok_s(NULL," \r\n\t", &context)) == NULL) {
                printf("\tError: polygon mesh, something went wrong loading ply file (3).\n\n");
                fclose(plyFP); destroyMemory(); init(); return false;
		  }
          if (!strcmp("x",token)) {
                x_index = numPropertiesRead;
          } else if (!strcmp("y",token)) {
                y_index = numPropertiesRead;
          } else if (!strcmp("z",token)) {
                z_index = numPropertiesRead;       
          } else if (!strcmp("nx",token)) {
                nx_index = numPropertiesRead;                 
          } else if (!strcmp("ny",token)) {
                ny_index = numPropertiesRead;                 
          } else if (!strcmp("nz", token)) {
                nz_index = numPropertiesRead;                 
          } else if (!strcmp("red",token)) {
                red_index = numPropertiesRead;                 
          } else if (!strcmp("green",token)) {
                green_index = numPropertiesRead;                 
          } else if (!strcmp("blue",token)) {
                blue_index = numPropertiesRead;                 
          } else {
                numPropertiesRead--;
          }
          numPropertiesRead++;
          lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
          memcpy(lineArrayCopyPtr, lineArrayPtr, sizeof(char)*MAX_TEXT_LENGTH);          
          lineArrayCopyPtr[12] = 0;
	}
	numPolygons = atoi(&lineArrayPtr[13]);
	while (strcmp("end_header", lineArrayPtr)) {
		lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
		lineArrayPtr[10] = 0;
	}
	
	if (x_index<0 || y_index<0 || z_index<0 || nx_index<0 || ny_index<0 || nz_index<0 || red_index<0 || green_index<0 || blue_index<0) {
          printf("\tError: polygon mesh, one or more PLY properties not included.\n\n");
		  fclose(plyFP); destroyMemory(); init(); return false;                  
    }
    
    if (numPropertiesRead != 9) {
          printf("\tError: polygon mesh, incorrect number of PLY properties read.\n\n");
		  fclose(plyFP); destroyMemory(); init(); return false;
    }	

	vertices = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*3);
	normals = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*3);
	colors = (GLfloat *)malloc(sizeof(GLfloat)*numVertices*4);

	for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < 4; j++) {
            colors[4*i+j] = 1.0;
            if (j < 3) {
                  vertices[3*i+j] = 0.0;
                  normals[3*i+j] = 0.0;
            }
        }
	}

	for (int i = 0; i < numVertices; i++) {
		lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
		token = strtok_s(lineArrayPtr," \r\n\t", &context);
		int j = 0;
		while (token != NULL && j < numPropertiesRead) {
              if (j == x_index) {
                    vertices[3*i] = (GLfloat)atof(token);                       
              } else if (j == y_index) {
                    vertices[3*i+1] = (GLfloat)atof(token);                                            
              } else if (j == z_index) {
                    vertices[3*i+2] = (GLfloat)atof(token);                                            
              } else if (j == nx_index) {
                    normals[3*i] = (GLfloat)atof(token);                                            
              } else if (j == ny_index) {
                    normals[3*i+1] = (GLfloat)atof(token);                                                                 
              } else if (j == nz_index) {
                    normals[3*i+2] = (GLfloat)atof(token);                                                                 
              } else if (j == red_index) {
                    colors[4*i] = (GLfloat)(atof(token)/255.0);                                                                
              } else if (j == green_index) {
                    colors[4*i+1] = (GLfloat)(atof(token)/255.0);                                                                                     
              } else if (j == blue_index) {
                    colors[4*i+2] = (GLfloat)(atof(token)/255.0);                                                                                     
              } else {
                     printf("\tError: polygon mesh, something went wrong reading PLY file (4).\n\n");       
		             fclose(plyFP); destroyMemory(); init(); return false;
              }
		      token = strtok_s(NULL," \r\n\t", &context);
		      j++;
		}
	}

	indices = (unsigned int *)malloc(sizeof(unsigned int)*numPolygons*3);

	for (int i = 0; i < numPolygons; i++) {
		lineArrayPtr = fgets(lineArrayPtr, MAX_TEXT_LENGTH, plyFP);
		token = strtok_s(lineArrayPtr," \r\n\t", &context);
		if (i == 0) {
			numVerticesPerPolygon = atoi(token);
			if (numVerticesPerPolygon != 3) {
				printf("\tError: polygon mesh, can only display triangle meshes right now.\n\n");
          		fclose(plyFP); destroyMemory(); init(); return false;
			}
		} else if (atoi(token) != numVerticesPerPolygon) {
            printf("\tError: polygon mesh, found a polygon in the PLY file that was not a triangle.\n\n");
    		fclose(plyFP); destroyMemory(); init(); return false;
		}
		token = strtok_s(NULL," \r\n\t", &context);
		int j = 0;
		while (token != NULL && j < numVerticesPerPolygon) {
			indices[3*i+(j++)] = (unsigned int)atoi(token);
		    token = strtok_s(NULL," \r\n\t", &context);
		}
	}
	fclose(plyFP);
	return true;
}

/******************************************************************************\
*                                                                              *
*  polygonMesh::drawPolygonMesh()                                              *
*                                                                              *
*  This function draws the polygon mesh based on the vertex locations and the  *
*     indices array, which defines the neighboring vertices.                   *
*                                                                              *
\******************************************************************************/
bool polygonMesh::drawPolygonMesh(int glListIndex) {

	if (!vertices || !colors || !normals || !indices) {
		printf("\tError: polygon mesh, could not draw polygon mesh.\n\n");
		return false;
	}
	for (int i = 0; i < numVertices; i++)
		colors[4*i+3] = alpha;

	glNewList(glListIndex, GL_COMPILE);
	glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
	glNormalPointer(GL_FLOAT, 0, normals);
    glColorPointer(4, GL_FLOAT, 0, colors);
    glVertexPointer(3, GL_FLOAT, 0, vertices);
    glDrawElements(GL_TRIANGLES, numPolygons*3, GL_UNSIGNED_INT, indices);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
	glEndList();

	updateDrawFlag = false;
	return true;
}
