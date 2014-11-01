/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  polygonMesh.h                                                                        |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Custom functions to parse, store, and draw polygon meshes.              |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#ifndef POLYGONMESH_H
#define POLYGONMESH_H

class polygonMesh {
public:

	polygonMesh();
	~polygonMesh();

	void setUpdateDrawFlag(bool val);
	void setVisibleFlag(bool val);
	bool getUpdateDrawFlag(void);
	bool getVisibleFlag(void);
	bool loadPolygonMeshFile(char *filename);
	bool drawPolygonMesh(int glListIndex);
	bool setAlpha(float val);
	int getNumVertices(void);
	int getNumPolygons(void);
	int getNumVerticesPerPolygon(void);
	int getIndex(int input);
	GLfloat getVertexValue(int input);

private:

	bool init();
	bool destroyMemory();
	bool parsePLY(char *filename);

	int numVertices;
	int numPolygons;
	int numVerticesPerPolygon;
	bool visible;
	bool updateDrawFlag;
	float alpha;
	GLfloat *vertices;
	GLfloat *normals;
	GLfloat *colors;
	unsigned int *indices;

};

#endif POLYGONMESH_H
