// Obj_loader.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include "objLoader.h"

void printVector(obj_vector *v)
{
	printf("(%.2f), ", v->e[0] );
	printf("(%.2f), ", v->e[1] );
	printf("(%.2f)  ", v->e[2] );
}

int main(int argc, char **argv)
{
	objLoader *objData = new objLoader();
	objData->load("20mm_cube.obj");

	printf("const int VertexCount = %i\n", objData->vertexCount);
	printf("const int IndexCount = %i\n", objData->faceCount);
	printf("Number of vertex normals: %i\n", objData->normalCount);
	printf("\n");

	//print vertices
	printf("Vertices: %i\n", objData->vertexCount);
	for(int i=0; i<objData->vertexCount; i++)
	{
		printVector(objData->vertexList[i]);
		printf("\n");
	}
	printf("\n");

	//print indices
	printf("Indices: %i\n", objData->faceCount);
	for(int i=0; i<objData->faceCount; i++)
	{
		obj_face *o = objData->faceList[i];
		for(int j=0; j<3; j++)
		{
			printf( "%d", o->vertex_index[j] );
			printf(", ");
			//printVector(objData->vertexList[ o->vertex_index[j] ]);
		}
		printf("\n");
	}

	return 0;
}



