// Obj_loader.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include "objLoader.h"
#include <string.h>
#include <iostream>
#include <fstream>
using namespace std;
#define SCALE 100  //need to divide by this number or else object is huge
#define SHIFT 1  //want object to be centered at the origin

void printVector(FILE *out, obj_vector *v)
{
	fprintf(out, "	REAL(%.2f), ", v->e[0]/SCALE );
	fprintf(out, "REAL(%.2f), ", v->e[1]/SCALE );
	fprintf(out, "REAL(%.2f),  ", (v->e[2]/SCALE) - SHIFT);
}

int main(int argc, char **argv){


	//used for writing to file
	FILE *out;
	out = fopen("milk_carton2.h", "w");

	objLoader *objData = new objLoader();
	objData->load("milk_carton.obj");


	//try ffprinting with cout<<
	fprintf(out, "const int VertexCount = %i;", objData->vertexCount);
	fprintf(out, "\n");	
	fprintf(out, "const int IndexCount = %i * 3;", objData->faceCount);
	fprintf(out, "\n");
	fprintf(out, "\n");	
	fprintf(out, "float Vertices[VertexCount * 3] = {\n");
	//fprint vertices
	for(int i=0; i<objData->vertexCount; i++)
	{
		printVector(out, objData->vertexList[i]);
		fprintf(out, "\n");
	}
	fprintf(out, "};\n");
	fprintf(out, "\n");

	//fprint indices
	fprintf(out, "int Indices[IndexCount / 3][3] = {");
	fprintf(out, "\n");
	for(int i=0; i<objData->faceCount; i++)
	{
		obj_face *o = objData->faceList[i];
		fprintf(out, "	{%d,", o->vertex_index[0] );
		fprintf(out, "%d,", o->vertex_index[1] );
		fprintf(out, "%d},", o->vertex_index[2] );
		//printVector(objData->vertexList[ o->vertex_index[j] ]);	
		fprintf(out, "\n");
	}
	fprintf(out, "};");
	fprintf(out, "\n");


	fclose(out);

	return 0;
}