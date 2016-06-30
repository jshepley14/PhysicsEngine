// Obj_loader.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include "objLoader.h"
#include <string.h>
#include <iostream>
#include <fstream>
using namespace std;

void printVector(obj_vector *v)
{
	printf("	REAL(%.2f), ", v->e[0] );
	printf("REAL(%.2f), ", v->e[1] );
	printf("REAL(%.2f),  ", v->e[2] );
}

int main(int argc, char **argv){


	//used for writing to file
	ofstream myfile;
	myfile.open ("results.txt");

	objLoader *objData = new objLoader();
	objData->load("cube.obj");


	//try printing with cout<<
	printf("\n");
	cout<<"const int VertexCount = "<<objData->vertexCount<<";"<<endl;
	cout<<"const int IndexCount = "<<objData->faceCount<<" * 3;"<<endl;
	printf("\n");
	
	printf("float Vertices[VertexCount * 3] = {\n");
	//print vertices
	for(int i=0; i<objData->vertexCount; i++)
	{
		printVector(objData->vertexList[i]);
		printf("\n");
	}
	printf("};\n");
	printf("\n");

	//print indices
	printf("int Indices[IndexCount / 3][3] = {");
	printf("\n");
	for(int i=0; i<objData->faceCount; i++)
	{
		obj_face *o = objData->faceList[i];
		printf("	{%d,", o->vertex_index[0] );
		printf("%d,", o->vertex_index[1] );
		printf("%d},", o->vertex_index[2] );
		//printVector(objData->vertexList[ o->vertex_index[j] ]);	
		printf("\n");
	}
	printf("};");
	printf("\n");


	myfile.close();

	return 0;
}



