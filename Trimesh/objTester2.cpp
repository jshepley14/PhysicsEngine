// Obj_loader.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include "objLoader.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
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
	printf("\n");

	/*

	//try printing with cout<<
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
		printf("\n");
	}
	printf("};");
	printf("\n");


//////////////////
	int indCount= objData->faceCount;
	int Indices[indCount][3];
	for(int i=0; i<indCount; i++)
	{
		Indices[i][0]=(objData->faceList[i])->vertex_index[0];
		Indices[i][1]=(objData->faceList[i])->vertex_index[1];
		Indices[i][2]=(objData->faceList[i])->vertex_index[2];
	}
//////////////


	for(int i=0; i<indCount; i++)
	{
		printf("%i,",Indices[i][0]);
		printf("%i,",Indices[i][1]);
		printf("%i\n",Indices[i][2]);
	}

	printf("VECTORRRRRR\n");
	int num;
	vector< vector<int> > vec;

	num = indCount;
	for(int i=0; i<num; i++)
	{	vector<int> temp_vec;
		temp_vec.push_back((objData->faceList[i])->vertex_index[0]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[1]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[2]);
		vec.push_back(temp_vec);
		//vec[i][0]=(objData->faceList[i])->vertex_index[0];
		//vec[i][1]=(objData->faceList[i])->vertex_index[1];
		//vec[i][2]=(objData->faceList[i])->vertex_index[2];
	}

	cout<<"vec size:"<< vec.size() <<endl;
	for(int i=0; i<num; i++)
	{
		
		printf("%i,",vec[i][0]);
		printf("%i,",vec[i][1]);
		printf("%i\n",vec[i][2]);
	}
	*/


	int indexCount = objData->faceCount;
	vector< vector<int> > indexVec;
	for(int i=0; i< indexCount; i++)
	{	vector<int> temp_vec;
		temp_vec.push_back((objData->faceList[i])->vertex_index[0]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[1]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[2]);
		indexVec.push_back(temp_vec);
		
	}

	
	int Indices[indexCount][3];
	for(int i=0; i<indexCount; i++)
	{
		Indices[i][0]=indexVec[i][0];
		Indices[i][1]=indexVec[i][1];
		Indices[i][2]=indexVec[i][2];
	}

	for(int i=0; i<indexCount; i++)
	{
		printf("%i,",Indices[i][0]);
		printf("%i,",Indices[i][1]);
		printf("%i\n",Indices[i][2]);
	}

	myfile.close();

	return 0;
}
