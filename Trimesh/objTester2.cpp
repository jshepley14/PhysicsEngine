// Obj_loader.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include "objLoader.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#define ROUNDSCALE 0.01

class data // 3 vertices of each triangle
{
public:
    float x1,y1,z1;
    float x2,y2,z2;
    float x3,y3,z3;
};

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
	objData->load("milk_carton.obj");
	printf("\n");


	int numTriangles = objData->faceCount; // pull in the STL file and determine number of triangles
    //data * triangles = new triangles [objData->faceCount];
	data triangles[numTriangles];

	// fill the triangles array with the data in the STL file
	for (int i =0; i < numTriangles; i ++){
		obj_face *o = objData->faceList[i];
		//cout<<o->vertex_index[0]<<"," <<o->vertex_index[1]<<","<< o->vertex_index[2] <<endl;
		triangles[i].x1=objData->vertexList[ o->vertex_index[0] ]->e[0] ;  //(int)(objData->vertexList[ o->vertex_index[0] ]->e[0]/ROUNDSCALE) * ROUNDSCALE ; used for scaling
		triangles[i].y1=objData->vertexList[ o->vertex_index[0] ]->e[1] ;
		triangles[i].z1=objData->vertexList[ o->vertex_index[0] ]->e[2] ;
		triangles[i].x2=objData->vertexList[ o->vertex_index[1] ]->e[0] ;
		triangles[i].y2=objData->vertexList[ o->vertex_index[1] ]->e[1] ;
		triangles[i].z2=objData->vertexList[ o->vertex_index[1] ]->e[2] ;
		triangles[i].x3=objData->vertexList[ o->vertex_index[2] ]->e[0] ;
		triangles[i].y3=objData->vertexList[ o->vertex_index[2] ]->e[1] ;
		triangles[i].z3=objData->vertexList[ o->vertex_index[2] ]->e[2] ;
	}
    
	/*
	//print them
	for (int i =0; i < numTriangles; i ++){
		
		cout<<"Triangle: "<<i<<endl;
		//cout<<triangles[i].x1<<","<<triangles[i].y1<<", "<<triangles[i].z1<<endl;
		//cout<<triangles[i].x2<<","<<triangles[i].y2<<", "<<triangles[i].z2<<endl;
		cout<<triangles[i].x3<<","<<triangles[i].y3<<", "<<triangles[i].z3<<endl;
		cout<<"\n";
		
	} 
*/



    double totalVolume = 0, currentVolume;
    double xCenter = 0, yCenter = 0, zCenter = 0;

    for (int i = 0; i < numTriangles; i++)
    {
        totalVolume += currentVolume = (triangles[i].x1*triangles[i].y2*triangles[i].z3 - triangles[i].x1*triangles[i].y3*triangles[i].z2 - triangles[i].x2*triangles[i].y1*triangles[i].z3 + triangles[i].x2*triangles[i].y3*triangles[i].z1 + triangles[i].x3*triangles[i].y1*triangles[i].z2 - triangles[i].x3*triangles[i].y2*triangles[i].z1) / 6;
        xCenter += ((triangles[i].x1 + triangles[i].x2 + triangles[i].x3) / 4) * currentVolume;
        yCenter += ((triangles[i].y1 + triangles[i].y2 + triangles[i].y3) / 4) * currentVolume;
        zCenter += ((triangles[i].z1 + triangles[i].z2 + triangles[i].z3) / 4) * currentVolume;
    }

    cout << endl << "Total Volume = " << totalVolume << endl;
    cout << endl << "X center = " << xCenter/totalVolume << endl;
    cout << endl << "Y center = " << yCenter/totalVolume << endl;
    cout << endl << "Z center = " << zCenter/totalVolume << endl;






	
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
	*/

/*
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
	


	int SCALE =100;
	int vertCount = objData->vertexCount;
	vector<float> vertVec;
	for(int i=0; i< vertCount ; i++){
		vertVec.push_back(objData->vertexList[i]->e[0]/SCALE);
		vertVec.push_back(objData->vertexList[i]->e[1]/SCALE);
		vertVec.push_back(objData->vertexList[i]->e[2]/SCALE);	
	}



	cout<<"vertVec size:"<< vertVec.size() <<endl;
	for(int i=0; i< vertCount*3 ; i=i+3){
		printf("%f,",vertVec[i]);
		printf("%f,",vertVec[i+1]);
		printf("%f\n",vertVec[i+2]);
	}
	*/
	


	myfile.close();

	return 0;
}
