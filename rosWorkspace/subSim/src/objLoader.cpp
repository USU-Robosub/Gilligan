#include <GL/gl.h>

#include <GL/glu.h>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include "objLoader.h"

struct coordinate {
	float x, y, z;
	coordinate(float a, float b, float c) : x(a), y(b), z(c) {};
};

struct face{
	int vertex[3];
	int normal[3];
	int texture[3];
};

int loadObject(const char* filename)
{
	std::vector<std::string*> coord;        //read every single line of the obj file as a string
	std::vector<coordinate*> vertex;
	std::vector<face*> faces;
	std::vector<coordinate*> normals;       //normal vectors for every face
	std::vector<coordinate*> textures;       //texture vectors for every face
	std::ifstream in(filename);     //open the .obj file
	if(!in.is_open())       //if not opened, exit with -1
	{
		std::cout << "Nor oepened" << std::endl;
		return -1;
	}
	char buf[256];
	//read in every line to coord
	while(!in.eof())
	{
		in.getline(buf,256);
		coord.push_back(new std::string(buf));
	}
	//go through all of the elements of coord, and decide what kind of element is that
	for(unsigned int i=0;i<coord.size();i++)
	{
		if(coord[i]->c_str()[0]=='#')   //if it is a comment (the first character is #)
			continue;       //we don't care about that
		else if(coord[i]->c_str()[0]=='v' && coord[i]->c_str()[1]==' ') //if vector
		{
			float tmpx,tmpy,tmpz;
			sscanf(coord[i]->c_str(),"v %f %f %f",&tmpx,&tmpy,&tmpz);       //read in the 3 float coordinate to tmpx,tmpy,tmpz
			vertex.push_back(new coordinate(tmpx,tmpy,tmpz));       //and then add it to the end of our vertex list
		} else if(coord[i]->c_str()[0]=='v' && coord[i]->c_str()[1]=='n')        //if normal vector
		{
			float tmpx,tmpy,tmpz;   //do the same thing
			sscanf(coord[i]->c_str(),"vn %f %f %f",&tmpx,&tmpy,&tmpz);
			normals.push_back(new coordinate(tmpx,tmpy,tmpz));     
		} else if(coord[i]->c_str()[0]=='v' && coord[i]->c_str()[1]=='t')        //if normal vector
		{
			float tmpx,tmpy,tmpz;   //do the same thing
			sscanf(coord[i]->c_str(),"vt %f %f %f",&tmpx,&tmpy,&tmpz);
			textures.push_back(new coordinate(2*tmpx,2*tmpy,2*tmpz));     
			printf("read texture of (%f, %f, %f)\n", tmpx, tmpy, tmpz);
		} else if(coord[i]->c_str()[0]=='f')     //if face
		{
			face* f = new face;
			sscanf(coord[i]->c_str(),"f %d/%d/%d %d/%d/%d %d/%d/%d",
					&(f->vertex[0]), &(f->texture[0]), &(f->normal[0]),
					&(f->vertex[1]), &(f->texture[1]), &(f->normal[1]),
					&(f->vertex[2]), &(f->texture[2]), &(f->normal[2]));
			faces.push_back(f);     //read in, and add to the end of the face list
		} else {
		}
	}
	//raw
	int num;        //the id for the list
	num=glGenLists(1);      //generate a uniqe
	glNewList(num,GL_COMPILE);      //and create it
	for(unsigned int i=0;i<faces.size();i++)
	{
		glBegin(GL_TRIANGLES);
		glNormal3f(normals[faces[i]->normal[0]-1]->x,
				   normals[faces[i]->normal[0]-1]->y,
				   normals[faces[i]->normal[0]-1]->z);
		glTexCoord2f(textures[faces[i]->texture[0]-1]->x,
				   textures[faces[i]->texture[0]-1]->y);
//				   textures[faces[i]->texture[0]-1]->z);
		glVertex3f(vertex[faces[i]->vertex[0]-1]->x,
				   vertex[faces[i]->vertex[0]-1]->y,
				   vertex[faces[i]->vertex[0]-1]->z);

		glNormal3f(normals[faces[i]->normal[1]-1]->x,
				   normals[faces[i]->normal[1]-1]->y,
				   normals[faces[i]->normal[1]-1]->z);
		glTexCoord2f(textures[faces[i]->texture[1]-1]->x,
				   textures[faces[i]->texture[1]-1]->y);
//				   textures[faces[i]->texture[1]-1]->z);
		glVertex3f(vertex[faces[i]->vertex[1]-1]->x,
				   vertex[faces[i]->vertex[1]-1]->y,
				   vertex[faces[i]->vertex[1]-1]->z);

		glNormal3f(normals[faces[i]->normal[2]-1]->x,
				   normals[faces[i]->normal[2]-1]->y,
				   normals[faces[i]->normal[2]-1]->z);
		glTexCoord2f(textures[faces[i]->texture[2]-1]->x,
				   textures[faces[i]->texture[2]-1]->y);
//				   textures[faces[i]->texture[2]-1]->z);
		glVertex3f(vertex[faces[i]->vertex[2]-1]->x,
				   vertex[faces[i]->vertex[2]-1]->y,
				   vertex[faces[i]->vertex[2]-1]->z);
		glEnd();
	}
	glEndList();
	//delete everything to avoid memory leaks
	for(unsigned int i=0;i<coord.size();i++)
		delete coord[i];
	for(unsigned int i=0;i<faces.size();i++)
		delete faces[i];
	for(unsigned int i=0;i<normals.size();i++)
		delete normals[i];
	for(unsigned int i=0;i<vertex.size();i++)
		delete vertex[i];
	return num;     //return with the id
}
