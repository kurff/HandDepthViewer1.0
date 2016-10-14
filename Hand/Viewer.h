#pragma once
#include <Eigen/Core>

#include <GL/glut.h>

#include "Model.h"

const unsigned char colormap[30][3] = {{255,182,193},
{220,20,60},{0,206,209},{148,0,211},{72,61,139},
{0,0,128},{30,144,255},{72,209,204},{0,255,127},
{0,255,0},{255,255,0},{245,222,179},{210,105,30},
{255,0,0},{46,139,87},{0,255,255},{147,112,219},
{255,0,255},{0,191,255},{95,158,160},{218,112,214},
{64,224,208},{0,100,0},{147,112,219},
{255,0,255},{0,191,255},{95,158,160},{218,112,214},
{64,224,208},{0,100,0}};




struct Config{
	bool show_point;
	bool show_skeleton;
	bool show_vertices;
	bool show_mesh;

	Config():show_point(true),show_skeleton(true),show_vertices(false),show_mesh(false){
	}
};


struct VisData{
	GLfloat *vertices;
    GLuint *indices;
	GLfloat* colors;
	int num_vertice;
	int num_face;

	Eigen::MatrixXd joints;

	VisData():vertices(nullptr),indices(nullptr), colors(nullptr){
		joints = Eigen::MatrixXd::Zero(NUM_JOINT,4);
	}
	~VisData(){
	   if(vertices) delete [] vertices;
	   if(indices) delete [] indices;
	   if(colors) delete [] colors;
	
	}
	void init(int num_vertices, int num_faces){
		this->num_vertice = num_vertices;
		this->num_face = num_faces;
		vertices = new GLfloat [num_vertices*3];
		indices = new GLuint [num_faces*3] ;
		colors = new GLfloat[num_vertices*3];
	}
	void set_color(Eigen::MatrixXd weight){
		double t = -1e10;
		int idx = 0;
		for(int i = 0; i < weight.rows(); i ++ ){
			t = -1e10;
			idx = 0;
			for(int j = 0; j < weight.cols(); j ++ ){
				if(t < weight(i,j)){
				   t = weight(i,j);
				   idx = j;
				}
			}
			colors[3*i] = float(colormap[idx][0])/255.0f;
			colors[3*i+1] = float(colormap[idx][1])/255.0f;
			colors[3*i+1] = float(colormap[idx][2])/255.0f;
		}
	    
	}
	void set_vertices(Eigen::MatrixXd vertices){
		for(int i = 0; i < vertices.rows(); i ++ ){
			this->vertices[3*i] = float(vertices(i,0));
			this->vertices[3*i+1] = float(vertices(i,1));
			this->vertices[3*i+2] = float(vertices(i,2));
		}
	}
	void set(Eigen::MatrixXd vertices, Eigen::MatrixXi faces){
		for(int i = 0; i < faces.rows(); i ++ ){
		   indices[3*i] = faces(i,0);
		   indices[3*i+1] = faces(i,1);
		   indices[3*i+2] = faces(i,2);
		}
		for(int i = 0; i < vertices.rows(); i ++ ){
			this->vertices[3*i] = float(vertices(i,0));
			this->vertices[3*i+1] = float(vertices(i,1));
			this->vertices[3*i+2] = float(vertices(i,2));
		}
	}

	void set_skeleton(Model* model){
	    int num_joint = model->get_number_of_joint();
		double* position = nullptr;
		int idxparent[NUM_JOINT] = {};
		for(int i = 0; i < num_joint; i ++ ){
		   position = model->get_joint_position(i);	
		   joints(i,0) = position[0];
		   joints(i,1) = position[1];
		   joints(i,2) = position[2];
		   if(i==0){
		      joints(i,3) = -1;
		   }else{
			   joints(i,3) = model->get_parent_of_joint(i);
		   }
		}
		//cout<<joints<<endl;
	}


};

struct Control{
	int x;
    int y;
	bool mouse_click;
	GLfloat rotx;
	GLfloat roty;
	double gx;
	double gy;
	double gz;

	Control():x(0),y(0),rotx(0.0),roty(0.0),mouse_click(false),
	gx(0),gy(0),gz(0){
	     
	}
};


