#pragma once
#include <Eigen/Dense>
#include "BVH.h"
#include "Model.h"
#include "HandStruct.h"

// the hand model is designed according to the mathematic formulation
// from Microsoft Research paper

// Learning an Efficent Model of Hand Shape Variation from Depth Images
// Sameh Khamis, Jamie Shotton, Andrew Fitzgibbon
// CVPR 2015



#define NUM_JOINT (23)
class Model{
public: 
	Model();
	Model(char* file);
	~Model();
	void set_dof();
	void set_upper_lower_bound();
	void forward_kinematic();
	void inverse_kinematic();
	void compute_local_coordinate();
	void compute_init_global_position();
	void compute_parent_child_transform();
	void compute_local_inverse();

	void set_one_rotation(Pose pose, int index);
	void set_rotation(Pose* pose);
	void set_global_position(Pose global_position);
	void set_global_position_center(Pose global_position);

	void transform_matrix(Pose pose, Eigen::MatrixXd& rot);
	void compute_global();

	void init();

	//// used for inverse kinematic

	void set_one_target_position(double target_position[3], int index);
	void set_target_position(double target_position[NUM_JOINT*3], bool visiable[NUM_JOINT]);


	void load_faces(char* file);
	void load_vertices(char* file);
	void load_weight(char* file);
	void load_weight_0(char* file);
	void compute_mesh();

	void save(char* file);
	void save2(char* file);
	void save_trans(char* file);
	void save_local(char* file);
	void save_global(char* file);

	Pose get_global_position(){return global_position_;}
	Pose get_upper_of_angle(int idx){return bvh_.GetJoint(idx)->upper;}
	Pose get_lower_of_angle(int idx){return bvh_.GetJoint(idx)->lower;}
	void save_upper_lower_of_angle(string file);
	int  get_number_of_joint(){return bvh_.GetNumJoint();};
	int  get_parent_of_joint(int idx){return bvh_.GetJoint(idx)->parent->index;}
	bool* get_dof(int idx){return bvh_.GetJoint(idx)->dof;}
	Pose get_pose_of_joint(int idx){return bvh_.GetJoint(idx)->pose;}
	double* get_joint_position(int idx){return bvh_.GetJoint(idx)->global_position;}
public:

	Eigen::MatrixXi faces_;
	Eigen::MatrixXd vertices_;
	Eigen::MatrixXd weight_;
	Eigen::MatrixXd vertices_update_;


private:
	BVH bvh_;
	Pose global_position_;
	Pose global_position_center_;
	int corresponds_[3];
	int number_visiable_;



	int num_faces_;
	
	//Eigen::MatrixXd vertices0_; // vertices0 is 
	int num_vertices_;
	
	int num_weight_;
};