// sampling pose degree
#pragma once
#include "Model.h"
#include <Eigen/Core>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "Random.h"
class Sample{
public:
	Sample(double range){
		srand(time(NULL));
		range_ = range;
	}

	~Sample(){
	
	}

public:
	double range_;
	Random randn_;

public:

	void select_one_for_global(Model* model, Pose pose){

		double r = randn_.NextDouble()-0.5;
		pose.x += r* range_;
		pose.y += r*range_;
		pose.z += r* range_;
		model->set_global_position(pose);	
	}

	void select_one(Model* model){
		Pose upper(0,0,0);
		Pose lower(0,0,0);
		Pose pose(0,0,0);

		int t =0 ;
		int num_joint = model->get_number_of_joint();
	    int idx = rand()%num_joint ; 
		//pose = model->get_pose_of_joint(idx);
		upper = model->get_upper_of_angle(idx);
		lower = model->get_lower_of_angle(idx);
		if(model->get_dof(idx)[0] == true){
			   t = upper.x - lower.x+1;
			   pose.x = (rand()%t) + lower.x;		   
		}
		if(model->get_dof(idx)[1] == true){
		       t = upper.y - lower.y+1;
			   pose.y = (rand()%t) + lower.y;			
		}

		if(model->get_dof(idx)[2] == true){
		       t = upper.z - lower.z+1;
		       pose.z = (rand()%t) + lower.z;			
		}

		model->set_one_rotation(pose,idx);
		//for(int i = 0; i < model->get_number_of_joint(); i ++ ){
		//	upper = model->get_upper_of_angle(i);
		//	lower = model->get_lower_of_angle(i);
		//	pose.x = 0; pose.y = 0;  pose.z = 0;
		//	if(model->get_dof(i)[0] == true){
		//	   t = upper.x - lower.x+1;
		//	   pose.x = (rand()%t) + lower.x;
		//	}

		//	if(model->get_dof(i)[1] == true){
		//    	t = upper.y - lower.y+1;
		//	    pose.y = (rand()%t) + lower.y;			
		//	}

		//	if(model->get_dof(i)[2] == true){
		//    	t = upper.z - lower.z+1;
		//    	pose.z = (rand()%t) + lower.z;			
		//	}

		//	model->set_one_rotation(pose,i);
		//}	
	}




	




};


