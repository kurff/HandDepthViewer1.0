#include <fstream>
#include "HandStruct.h"
using namespace std;

class msra{

public:

	msra():poses_(nullptr), nframe_(400), dim_(21*3){
		index_ = new int [dim_];
		memset(index_,0,sizeof(dim_));

	}

	~msra(){
		delete [] poses_;
		delete [] index_;
	}

	void set_index(int idx_joint_model, int idx_joint_msra){
	   index_[3*idx_joint_msra] = 3* idx_joint_model ;
	   index_[3*idx_joint_msra+1] = 3* idx_joint_model +1;
	   index_[3*idx_joint_msra+2] = 3* idx_joint_model +2;
	}

	void init(){
		int idx_joint[21] = {21,1,2,3,4,5,6,7,8,13,14,15,16,9,10,11,12,
		17,18,19,20};

		for(int i = 0; i < 21; i ++ ){
		    set_index(idx_joint[i],i);
		}

	}
	void read_joint_msra(char* file){
		ifstream f;
		f.open(file,ios::in);
		poses_ = new double [nframe_*dim_];
		for(int i = 0; i < nframe_; i ++ ){
			for(int j = 0; j < dim_; j ++ ){
				f >> poses_[i*dim_+j];
			}
		}
		f.close();
	}

	void msra_to_model(double position[3*NUM_JOINT], int frame){
		for(int i = 0; i < dim_; i ++ ){
		    position[index_[i]] = poses_[frame*dim_ + i];
		}
	}
	void set_visiable(bool visiable[NUM_JOINT]){
	   int idx_joint[21] = {21,1,2,3,4,5,6,7,8,13,14,15,16,9,10,11,12,
		  17,18,19,20};
	   for(int i = 0; i < 21; i ++ ){
	      visiable[idx_joint[i]] = true;
	   }
	}

public:
	double* poses_;
	int nframe_;
	int dim_;
	int* index_;

};
