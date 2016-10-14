

#ifndef  _BVH_H_
#define  _BVH_H_


#include<vector>

#include <map>
#include <string>
using namespace  std;

#include <Eigen/Core>
#include <Eigen/LU>


#include "HandStruct.h"

class  BVH
{
  public:
	enum  ChannelEnum
	{
		X_ROTATION, Y_ROTATION, Z_ROTATION,
		X_POSITION, Y_POSITION, Z_POSITION
	};
	struct  Joint;

	struct  Channel
	{

		Joint *              joint;

		ChannelEnum          type;

		int                  index;
	};

	struct  Joint
	{
		string               name;
		int                  index;
		Joint *              parent;
		vector< Joint * >    children;
		double               offset[3];
		vector< Channel * >  channels;

		// local coordiate // 4*4
		Eigen::MatrixXd local;
		// local inverse
		Eigen::MatrixXd local_inverse;
		// global coordinate // 4*4
		Eigen::MatrixXd global;
		// local rotation // 4*4
		Eigen::MatrixXd rotation;
		// transformation to parent: 4*4
		Eigen::MatrixXd trans;
		// trans* rotation
		Eigen::MatrixXd transR;

		// the rotation of x, y, z;
		Pose pose;
		Pose upper;
		Pose lower;

		bool dof[3];
		int corresponds[3]; // the index of parameter that 

		// local position
		double local_position[3];

		// global position
		double global_position[3];

		// initial global position
		double init_global_position[3];

		// target position: used for inverse kinematics
		double target_position[3];

		// scale
		double scale;

		bool visiable;

		int index_visiable;

	};


  private:
	bool                     is_load_success;
	string                   file_name;   
	string                   motion_name; 
	int                      num_channel; 
	vector< Channel * >      channels;    

	vector< Joint * >        joints;      
	map< string, Joint * >   joint_index; 
	int                      num_frame;   
	double                   interval;    
	double *                 motion;


  public:
	BVH();
	BVH( const char * bvh_file_name );
	~BVH();
	void  Clear();
	void  Load( const char * bvh_file_name );

  public:
	bool  IsLoadSuccess() const { return is_load_success; }
	const string &  GetFileName() const { return file_name; }
	const string &  GetMotionName() const { return motion_name; }
	const int       GetNumJoint() const { return  joints.size(); }
	      Joint *   GetJoint( int no ) const { return  joints[no]; }
	const int       GetNumChannel() const { return  channels.size(); }
	const Channel * GetChannel( int no ) const { return  channels[no]; }

	Joint *   GetJoint( const string & j ) const  {
		map< string, Joint * >::const_iterator  i = joint_index.find( j );
		return  ( i != joint_index.end() ) ? (*i).second : NULL; }
	Joint *   GetJoint( const char * j ) const  {
		map< string, Joint * >::const_iterator  i = joint_index.find( j );
		return  ( i != joint_index.end() ) ? (*i).second : NULL; }
	int     GetNumFrame() const { return  num_frame; }
	double  GetInterval() const { return  interval; }
	double  GetMotion( int f, int c ) const { return  motion[ f*num_channel + c ]; }
	void  SetMotion( int f, int c, double v ) { motion[ f*num_channel + c ] = v; }

};



#endif // _BVH_H_
