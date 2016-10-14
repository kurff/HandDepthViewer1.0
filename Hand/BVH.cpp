
#include <fstream>
#include <string.h>

#include "BVH.h"

BVH::BVH()
{
	motion = NULL;
	Clear();
}

BVH::BVH( const char * bvh_file_name )
{
	motion = NULL;
	Clear();

	Load( bvh_file_name );
}

BVH::~BVH()
{
	Clear();
}

void  BVH::Clear()
{
	unsigned int  i;
	for ( i=0; i<channels.size(); i++ )
		delete  channels[ i ];
	for ( i=0; i<joints.size(); i++ )
		delete  joints[ i ];
	if ( motion != NULL )
		delete  motion;

	is_load_success = false;
	
	file_name = "";
	motion_name = "";
	
	num_channel = 0;
	channels.clear();
	joints.clear();
	joint_index.clear();
	
	num_frame = 0;
	interval = 0.0;
	motion = NULL;
}

void  BVH::Load( const char * bvh_file_name )
{
	#define  BUFFER_LENGTH  1024*4

	ifstream  file;
	char      line[ BUFFER_LENGTH ];
	char *    token;
	char      separater[] = " :,\t";
	vector< Joint * >   joint_stack;
	Joint *   joint = NULL;
	Joint *   new_joint = NULL;
	bool      is_site = false;
	double    x, y ,z;
	unsigned int       i; int j;
	Clear();
	file_name = bvh_file_name;
	const char *  mn_first = bvh_file_name;
	const char *  mn_last = bvh_file_name + strlen( bvh_file_name );
	if ( strrchr( bvh_file_name, '\\' ) != NULL )
		mn_first = strrchr( bvh_file_name, '\\' ) + 1;
	else if ( strrchr( bvh_file_name, '/' ) != NULL )
		mn_first = strrchr( bvh_file_name, '/' ) + 1;
	if ( strrchr( bvh_file_name, '.' ) != NULL )
		mn_last = strrchr( bvh_file_name, '.' );
	if ( mn_last < mn_first )
		mn_last = bvh_file_name + strlen( bvh_file_name );
	motion_name.assign( mn_first, mn_last );
	file.open( bvh_file_name, ios::in );
	if ( file.is_open() == 0 )  return; 

	while ( ! file.eof() )
	{
		if ( file.eof() )  goto bvh_error;

		file.getline( line, BUFFER_LENGTH );
		token = strtok( line, separater );
		if ( token == NULL )  continue;
		if ( strcmp( token, "{" ) == 0 )
		{
			joint_stack.push_back( joint );
			joint = new_joint;
			continue;
		}
		if ( strcmp( token, "}" ) == 0 )
		{
			joint = joint_stack.back();
			joint_stack.pop_back();
			is_site = false;
			continue;
		}

		if ( ( strcmp( token, "ROOT" ) == 0 ) ||
		     ( strcmp( token, "JOINT" ) == 0 ) || ( strcmp( token, "End" ) == 0 ))
		{
			new_joint = new Joint();
			new_joint->index = joints.size();
			new_joint->parent = joint;
			new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
			new_joint->scale = 0.75;

			new_joint->global_position[0] = 0.0;
			new_joint->global_position[1] = 0.0;
			new_joint->global_position[2] = 0.0;

			new_joint->local_position[0] = 0.0;
			new_joint->local_position[1] = 0.0;
			new_joint->local_position[2] = 0.0;

			new_joint->init_global_position[0] = 0.0;
			new_joint->init_global_position[1] = 0.0;
			new_joint->init_global_position[2] = 0.0;

			new_joint->target_position[0] = 0.0;
			new_joint->target_position[1] = 0.0;
			new_joint->target_position[2] = 0.0;


			new_joint->local = Eigen::MatrixXd::Zero(4,4);
			//new_joint->local_inverse = Eigen::MatrixXd::Zero(4,4);
			new_joint->global = Eigen::MatrixXd::Zero(4,4);
			new_joint->trans = Eigen::MatrixXd::Zero(4,4);
			new_joint->rotation = Eigen::MatrixXd::Identity(4,4);

			new_joint->pose.x = 0.0;
			new_joint->pose.y = 0.0;
			new_joint->pose.z = 0.0;

			new_joint->dof[0] = true;
			new_joint->dof[1] = true;
			new_joint->dof[2] = true;

			new_joint->corresponds[0] = -1;
			new_joint->corresponds[1] = -1;
			new_joint->corresponds[2] = -1;

			new_joint->visiable = false;

			new_joint->index_visiable = 0;

			joints.push_back( new_joint );
			if ( joint )
				joint->children.push_back( new_joint );

			token = strtok( NULL, "" );
			while ( *token == ' ' )  token ++;
			new_joint->name = token;

			joint_index[ new_joint->name ] = new_joint;
			continue;
		}
		if ( strcmp( token, "OFFSET" ) == 0 )
		{
			token = strtok( NULL, separater );
			x = token ? atof( token ) : 0.0;
			token = strtok( NULL, separater );
			y = token ? atof( token ) : 0.0;
			token = strtok( NULL, separater );
			z = token ? atof( token ) : 0.0;
			joint->offset[0] = x;
			joint->offset[1] = y;
			joint->offset[2] = z;

			continue;
		}

		if ( strcmp( token, "CHANNELS" ) == 0 )
		{
			token = strtok( NULL, separater );
			joint->channels.resize( token ? atoi( token ) : 0 );
			for ( i=0; i<joint->channels.size(); i++ )
			{
				Channel *  channel = new Channel();
				channel->joint = joint;
				channel->index = channels.size();
				channels.push_back( channel );
				joint->channels[ i ] = channel;
				token = strtok( NULL, separater );
				if ( strcmp( token, "Xrotation" ) == 0 )
					channel->type = X_ROTATION;
				else if ( strcmp( token, "Yrotation" ) == 0 )
					channel->type = Y_ROTATION;
				else if ( strcmp( token, "Zrotation" ) == 0 )
					channel->type = Z_ROTATION;
				else if ( strcmp( token, "Xposition" ) == 0 )
					channel->type = X_POSITION;
				else if ( strcmp( token, "Yposition" ) == 0 )
					channel->type = Y_POSITION;
				else if ( strcmp( token, "Zposition" ) == 0 )
					channel->type = Z_POSITION;
			}
		}
		if ( strcmp( token, "MOTION" ) == 0 )
			break;
	}
	//file.getline( line, BUFFER_LENGTH );
	//token = strtok( line, separater );
	//if ( strcmp( token, "Frames" ) != 0 )  goto bvh_error;
	//token = strtok( NULL, separater );
	//if ( token == NULL )  goto bvh_error;
	//num_frame = atoi( token );

	//file.getline( line, BUFFER_LENGTH );
	//token = strtok( line, ":" );
	//if ( strcmp( token, "Frame Time" ) != 0 )  goto bvh_error;
	//token = strtok( NULL, separater );
	//if ( token == NULL )  goto bvh_error;
	//interval = atof( token );

	//num_channel = channels.size();
	//motion = new double[ num_frame * num_channel ];
	//for (int i=0; i<num_frame; i++ )
	//{
	//	file.getline( line, BUFFER_LENGTH );
	//	token = strtok( line, separater );
	//	for ( j=0; j<num_channel; j++ )
	//	{
	//		if ( token == NULL )
	//			goto bvh_error;
	//		motion[ i*num_channel + j ] = atof( token );
	//		token = strtok( NULL, separater );
	//	}
	//}
	file.close();
	is_load_success = true;

	return;

bvh_error:
	file.close();
}





// End of BVH.cpp
