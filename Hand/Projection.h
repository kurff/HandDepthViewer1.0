#pragma once
#include "opencv2/opencv.hpp"
#include "model.h"
#include "visualize.h"
#include <vector>
#include <fstream>

using namespace std;
#include <direct.h>
#ifdef _DEBUG
#pragma comment(lib,"opencv_core249d.lib")
#pragma comment(lib,"opencv_highgui249d.lib")
#pragma comment(lib,"opencv_imgproc249d.lib")
#pragma comment(lib,"opencv_contrib249d.lib")
#else 
#pragma comment(lib,"opencv_core249.lib")
#pragma comment(lib,"opencv_highgui249.lib")
#pragma comment(lib,"opencv_imgproc249.lib")
#pragma comment(lib,"opencv_contrib249.lib")
#endif
struct ProjectParam{
	double focal;
	double centerx;
	double centery;
	ProjectParam():focal(241.3), centerx(160), centery(120){}
};

struct Pixel{
	int x;
	int y;
};


int map_label[NUM_JOINT] = {1,2,3,4,0,5,6,7,0,8,9,10,0,11,12,13,0,14,15,16,0,17};


class Projection{
public:
	Projection(int height, int width, int num_joint, int num_pose){
		depth_ = cv::Mat::zeros(height,width,CV_16UC1);
		part_ = cv::Mat::zeros(height,width,CV_8UC1);
		orient_ = cv::Mat::zeros(height,width,CV_32FC1);
		joint_ = cv::Mat::zeros(num_joint,3,CV_32FC1);
		pose_ = cv::Mat::zeros(num_pose,1,CV_32FC1);
		joint2d_ = Eigen::MatrixXd::Zero(num_joint,3);
		fout_.open("joint.txt",ios::out);
		fpara_.open("motion.txt", ios::out);
	}

	~Projection(){
		fout_.close();
	    fpara_.close();
	
	}

	void set_color_index(Model* model){
		coloridx_ = Eigen::MatrixXi::Zero(model->weight_.rows(),1);
		double maxV = 0;
		int index = 0;
		for(int i = 0; i < model->weight_.rows(); i ++){
			maxV = 0.0;
			for(int j = 0; j < model->weight_.cols(); j ++ ){
				if(maxV <= model->weight_(i,j) ){
				   maxV = model->weight_(i,j);
				   index = j;
				}
			}
			coloridx_(i,0) = (index);
			//cout<<index<<" ";
		}

	}
	void projection0(double x3d, double y3d, double z3d, 
		double& x2d, double & y2d, double& depth ){
		depth = -z3d;
		x2d = x3d * param_.focal/ z3d + param_.centerx;
		y2d = -y3d* param_.focal/ z3d + param_.centery;
	}
	void get_joint_position(Model* model){
	    int num_joint = model->get_number_of_joint();
		double* position = nullptr;
		joint_.setTo(0.0);
		for(int i = 0; i < num_joint; i ++ ){
		   position = model->get_joint_position(i);	
		   joint_.at<float>(i,0) = position[0];
		   joint_.at<float>(i,1) = position[1];
		   joint_.at<float>(i,2) = position[2];
		}
	}
	void get_joint_pose(Model* model){
	    int num_joint = model->get_number_of_joint();
        Pose pose(0,0,0);
		pose_.setTo(0.0);
		bool* dof = nullptr;
		int cnt = -1;
		for(int i = 0; i < num_joint ; i ++){
			pose = model->get_pose_of_joint(i);
			dof = model->get_dof(i);
			if(dof[0] == true){
			  cnt ++;
			  pose_.at<float>(cnt,0) = pose.x;
			}
			if(dof[1] == true){
			  cnt ++;
			  pose_.at<float>(cnt,0) = pose.y;
			}
			if(dof[2] == true){
			  cnt ++;
			  pose_.at<float>(cnt,0) = pose.z;
			}
		}

		
		pose = model->get_global_position();
        cnt ++;
		pose_.at<float>(cnt,0) = pose.x;
        cnt ++;
		pose_.at<float>(cnt,0) = pose.y;
		cnt ++;
		pose_.at<float>(cnt,0) = pose.z;
	
	}
	void compute_current_orientation(Model* model){
		int num_joint = model->get_number_of_joint();
		double* position = nullptr;
		double x2d =0,y2d =0, depth =0;
		for(int i = 0; i < num_joint; i ++ ){
		   position = model->get_joint_position(i);	
		   projection0(position[0], position[1], position[2], 
		    x2d, y2d, depth );
		   joint2d_(i,0) = x2d;
		   joint2d_(i,1) = y2d;
		}
		//cout<<joint2d_;
		int idxchild[NUM_JOINT] = {5,2,3,4,0,6,7,8,0,10,11,12,0,
		14,15,16,0,18,19,20,0,22,0};
		for(int i = 0; i < num_joint;i ++){
			if(idxchild[i]!=0){
				x2d = joint2d_(idxchild[i],0) - joint2d_(i,0);
				y2d = joint2d_(idxchild[i],1) - joint2d_(i,1);
	    	    joint2d_(i,2) = atan2(y2d,x2d)*180.0/3.1415926;	
				//cout<<x2d<<" "<<y2d<<" "<<joint2d_(i,2)<<endl;
			}else{
			    joint2d_(i,2) = joint2d_(i-1,2);
				//cout<<i<<endl;
			}

		}

	}

	void projection(double x3d, double y3d, double z3d, 
		double& x2d, double & y2d, double& depth ){
		depth = -z3d;
		x2d = x3d * param_.focal/ z3d + param_.centerx;
		y2d = -y3d* param_.focal/ z3d + param_.centery;

		if(x2d < 0) x2d = 0;
		if(x2d > depth_.cols-1) x2d = depth_.cols-1;
		if(y2d <0) y2d = 0;
		if(y2d > depth_.rows-1) y2d = depth_.rows - 1;
	}

	void select_points_from_one_mesh(Model* model, int idx){
		int f0 = model->faces_(idx,0);
		int f1 = model->faces_(idx,1);
		int f2 = model->faces_(idx,2);
		Pixel p0, p1, p2; 
		p0.x = vertice2d_(f0,0); p0.y = vertice2d_(f0,1);
		p1.x = vertice2d_(f1,0); p1.y = vertice2d_(f1,1);
		p2.x = vertice2d_(f2,0); p2.y = vertice2d_(f2,1);

		int xmin = min(min(p0.x,p1.x),p2.x);
		int xmax = max(max(p0.x,p1.x),p2.x);
		int ymin = min(min(p0.y,p1.y),p2.y);
	    int ymax = max(max(p0.y,p1.y),p2.y);
		Pixel pixel;
		for(int i = ymin; i <= ymax; i ++ ){
			for(int j = xmin; j <= xmax; j ++ ){
				int a = (p1.x-p0.x)*(i-p0.y) - (p1.y-p0.y)*(j-p0.x);
				int b = (p2.x-p1.x)*(i-p1.y) - (p2.y-p1.y)*(j-p1.x);
				int c = (p0.x-p2.x)*(i-p2.y) - (p0.y-p2.y)*(j-p2.x);
			    pixel.x = j; pixel.y = i;
				if(a>=0&&b>=0&&c>=0){pixels_[idx].push_back(pixel);}
				if(a<=0&&b<=0&&c<=0){pixels_[idx].push_back(pixel);}
			}
		}
	}
	

	void choose_point(Model* model){
		int num_faces = model->faces_.rows();
		pixels_.resize(num_faces);
		for(int i = 0; i < num_faces; i ++){
			pixels_[i].clear();  
			//printf("%d\n",i);
			select_points_from_one_mesh(model, i);
		}
	}


	template<typename T>
	void remove_bound_pixels(cv::Mat& img, int pixels){

		for(int i = 0; i < img.rows; ++ i){
			for(int j = 0; j < img.cols; ++ j){
				if( i <= pixels || j <= pixels || i >= img.rows - pixels || j >= img.cols - pixels){
					//if(img.type() == CV_8UC1){
				       img.at<T>(i,j) = 0;					
					//}else{
					//   img.at<ushort>(i,j) = 0;
					//}

				}

			}
		}

	
	}

	void remove_arm(cv::Mat& depth, const cv::Mat& part){
		for(int i = 0; i < depth.rows; ++ i){
		
		}
	
	
	}


	void project_3d_to_2d(Model* model){
		Eigen::MatrixXd vertice = model->vertices_update_;
		int num_vertice = model->vertices_update_.rows();
		vertice2d_ = Eigen::MatrixXd::Zero(num_vertice,3);
		double x2d, y2d, depth;

		depth_.setTo(0);
		part_.setTo(0);
		orient_.setTo(0);

		for(int i = 0; i < model->vertices_update_.rows(); i ++ ){
			projection(model->vertices_update_(i,0),
				model->vertices_update_(i,1), model->vertices_update_(i,2),
				x2d, y2d, depth);
		   vertice2d_(i,0) = x2d;
		   vertice2d_(i,1) = y2d;
		   vertice2d_(i,2) = depth;	
		}
		//cout<<vertice2d_<<endl;
		choose_point(model);
		double x0 = 0, y0 = 0, x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		double alpha0 = 0, alpha1 = 0;
		double depth0 = 0, depth1 = 0, depth2 = 0;
		double d0 = 0, d1 = 0, d2 = 0;

		for(int i = 0; i < pixels_.size(); i ++ ){
			int f0 = model->faces_(i,0);
		    int f1 = model->faces_(i,1);
		    int f2 = model->faces_(i,2);
			x0 = vertice2d_(f1,0) - vertice2d_(f0,0); 
			y0 = vertice2d_(f1,1) - vertice2d_(f0,1);
	        x1 = vertice2d_(f2,0) - vertice2d_(f0,0);
		    y1 = vertice2d_(f2,1) - vertice2d_(f0,1);
			depth0 = vertice2d_(f0,2);
			depth1 = vertice2d_(f1,2);
			depth2 = vertice2d_(f2,2);
			for(int j = 0; j < pixels_[i].size(); j ++ ){
				x2 = pixels_[i][j].x - vertice2d_(f0,0);
				y2 = pixels_[i][j].y - vertice2d_(f0,1);
				d0 = sqrt((x2*x2)+y2*y2);
				double fx1 = pixels_[i][j].x - vertice2d_(f1,0);
				double fy1 = pixels_[i][j].y - vertice2d_(f1,1);
				d1 = sqrt((fx1*fx1)+fy1*fy1);
				double fx2 = pixels_[i][j].x - vertice2d_(f2,0);
				double fy2 = pixels_[i][j].y - vertice2d_(f2,1);
				d2 = sqrt((fx2*fx2)+fy2*fy2);
				int f = 0;
				if(d0<=d1&&d0<=d2){
					f = f0;						   
				}
				if(d1<=d0&&d1<=d2){
					f = f1;
				}
				if(d2<=d0&&d2<=d1){
					f = f2;
				}

				if(x1*y0-x0*y1!=0){
					alpha1 = (x2*y0-y2*x0)/(x1*y0-x0*y1);
					alpha0 = (x2*y1-x1*y2)/(x0*y1-x1*y0);
				}
				else{
					alpha1 = 0;
					if(y0!=0){
				    	alpha0 = (y2)/(y0);					
					}else{alpha0 = (x2)/(x0);}
				}
				if(pixels_[i][j].x>=0 && pixels_[i][j].x< depth_.cols
					&& pixels_[i][j].y >=0 && pixels_[i][j].y < depth_.rows){
					depth = depth0+ alpha0*(depth1-depth0)+alpha1*(depth2-depth0);
				   ushort v= depth_.at<ushort>(pixels_[i][j].y,pixels_[i][j].x);
				   if(v!=0){
					   depth_.at<ushort>(pixels_[i][j].y,pixels_[i][j].x) = min(v, (ushort)depth);
					   if(v>(ushort) depth){
						   part_.at<uchar>(pixels_[i][j].y,pixels_[i][j].x) = (uchar)(map_label[coloridx_(f,0)]);
						   orient_.at<float>(pixels_[i][j].y,pixels_[i][j].x) = joint2d_(coloridx_(f,0),2);
					   }
				   }
				   else{
					   depth_.at<ushort>(pixels_[i][j].y,pixels_[i][j].x) = (ushort)depth;
					   part_.at<uchar>(pixels_[i][j].y,pixels_[i][j].x) = (uchar)(map_label[coloridx_(f,0)]);
					   orient_.at<float>(pixels_[i][j].y,pixels_[i][j].x) = joint2d_(coloridx_(f,0),2);
				   }
				}
			}
		}

		get_joint_position(model);
		get_joint_pose(model);
		//VisualizationTool::imageSC("depth",depth_);		
		//VisualizationTool::imageSC("part",part_);
		//VisualizationTool::imageSC("orient",orient_);



		cv::Mat save_depth;
	    cv::medianBlur(depth_,save_depth,5);
		cv::Mat save_part;
		cv::medianBlur(part_,save_part,5);
		cv::Mat save_orient;
		cv::medianBlur(orient_,save_orient,5);

		// if depth value is larger than 65536, it is error.
		 double minPixelValue, maxPixelValue;
         cv::minMaxIdx(save_depth, &minPixelValue, &maxPixelValue);
		 if(maxPixelValue >= 5000){
		   return;
		 }

		cv::Mat save_orient_descrete = cv::Mat::zeros(save_orient.rows, save_orient.cols,CV_8UC1);
		for(int i = 0; i < save_orient.rows; i ++ ){
			for(int j = 0; j < save_orient.cols; j ++ ){
				if(save_depth.at<ushort>(i,j) != 0)
				save_orient_descrete.at<uchar>(i,j) = uchar((save_orient.at<float>(i,j)+180)/10);
			}
		}


		remove_bound_pixels<ushort>(save_depth, 3);
		remove_bound_pixels<uchar>(save_part, 3);
		//remove_bound_pixels<uchar>(save_orient, 3);

		VisualizationTool::imageSC("depth0",save_depth);		
		VisualizationTool::imageSC("part0",save_part);
		VisualizationTool::imageSC("orient0",save_orient);
		cvWaitKey(1);
		static int nframe = 0;
        char name[200];
		printf("%d\n",nframe);
		Configuration* pconfig = Global();
		if(nframe==pconfig->number_frame){printf("enough");exit(0);}
		if(nframe ==0){ _mkdir("./depth");_mkdir("./part");_mkdir("./orient");}
		sprintf(name,"./depth/%d.png",nframe);
		cv::imwrite(name,save_depth);
		sprintf(name,"./part/%d.png",nframe);
		cv::imwrite(name,save_part);
		sprintf(name,"./orient/%d.png",nframe);
		cv::imwrite(name,save_orient_descrete);



		sprintf(name,"./depth/%d.png",nframe);
		fout_<< name;
		for(int i  = 0; i < joint_.rows; ++ i){
			for(int j = 0; j < joint_.cols; ++ j){
			   fout_<<" "<<joint_.at<float>(i,j);
			}
		}
		fout_<<endl;


		fpara_<<name;
		Pose pose = model->get_global_position();
		fpara_<<" "<<pose.x<<" "<<pose.y<<" "<<pose.z;


		for(int i = 0; i < model->get_number_of_joint(); ++i){
			pose = model->get_pose_of_joint(i);
			bool* dof = model->get_dof(i);
			if(dof[0] == true){
			  fpara_<<" "<<pose.x;
			}
			if(dof[1] == true){
			  fpara_<<" "<< pose.y;
			}
			if(dof[2] == true){
			  fpara_<<" "<<pose.z;			
			}
		}
		fpara_<<endl;




		nframe ++;



	}


private:
	cv::Mat depth_;
	cv::Mat orient_;
	cv::Mat part_;
	cv::Mat joint_;
	cv::Mat pose_;
	ProjectParam param_;
	vector<vector<Pixel> > pixels_;
	Eigen::MatrixXd vertice2d_;
	Eigen::MatrixXi coloridx_;
	Eigen::MatrixXd joint2d_;

	// write file

	ofstream fout_; // write joint file
	ofstream fpara_; // write 27 parameters



};