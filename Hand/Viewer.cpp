#include "Viewer.h"


void Viewer::set_faces(Eigen::MatrixXi faces){
	faces_ = faces;
}

void Viewer::set_vertices(Eigen::MatrixXd vertices){
	vertices_ = vertices;
}

void Viewer::set_weight(Eigen::MatrixXd weight){
    weight_ = weight;
}



