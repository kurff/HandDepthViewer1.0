#include "Model.h"
#include "MSRA.h"
#include <iostream>
#include "GL/freeglut.h"
#include "Config.h"
Model* model = nullptr;
double target_position[3*NUM_JOINT];
bool visiable[NUM_JOINT];
msra msr;

#include <iostream>
#include "Viewer.h"
 
#pragma comment(lib,"freeglutd.lib")

#include "Sampling.h"

#include "Projection.h"


Config config;
VisData data;
Control control;
Sample sample(30.0);
Projection projection(240,320,23,28);

void menu(int op) {
 
  switch(op) {
  case 'Q':
  case 'q':
    exit(0);
  }
}
 
/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {
 
  switch(key) {
  case 'q':
	  config.show_mesh = true;
	  config.show_point = false;
	  config.show_skeleton = false;
	  break;
  case 'w':
	  config.show_mesh = false;
	  config.show_point = true;
	  config.show_skeleton = true;
	  break;

  case  27:   // ESC
    exit(0);
  }
}
 
/* executed when a regular key is released */
void keyboardUp(unsigned char key, int x, int y) {
 
}
 
/* executed when a special key is pressed */
void keyboardSpecialDown(int k, int x, int y) {
 
}
 
/* executed when a special key is released */
void keyboardSpecialUp(int k, int x, int y) {
 
}
 
/* reshaped window */
void reshape(int width, int height) {
 
  GLfloat fieldOfView = 90.0f;
  glViewport (0, 0, (GLsizei) width, (GLsizei) height);
 
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fieldOfView, (GLfloat) width/(GLfloat) height, 0.1, 500.0);
 
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}
 
/* executed when button 'button' is put into state 'state' at screen position ('x', 'y') */
void mouseClick(int button, int state, int x, int y) {
	control.mouse_click = 1;
	control.x = x;
	control.y = y;
}
 
/* executed when the mouse moves to position ('x', 'y') */
void logo(){
  glRasterPos2i(100, 100);
  glColor3d(0.0, 0.0, 1.0);
  const unsigned char kurff0[] = "kurff";
  glutBitmapString(GLUT_BITMAP_HELVETICA_18, kurff0);
   glRasterPos2i(-100, 100);
  glColor3d(0.0, 1.0, 0.0);
  const unsigned char kurff1[] = "kurff";
  glutBitmapString(GLUT_BITMAP_HELVETICA_18, kurff1);
     glRasterPos2i(100, -100);
  glColor3d(1.0, 0.0, 0.0);
  const unsigned char kurff2[] = "kurff";
  glutBitmapString(GLUT_BITMAP_HELVETICA_18, kurff2);
   glRasterPos2i(-100, -100);
  glColor3d(1.0, 1.0, 0);
  const unsigned char kurff3[] = "kurff";
  glutBitmapString(GLUT_BITMAP_HELVETICA_18, kurff3);
}
 
/* render the scene */
void draw() {
 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  gluPerspective(180, 1.5, -1000, 1000); 
  glLoadIdentity();
  control.gx = model->get_global_position().x;
  control.gy = model->get_global_position().y;
  control.gz = model->get_global_position().z;
  double r = 200;
  double x = r*sin(control.roty)*cos(control.rotx);
  double y = r*sin(control.roty)*sin(control.rotx);
  double z = r*cos(control.roty);
  //cout<< x <<" "<< y <<" " << z<<endl;
  gluLookAt(x+control.gx,y+control.gy,z+control.gz,control.gx,control.gy,control.gz,0.0, 1.0, 0.0);

  logo();
  /* render the scene here */
  //glColor3d(1.0,1.0,1.0);
  if(config.show_point){
	  glPointSize(2);
	  glBegin(GL_POINTS);
      glColor3d(1.0,0.0,0.0);
	  for(int i = 0; i < model->vertices_update_.rows(); i ++ ){		  
		  glVertex3d(model->vertices_update_(i,0),model->vertices_update_(i,1),model->vertices_update_(i,2));
		  //cout<< model->vertices_(i,0)<< " " << model->vertices_(i,1) <<" "<< model->vertices_(i,2)<<endl;
	  }
	  glEnd();
  }

  if(config.show_mesh){
	  if(data.indices == nullptr) return;
	  if(data.vertices == nullptr ) return;
	  glColor3d(0.0,0.0,1.0);
      glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(3, GL_FLOAT, 0, data.vertices);
	  glEnableClientState(GL_COLOR_ARRAY);
	  glColorPointer(3, GL_FLOAT, 0, data.colors);
//glDrawElements(GL_TRIANGLE_STRIP, 12, GL_UNSIGNED_BYTE, indices);
	  glDrawElements(GL_TRIANGLES, 3*data.num_face, GL_UNSIGNED_INT, data.indices);

// deactivate vertex arrays after drawing
      glDisableClientState(GL_VERTEX_ARRAY);
  
  }
  //glEnable(GL_LIGHTING);
  if(config.show_skeleton){
	  for(int i = 0; i < data.joints.rows(); i++ ){
		  glColor3f(1.0,0.0,0.0); 
		  glPushMatrix();
		  glTranslatef(data.joints(i,0), data.joints(i,1), data.joints(i,2));
		  glutSolidSphere(5, 31, 10);
		  glPopMatrix();
		  if(i!=0){
			  glLineWidth(5);
			  glColor3f(0.0,1.0,0); 
			  glBegin(GL_LINES);
			  int ii = data.joints(i,3);
			  glVertex3f(data.joints(ii,0), data.joints(ii,1), data.joints(ii,2));
			  glVertex3f(data.joints(i,0), data.joints(i,1), data.joints(i,2));
			  glEnd();		
		  }
	  }
  }


  
 
  glFlush();
  glutSwapBuffers();
}
 void mouseMotion(int x, int y) {
	control.rotx = (x - control.x)*0.05;
	control.roty = (y - control.y)*0.05;

    //cout<< control.rotx <<" " << control.roty << endl;
	glutPostRedisplay();
}

/* executed when program is idle */
void idle() { 
	//for(int i = 0; i < msr.nframe_; i ++ ){
	//	msr.msra_to_model(target_position,i);	
	//	model->set_target_position(target_position,visiable);
	//	for(int j = 0; j < NUM_JOINT*3; j ++ ){
	//	    std::cout<< target_position[j] << " "; 
	//	}
	//	model->inverse_kinematic();
	//	model->forward_kinematic();
	//	model->compute_mesh();
	//	data.set_vertices(model->vertices_update_);
	//	draw();
	//}
	Pose pose;
	pose.x = 0; pose.y = 0; pose.z = -200;
	//sample.select_one_for_global(model,pose);
	sample.select_one(model);
	model->forward_kinematic();
	model->compute_mesh();
	projection.compute_current_orientation(model);
	projection.project_3d_to_2d(model);
	data.set_vertices(model->vertices_update_);
	data.set_skeleton(model);
	glutPostRedisplay();
}
 
/* initialize OpenGL settings */
void initGL(int width, int height) {
 
  reshape(width, height);
 
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0f);
 
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
}


void main(int argc, char** argv){
    //Model model("HandBase.bvh");
	model = new Model(".\\model\\HandBase.bvh");
    model->init();

	Pose pose(0,0,0);
	pose.x = 0; pose.y = -0; pose.z = -20;
	model->set_one_rotation(pose,21);
	pose.x = 0; pose.y = 0; pose.z = -90;
	model->load_faces(".\\model\\handfaces.txt");
	model->load_vertices(".\\model\\handverts.txt");
	model->load_weight(".\\model\\weight.txt");

	Configuration* pconfig = Global();
	pconfig->LoadConfiguration("configuration.txt");
	//model->set_one_rotation(pose,3);
	 
	pose.x = 0; pose.y = 0; pose.z = -200;
	model->set_global_position(pose);
	model->set_global_position_center(pose);
	model->save_upper_lower_of_angle("parameters_range.txt");
	model->forward_kinematic();
	model->compute_mesh();
	projection.set_color_index(model);
	projection.compute_current_orientation(model);
	projection.project_3d_to_2d(model);

	data.init(model->vertices_.rows(),model->faces_.rows());
	data.set(model->vertices_update_,model->faces_);
	data.set_color(model->weight_);
	data.set_skeleton(model);
	//model->compute_mesh();
	//double target_position[3*NUM_JOINT];
	//memset(target_position,0,sizeof(double)*NUM_JOINT*3);
	//bool visiable[NUM_JOINT];
	//memset(visiable,0,sizeof(bool)*NUM_JOINT);

	//ofstream f;
	//f.open("result.txt",ios::out);
	//for(int i= 0 ; i< model->vertices_.rows(); i ++ ){
	//	for(int j = 0; j < model->vertices_.cols(); j ++ ){
	//		f<<model->vertices_(i,j)<<" ";
	//	}
	//	f<<endl;
	//
	//}
	//f.close();

	//memset(visiable,1,sizeof(bool)*NUM_JOINT);
	//visiable[0] = 0; visiable[NUM_JOINT-1] = 0;
	//msr.read_joint_msra("joint.txt");
	//msr.init();
	//msr.set_visiable(visiable);


	//msra msr;
	//msr.read_joint_msra("joint.txt");
	//msr.init();
	//msr.set_visiable(visiable);
	//for(int i = 0; i < msr.nframe_; i ++ ){
	//	msr.msra_to_model(target_position,i);	
	//	model.set_target_position(target_position,visiable);
	//	for(int j = 0; j < NUM_JOINT*3; j ++ ){
	//	    std::cout<< target_position[j] << " "; 
	//	}
	//	model.inverse_kinematic();
	//}


  glutInit(&argc, argv);
 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(800, 600);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Interactron");
 
  // register glut call backs
  glutKeyboardFunc(keyboardDown);
  glutKeyboardUpFunc(keyboardUp);
  glutSpecialFunc(keyboardSpecialDown);
  glutSpecialUpFunc(keyboardSpecialUp);
  glutMouseFunc(mouseClick);
  glutMotionFunc(mouseMotion);
  glutReshapeFunc(reshape);
  glutDisplayFunc(draw);  
  glutIdleFunc(idle);
  glutIgnoreKeyRepeat(true); // ignore keys held down
 
  // create a sub menu 
  int subMenu = glutCreateMenu(menu);
  glutAddMenuEntry("Do nothing", 0);
  glutAddMenuEntry("Really Quit", 'q');
 
  // create main "right click" menu
  glutCreateMenu(menu);
  glutAddSubMenu("Sub Menu", subMenu);
  glutAddMenuEntry("Quit", 'q');
  glutAttachMenu(GLUT_RIGHT_BUTTON);
 
  initGL(800, 600);

  glutMainLoop();



	//model->save("position.txt");
	//model->save2("init_position.txt");
	//model->save_trans("trans.txt");
	//model->save_local("local.txt");
	//model->save_global("global.txt");
}