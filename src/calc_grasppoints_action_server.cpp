/*
 * calc_grasppoints_action_server.cpp
 *
 *     ActionServer for calculating grasp points with Height Accumulated Features
 *     (see IJRR 2015 => Link)
 *
 *     Author: David Fischinger, Vienna University of Technology
 *     Date:   April 2015
 *
 *     This ActionServer is calculating grasp points and approach vectors, given an input-goal
 *     (incl. point cloud and grasp search parameters such as area where to search for grasps (center and size),
 *     goal frame_id for point cloud transformation, maximal calculation time, ..).
 *     In a first step a height grid is created from the point cloud
 *     (for rotated and tilted point clouds <==> different rolls of hand and approaching directions).
 *     For each 14x14 square of the height grid a feature vector is created.
 *     Using SVM with an existing model file, it is predicted if the center of the square is a good
 *     grasping point. For the best grasping point the coordinates and the direction of the approach vector
 *     is published.
 *
 *
 *      == Input ==
 *
 *      An action goal including a point cloud of objects (in a local coordinates system (lcs) where z-axis is pointing upwards)
 *
 *      == Output ==
 *
 *      Grasp points and approach vector which are detected using Support Vector Machines
 *      Format: eval, gp1_x, gp1_y, gp1_z, gp2_x, gp2_y, gp2_z, appr_dir_x, appr_dir_y, appr_dir_z roll
 *      	- eval: an evaluation value between 10 and 99. (99 is the top value)
 *      	- gp1_x: x value for the first of the two grasp points
 *      	- gp2_x: x value for the second grasp point (analog for y- and z-axis)
 *      	- appr_dir_x, appr_dir_y: approach direction; direction that has to be used to approach to the object with the gripper
 *      	- roll: the roll orientation of the gripper in degree
 *
 *
 *      == USAGE ==
 *
 *      us the calc_grasppoints_action_client for convenient usage
 *
 *      == PARAMETERS ==
 *
 *      outputpath_full = "/tmp/features.txt";
 *      path_svm_output = "/tmp/output_calc_gp.txt";
 *
 */


//System includes
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <math.h>
#include <time.h>
//tf
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "tf/LinearMath/Transform.h"
//PCL includes
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include "/usr/include/pcl-1.7/pcl/io/io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl_ros/publisher.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//ROS includes
#include <ros/ros.h>
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//Actionlib
#include <actionlib/server/simple_action_server.h>
#include <haf_grasping/CalcGraspPointsServerAction.h>


#include <CIntImage_to_Featurevec.h>

//new HEIGHT and WIDTH have to be next higher even number of the up-rounded square root of (HEIGHT*HEIGHT+WIDTH*WIDTH)
#define HEIGHT 56
#define WIDTH 56
#define PI 3.141592653
#define ROLL_STEPS_DEGREE 15	//define in degree how different the tested rolls should be
#define TILT_STEPS_DEGREE 40	//define in degree the angle for one tilt step
#define TILT_STEPS 1			//define the number of tilt steps

//define maximal degree of rotation of box, should be normally 180+ROLL_STEPS_DEGREE (179,9) because in OR the opposite
//grasp is always tested (=> overall 360 degree). Set this value to ROLL_STEPS_DEGREE if only the first rotation should be checked!
#define ROLL_MAX_DEGREE 190//190


using namespace std;
using namespace cv;

class CCalc_Grasppoints
{
protected:
	// define ROS node
	ros::NodeHandle nh_;
	// define simple action server for grasp calculation
	actionlib::SimpleActionServer<haf_grasping::CalcGraspPointsServerAction> as_;
	std::string action_name_;
	// create messages that are used to publish feedback/result
	haf_grasping::CalcGraspPointsServerFeedback feedback_;
	haf_grasping::CalcGraspPointsServerResult result_;


public:
	ros::Subscriber box_position_sub;	//subscriber for x and y coordinates of the box center and the rotation
	ros::Subscriber pc_sub;				//subscriber for the point cloud (point of objects without basket)
	ros::Publisher pubInputPCPCL;
	ros::Publisher pubInputPCROS;
	ros::Publisher pubTransformedPCROS;	//publisher for transformed point cloud for visualization purpose
	ros::Publisher pubGraspPoints;		//publisher for grasp points
	ros::Publisher pubGraspPointsEval;	//publisher for grasp points with evaluation at the first 2 position (value: 10-99)
	ros::Publisher vis_pub;				//Marker (for visualization of grasps)
	ros::Publisher vis_pub_ma;			//MarkerArray (for visualization of grasps)
	ros::Publisher vis_pub_ma_params;	//MarkerArray (for visualization of grasp parameters: search filed size, gripper closing direction,..)
	float box_center_x;					//box center x-coordinate w.r.t world coordinate system (x-coordinate of center where grasps are searched)
	float box_center_y;					//box center y-coordinate w.r.t world coordinate system (y-coordinate of center where grasps are searched)
	float boxrot_angle_init;			//angle the box is rotated in real world w.r.t. "default" position
	bool box_position_set;
	int grasp_search_area_size_x_dir;	// defines the size (x-direction) of the rectangle where grasps are searched
	int grasp_search_area_size_y_dir;   // defines the size (y-direction) of the rectangle where grasps are searched
	float max_duration_for_grasp_calc;	//maximal time (in seconds) before a result is returned
 	float heightsgridroll[ROLL_MAX_DEGREE/ROLL_STEPS_DEGREE][TILT_STEPS][HEIGHT][WIDTH];
 	float integralimageroll[ROLL_MAX_DEGREE/ROLL_STEPS_DEGREE][TILT_STEPS][HEIGHT+1][WIDTH+1];
	bool point_inside_box_grid[ROLL_MAX_DEGREE/ROLL_STEPS_DEGREE][TILT_STEPS][HEIGHT][WIDTH];	//saves for which points featurevector is calculated
 	string outputpath_full;
 	bool return_only_best_gp;
 	int graspval_th; 					//treshhold if grasp hypothesis should be returned
 	int graspval_top;					//optimal grasp evaluation possible
 	int graspval_max_diff_for_pub;
	//define x,y,and roll for top grasp for overall grasping (all tilts, all rows)
	int id_row_top_overall;
	int id_col_top_overall;
	int nr_roll_top_overall;
	int nr_tilt_top_overall;
	int topval_gp_overall;
	int marker_cnt;
	string gp_result; //saves return values for ActionServer (bad programming style)
	tf::TransformListener tf_listener;
	bool visualization;
	bool print_heights_bool; //indicates if grid heights should be printed on screen

 	//void set_box_position_cb(std_msgs::String box_position_str);
 	void print_heights(int nr_roll, int nr_tilt);
 	//void read_pc_cb(const sensor_msgs::PointCloud2ConstPtr& pc_in); //before action server was used
 	void read_pc_cb(const haf_grasping::CalcGraspPointsServerGoalConstPtr &goal); // receive point cloud as action goal

 	void loop_control(pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in);
 	void generate_grid(int roll, int tilt, pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in);
 	void publish_transformed_pcl_cloud(int roll, int tilt, pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in);
 	void calc_intimage(int roll, int tilt);
 	void calc_featurevectors(int roll, int tilt);
	void pnt_in_box(int nr_roll, int nr_tilt);
 	void predict_bestgp_withsvm(bool svm_with_probability=false);
 	void show_predicted_gps(int nr_roll, int tilt, bool svm_with_probability=false);
 	void transform_gp_in_wcs_and_publish(int id_row_top_all, int id_col_top_all,int nr_roll_top_all, int nr_tilt_top_all, int scaled_gp_eval);
	void publish_grasp_grid(int nr_roll, int tilt, float graspsgrid[][WIDTH], int gripperwidth);

	void gp_to_marker(visualization_msgs::Marker *marker, float x, float y, float z, float green, bool pubmarker, int gripperwidth, int nr_roll, bool pub_grip_open_dir);
	void grasp_area_to_marker(visualization_msgs::Marker *marker, float x, float y, float z, int gripperwidth, int nr_roll, int param_id, bool top_grasp);

	CCalc_Grasppoints(string name) :
	    as_(nh_, name, boost::bind(&CCalc_Grasppoints::read_pc_cb/*executeCB*/, this, _1), false),
	    action_name_(name)
	{
		//this->pc_sub = nh_.subscribe("/SS/points2_object_in_rcs",1, &CCalc_Grasppoints::read_pc_cb, this);	//callback for reading point cloud of box content
		//this->pubGraspPoints = nh_.advertise<std_msgs::String>("/SVM/grasp_hypothesis", 1);
		this->pubGraspPointsEval = nh_.advertise<std_msgs::String>("/haf_grasping/grasp_hypothesis_with_eval", 1);
		this->vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );					//Marker
		this->vis_pub_ma = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1 );	//MarkerArray
		this->vis_pub_ma_params = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array_grasp_params", 1 );	//MarkerArray for grasp params
		this->pubInputPCROS = nh_.advertise<sensor_msgs::PointCloud2>( "/haf_grasping/calc_gp_as_inputpcROS", 1);
		this->pubTransformedPCROS = nh_.advertise<sensor_msgs::PointCloud2>( "/haf_grasping/transformed_point_cloud",1);
		this->pubInputPCPCL = nh_.advertise<sensor_msgs::PointCloud2>( "/calc_gp_as_inputpcPCL", 1);
		box_position_set = true;
		this->box_center_x = 0.07;
		this->box_center_y = -0.49;
		this->grasp_search_area_size_x_dir = 32;
		this->grasp_search_area_size_y_dir = 44;
		this->max_duration_for_grasp_calc = 50; 	//max calculation time before restult is returned (in sec)
		outputpath_full = "/tmp/features.txt";
		return_only_best_gp = true;
		graspval_th = 70;					//treshold if grasp hypothesis should be returned (in function - so program internal) (for top result of one loop run)
		graspval_top = 119;
		graspval_max_diff_for_pub = 80;		//if the value of grasps is more than graspval_max_diff_for_pub lower than optimal value graspval_top, nothing gets published (for top result of one whole roll run)
		id_row_top_overall = -1;
		id_col_top_overall = -1;
		nr_roll_top_overall = -1;
		nr_tilt_top_overall = -1;
		topval_gp_overall = -1000;
		marker_cnt = 0;
		this->gp_result = "";
		this->visualization = true;
		this->print_heights_bool = false;

	    as_.start();
	}
};



// Print heights of heightsgridroll
void CCalc_Grasppoints::print_heights(int nr_roll, int nr_tilt)
{
	//print heights matrix
	cout << "\n CCalc_Grasppoints::print_heights: print heights matrix for roll number : " << nr_roll << "\t and tilt nr: "<< nr_tilt << endl;
	for (int i = 0; i < HEIGHT; i++){  //rows
		for (int j = 0; j < WIDTH; j++){  //cols
			cout << setw(5) << setprecision(2) << this->heightsgridroll[nr_roll][nr_tilt][HEIGHT-i-1][WIDTH-j-1];
		}
		cout << "\n";
	}
}



// Input: action goal including point clout, center position where to grasp (x-,y-coordinates)
// Variable are set from the goal and the loop for GP calculation is started
void CCalc_Grasppoints::read_pc_cb(const haf_grasping::CalcGraspPointsServerGoalConstPtr &goal)
{
	ROS_INFO("\nCCalc_Grasppoints::read_pc_cb --> From calc_grasppoints_action_server: grasp goal (incl. point cloud) was received");
	//transform point cloud to PCL
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in;
	pcl::PointCloud<pcl::PointXYZ> pc_new_cs;


	// set center of area where grasps are searched
	// missing: error handling
	this->box_center_x = goal->graspinput.grasp_area_center.x;
	this->box_center_y = goal->graspinput.grasp_area_center.y;
	cout << "SET NEW GRASP SEARCH CENTER: box_center_x: " << this->box_center_x << "  box_center_y: " << this->box_center_y << "\n";

	// define size of area where to search for grasp points
	// TODO  missing error handling and setting of min and max limits
	this->grasp_search_area_size_x_dir = goal->graspinput.grasp_area_length_x;
	this->grasp_search_area_size_y_dir = goal->graspinput.grasp_area_length_y;

	// set maximal calculation time
	this->max_duration_for_grasp_calc = (float) goal->graspinput.max_calculation_time.toSec();
	cout << "CCalc_Grasppoints::read_pc_cb --> max. calculation time was set to: " <<  this->max_duration_for_grasp_calc << "\n";

	//search for tf transform between camera ("frame") and robot ("base_link") camera coordinate system
	ros::Time now = ros::Time::now();
	bool foundTransform = tf_listener.waitForTransform(/*req.frame_id_desired.data.c_str()*/"/base_link", /*req.frame_id_original.data.c_str()*/"frame", now, ros::Duration(1.0));
	if (!foundTransform)
	{
		ROS_WARN("CCalc_Grasppoints::read_pc_cb: No transform for point cloud found");
	}

	//ROS_INFO(tf_listener);
	ROS_INFO("CCalc_Grasppoints::read_pc_cb: Transform for point cloud found");

	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in_old_cs;
	pcl::fromROSMsg(goal->graspinput.input_pc, pcl_cloud_in_old_cs); // transform ROS msg into PCL-pointcloud

	pcl_ros::transformPointCloud(/*req.frame_id_desired.data.c_str()*/"/base_link", pcl_cloud_in_old_cs, pc_new_cs, tf_listener);
	pcl_cloud_in = pc_new_cs;



	//publish input pc:
	this->pubInputPCROS.publish(goal->graspinput.input_pc);

	//set initial values for row,col,tilt,topval for top grasp points
	id_row_top_overall = -1;
	id_col_top_overall = -1;
	nr_roll_top_overall = -1;
	nr_tilt_top_overall = -1;
	topval_gp_overall = -1000;

	loop_control(pcl_cloud_in);
}



//controls the execution of class methods
//loop goes through all rolls and tilts and executes necessary methods for calculation of gps
void CCalc_Grasppoints::loop_control(pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in)
{

	int time_for_calc_in_secs = this->max_duration_for_grasp_calc;
	time_t start,end;
	time (&start);
	double timedif;
	bool success = true;

	for (int tilt = 0; tilt < TILT_STEPS; tilt++)			// tilt loop
	{
		for (int roll = 0; roll < ROLL_MAX_DEGREE/ROLL_STEPS_DEGREE; roll++)	// roll loop
		{
		    // feedback:		    (3.12.2014)
		    feedback_.feedback.data = "this->topval_gp_overall";	//shows best gp so far

		    if (as_.isPreemptRequested() || !ros::ok() || success == false)
		         {
		           ROS_INFO("%s: Preempted", action_name_.c_str());
		           // set the action state to preempted
		           as_.setPreempted();
		           success = false;
		           break;
		         }

		    as_.publishFeedback(feedback_);


			if (this->return_only_best_gp and (this->topval_gp_overall >= this->graspval_top)){
				cout << "Top grasp already found\n";
				break;
			}

			//calculate only for defined time
			time (&end);
			timedif = difftime (end,start);
			cout << "\n cur timediff: " << timedif << endl;
			if (timedif > time_for_calc_in_secs) {
				cout << "\n time is over, stop calculating of grasp points!!\n";
				break;
			}

			generate_grid(roll, tilt, pcl_cloud_in);
			if (this->print_heights_bool){
				print_heights(roll,tilt);
			}
			calc_intimage(roll, tilt);
			calc_featurevectors(roll, tilt);
			//ii_to_fv->print_heights(ii_to_fv->intimagemat);
			bool svm_with_probability = false;
			predict_bestgp_withsvm(svm_with_probability);
			show_predicted_gps(roll, tilt, svm_with_probability);
		}
	}
	//publish original point cloud (to see where grasp was found)
	//publish_transformed_pcl_cloud(this->nr_roll_top_overall, this->nr_tilt_top_overall, pcl_cloud_in);	//publish point cloud in orientation for best found grasp
	publish_transformed_pcl_cloud(0, 0, pcl_cloud_in);	//publish point cloud in orientation for best found grasp
	transform_gp_in_wcs_and_publish(this->id_row_top_overall, this->id_col_top_overall, this->nr_roll_top_overall,this->nr_tilt_top_overall, this->topval_gp_overall-20); //last entry is "scaled" (=> -16)

	time (&end);
	timedif = difftime (end,start);
	cout << "\n Gesamtzeit fuer Loop: " << timedif << endl;

    if(success)	// ActionServer: return grasp representation (overall best grasp)
    {
      result_.result.data = this->gp_result; //ss.str();
      ROS_INFO("%s: Succeeded", this->gp_result.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
}


// transforms point cloud and generates height grid
void CCalc_Grasppoints::generate_grid(int roll, int tilt, pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_transformed;
	int nr_rows = HEIGHT, nr_cols = WIDTH;
	float r_col_m = (0.5 * (float)nr_cols)/100.0;	//"Matrix radius" in meter for cols
	float r_row_m = (0.5 * (float)nr_rows)/100.0;	//"Matrix radius" in meter for rows
	pcl::PointXYZ pnt;  		//point for loop

	Eigen::Matrix4f mat_sh_orig = Eigen::Matrix4f::Identity(); //matrix which defines shift of point cloud from center to origin
	Eigen::Matrix4f mat_rot = Eigen::Matrix4f::Identity(); //matrix which rotates pc and then shifts pc back to box center
	Eigen::Matrix4f mat_tilt = Eigen::Matrix4f::Identity(); //matrix which tilts pc  NEW
	Eigen::Matrix4f mat_sh_from_orig = Eigen::Matrix4f::Identity(); //matrix which defines shift of point cloud from origin to basket center
	Eigen::Matrix4f mat_transform = Eigen::Matrix4f::Identity(); //matrix for transformation of pointcloud (rotation about box center)

	mat_sh_orig(0,3) = -this->box_center_x;  //shift x value
	mat_sh_orig(1,3) = -this->box_center_y;	 //shift y-value
	mat_sh_from_orig(0,3) = this->box_center_x;
	mat_sh_from_orig(1,3) = this->box_center_y;



	//define matrices s.t. transformation matrix can be calculated for point cloud (=> roll and tilt are simulated)

	//next 5 lines could be outside tilt-loop
	float angle = roll*ROLL_STEPS_DEGREE*PI/180 + this->boxrot_angle_init;	//angle for roll
	mat_rot(0,0) = cos(angle);
	mat_rot(0,1) = -sin(angle);
	mat_rot(1,0) = sin(angle);
	mat_rot(1,1) = cos(angle);

	float beta = -tilt*TILT_STEPS_DEGREE*PI/180; //angle for tilt in Rad
	mat_tilt(0,0) = cos(beta);
	mat_tilt(0,2) = -sin(beta);
	mat_tilt(2,0) = sin(beta);
	mat_tilt(2,2) = cos(beta);

	mat_transform = mat_sh_from_orig*mat_tilt*mat_rot*mat_sh_orig;  //define transformation matrix mat_transform

	pcl::copyPointCloud(pcl_cloud_in, pcl_cloud_transformed);	// copy notwendig?? pointer/nicht pointerzeug richtig???
	pcl::transformPointCloud(pcl_cloud_in, pcl_cloud_transformed, mat_transform);  //transform original point cloud


	//new: 8.4.2015: publish point cloud
	//publish transformed point cloud:
	if (this->visualization)
	{
		cout << "not publish transformed point cloud! \n";
		//this->pubTransformedPCROS.publish(pcl_cloud_transformed); df 15.4.2015
	}

	//set heightsgridroll to -1.0 at each position
	for (int i = 0; i < nr_rows; i++)
		for (int j = 0; j < nr_cols; j++)
			this->heightsgridroll[roll][tilt][i][j]= -1.0;	//david new: achtung, nach tilt sind werte <0 auch moeglich

	//make heightsgrid (find highest points for each 1x1cm rectangle); translate grid s.t. it starts with (0,0)
	for (size_t i = 0; i < pcl_cloud_transformed.points.size(); ++i)
	{
		//ASSUMING point cloud is centered w.r.t. (this->box_center_x,this->box_center_y)
		int idx_x = -1, idx_y = -1;
		pnt = pcl_cloud_transformed.points[i];

		if ((pnt.x > this->box_center_x-r_row_m) and (pnt.x < this->box_center_x+r_row_m) and
			(pnt.y > this->box_center_y-r_col_m) and (pnt.y < this->box_center_y+r_col_m))
		{ //point is relevant for object grid
			idx_x = (int) (floor (100*(pnt.x - (this->box_center_x - r_row_m))));
			idx_y = (int) (floor (100*(pnt.y - (this->box_center_y - r_col_m))));;
			if (heightsgridroll[roll][tilt][idx_x][idx_y] < pnt.z)
			{
				heightsgridroll[roll][tilt][idx_x][idx_y] = pnt.z;
			}
		}
	}

	//set heightsgridroll to 0 at each position where no points were detected (<=> entry still -1)
	//and adjust heightsgrid accordingly to tilt_angle (although orientation of plans get skewed)
	// NOT USED YET!!!
	if (tilt > 110){
		for (int i = 0; i < nr_rows; i++){
			for (int j = 0; j < nr_cols; j++){
				if (this->heightsgridroll[roll][tilt][i][j] < -0.99){
					this->heightsgridroll[roll][tilt][i][j] = 0; //set values where no points are, to zero
				} else {
					this->heightsgridroll[roll][tilt][i][j] -= (i+1-nr_rows/2)*tan(beta)*0.01;
					if (j == 28)
						cout << "\n i: " << i << "\t res: " << (i+1-nr_rows/2)*tan(beta)*0.01;
				}
			}
		}
	}

	for (int i = 0; i < nr_rows; i++){
		for (int j = 0; j < nr_cols; j++){
			if (this->heightsgridroll[roll][tilt][i][j] < -0.99){
				this->heightsgridroll[roll][tilt][i][j] = 0; //set values where no points are, to zero
			}
		}
	}
	//david: fehlt noch: abgleichung der höhen falls getiltet wurde
}


void CCalc_Grasppoints::publish_transformed_pcl_cloud(int roll, int tilt, pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_transformed;

	Eigen::Matrix4f mat_sh_orig = Eigen::Matrix4f::Identity(); //matrix which defines shift of point cloud from center to origin
	Eigen::Matrix4f mat_rot = Eigen::Matrix4f::Identity(); //matrix which rotates pc and then shifts pc back to box center
	Eigen::Matrix4f mat_tilt = Eigen::Matrix4f::Identity(); //matrix which tilts pc  NEW
	Eigen::Matrix4f mat_sh_from_orig = Eigen::Matrix4f::Identity(); //matrix which defines shift of point cloud from origin to basket center
	Eigen::Matrix4f mat_transform = Eigen::Matrix4f::Identity(); //matrix for transformation of pointcloud (rotation about box center)

	mat_sh_orig(0,3) = -this->box_center_x;  //shift x value
	mat_sh_orig(1,3) = -this->box_center_y;	 //shift y-value
	mat_sh_from_orig(0,3) = this->box_center_x;
	mat_sh_from_orig(1,3) = this->box_center_y;

	//define matrices s.t. transformation matrix can be calculated for point cloud (=> roll and tilt are simulated)

	//next 5 lines could be outside tilt-loop
	float angle = roll*ROLL_STEPS_DEGREE*PI/180 + this->boxrot_angle_init;	//angle for roll
	mat_rot(0,0) = cos(angle);
	mat_rot(0,1) = -sin(angle);
	mat_rot(1,0) = sin(angle);
	mat_rot(1,1) = cos(angle);

	float beta = -tilt*TILT_STEPS_DEGREE*PI/180; //angle for tilt in Rad
	mat_tilt(0,0) = cos(beta);
	mat_tilt(0,2) = -sin(beta);
	mat_tilt(2,0) = sin(beta);
	mat_tilt(2,2) = cos(beta);

	mat_transform = mat_sh_from_orig*mat_tilt*mat_rot*mat_sh_orig;  //define transformation matrix mat_transform

	pcl::copyPointCloud(pcl_cloud_in, pcl_cloud_transformed);	// copy notwendig??
	pcl::transformPointCloud(pcl_cloud_in, pcl_cloud_transformed, mat_transform);  //transform original point cloud


	//publish transformed point cloud:
	this->pubTransformedPCROS.publish(pcl_cloud_transformed);

}




//calculates integral image of heights grid
void CCalc_Grasppoints::calc_intimage(int roll, int tilt)
{
    Mat heightsIntegral;
    heightsIntegral = Mat(HEIGHT+1, WIDTH+1, CV_64FC1);
	//cout << "\n print integral height matrix on screen\n";

    double *TempHeights = (double *)malloc(HEIGHT * WIDTH * sizeof(double));

    int k = 0;
    for(int i = 0; i < HEIGHT; i++ )
      	for(int j = 0; j < WIDTH; j++ )
    	{
    		TempHeights[k] = this->heightsgridroll[roll][tilt][i][j];
    		k++;
    	}

    Mat const cvheightsmat = Mat(HEIGHT, WIDTH, CV_64FC1, TempHeights);
	//Mat const cvheightsmat = Mat(HEIGHT, WIDTH, CV_64FC1, this->heightsgridroll[roll][tilt]);
	integral(cvheightsmat, heightsIntegral,CV_64FC1);


	//copy integral image from opencv format to normal matrix format (better implementation would be fine)
	for (int row = 0; row < 1+HEIGHT; row++)  //rows
		for (int col = 0; col < 1+WIDTH; col++)  //cols
			this->integralimageroll[roll][tilt][row][col] = ((double*)(heightsIntegral.ptr() + heightsIntegral.step*row))[col];

	//print integral matrix
	/*cout << "\n print integral height matrix on screen (intuitiv way)\n";
	for (int row = 0; row < 1+HEIGHT; row++){  //rows
		cout << endl;
		for (int col = 0; col < 1+WIDTH; col++){  //cols
			cout << this->integralimageroll[roll][tilt][HEIGHT-row][WIDTH-col] << "\t";
		}
	}*/

	free(TempHeights);
}


void CCalc_Grasppoints::calc_featurevectors(int roll, int tilt)
{
	cout << "\n calc_featurevectors \n";

	//create object for calculating features
	CIntImage_to_Featurevec * ii_to_fv = new CIntImage_to_Featurevec();

	string pkg_path = ros::package::getPath("haf_grasping");
	//read all features (saved in external file)
	ii_to_fv->read_features(pkg_path);

	//silly way to delete file
	ofstream output_fv_file(outputpath_full.c_str());


	pnt_in_box(roll, tilt);	//calculate if points are inside box, saved in this->point_inside_box_grid[roll][nr_tilt][][]

	for (int row = 0; row < HEIGHT - 14; row++){
		for (int col = 0; col < WIDTH -14; col++)
		{
			//check if grasping point is possible (inside box) ("inside box detection code")
			if (!this->point_inside_box_grid[roll][tilt][row+7][col+7]){   //point not inside box (not relevant)
				continue;	//do not generate feature vector for this point
			}
			//if (row == 10 and col == 10) cout << "\n one 15x15 intimage: " << ++cnt << "\n";
			//write integral image (15x15!)  => better implementation will come later
			for (int i = 0; i < 15; i++){
				for (int j = 0; j < 15; j++){
					ii_to_fv->intimagemat[i][j] = this->integralimageroll[roll][tilt][row+i][col+j];
				}
			}

			//calculate featurevector and write it in file
			ii_to_fv->write_featurevector(outputpath_full.c_str());
		}
	}
}




// (short: calculates if for a position, the feature vector should be calculated)
//calculates if for a given row and column and a specified number of roll,
//the indicated point (center of 14x14 square) is inside the real box
//AND (new since 6.2.2012): if the point is surrounded by 14x14 integral square!! => feature calculation is possible
// !!!!!!!!!!!!!! at the moment nr_tilt is not used for more exact calculation !!!!!!!!!!
void CCalc_Grasppoints::pnt_in_box(int nr_roll, int nr_tilt){

	bool print_pnt_in_box = false;
    //function:
    //   ( cx )               ( cos(alpha) )
    //g: (    ) +  lambda *   (            ) = r
    //   ( cy )               ( sin(alpha) )

    //function parallel to center line:
    //   ( cx )       (-sin(alpha)                           ( cos(alpha) )
    //g: (      ) +/- (          )*DIMBOX/2    +    lambda * (            ) = r
    //   ( cy )       ( cos(alpha)                           ( sin(alpha) )

    float alpha_deg = -nr_roll*ROLL_STEPS_DEGREE - this->boxrot_angle_init * 180/PI;//angle for rotation in degree
    float alpha = alpha_deg*PI/180;            	//angle for rotation in rad
    float cx = HEIGHT/2;                        //x-coordinate for rotation center   works because  !! HEIGHT and WIDTH are equal!!
    float cy = HEIGHT/2;                         //y-coordinate for rotation center  works because  !! HEIGHT and WIDTH are equal!!
    float cx1,cy1,cx2,cy2,cx3,cy3,cx4,cy4;    //coordinates of points which are used to define points inside the box (points at box edges)

    //define points at the center of each of the 4 boarder edges of box
    float boarder = 7.0;  						//ungraspable inside in box (because hand would touch box)
    float height_r = this->grasp_search_area_size_x_dir/2 - boarder; 	//defines the half of the length of one side of the graspable inside of box (defines rectangle in box rectangle where grasping is possible)
    float width_r = this->grasp_search_area_size_y_dir/2 - boarder;		//width is equal to y direction!
    cx1 = cx - sin(alpha)*height_r;
    cy1 = cy + cos(alpha)*height_r;
    cx2 = cx + sin(alpha)*height_r;
    cy2 = cy - cos(alpha)*height_r;
    cx3 = cx - sin(alpha+PI/2)*width_r;
    cy3 = cy + cos(alpha+PI/2)*width_r;
    cx4 = cx + sin(alpha+PI/2)*width_r;
    cy4 = cy - cos(alpha+PI/2)*width_r;

    if (print_pnt_in_box)
    	cout << cx1 << "\t" << cy1 << "\n" << cx2 << "\t" << cy2 << "\n" << cx3 << "\t" << cy3 << "\n" << cx4 << "\t" << cy4 << "\n\n";
    int cnt_all = 0;
    int cnt_in = 0;
    int cnt_out = 0;
    //i = row, j = col
    for (int i = 0; i < HEIGHT; i++){
        for (int j = 0; j < HEIGHT; j++){	//(2 times HEIGHT works because HEIGHT and WIDTH are equal!!)
        	++cnt_all;

        	// parameters to check if calculation of feature vector is needed (if no object points are around a potential grasp position, no calculation necessary)
        	int th_empty_r = 4;			//r..radius;if 4 => 8x8cm field is checked if integral image has at least difference of ii_th_in_r
        	float ii_th_in_r = 0.03;	//threshold which must be smaller as integral-difference value if featurevec should be calculated

        	// check if point (i,j) is "inside the box" AND not near the overall boarder (of 56x56cm grid)
            if ((i>6 and i < HEIGHT-7 and j>6 and j<HEIGHT-7)           and	   // take only points which are not in the 7cm boarder of the 56x56 grid (otherwise no featurecalc possible)
           		(this->integralimageroll[nr_roll][nr_tilt][i+th_empty_r][j+th_empty_r]-		//check if there are heights (if every height about 0, no calc. of fv needed)
 	        	 this->integralimageroll[nr_roll][nr_tilt][i-th_empty_r-1][j+th_empty_r]-
            	 this->integralimageroll[nr_roll][nr_tilt][i+th_empty_r][j-th_empty_r-1]+
            	 this->integralimageroll[nr_roll][nr_tilt][i-th_empty_r-1][j-th_empty_r-1] > ii_th_in_r) and
            	(-sin(alpha)*(-cx1+j) + cos(alpha)*(-cy1+i) <  0.00001) and    //0.00001 and not 0 because of rounding errors otherwise
                (-sin(alpha)*(-cx2+j) + cos(alpha)*(-cy2+i) > -0.00001) and
                ( cos(alpha)*(-cx3+j) + sin(alpha)*(-cy3+i) > -0.00001) and
                ( cos(alpha)*(-cx4+j) + sin(alpha)*(-cy4+i) <  0.00001)){
                //if (print_pnt_in_box)
                //	cout << " + ";    //point inside box AND (new 6.2.2012) not in boarder of 56x56 grid
                this->point_inside_box_grid[nr_roll][nr_tilt][i][j] = true;
                cnt_in++;
            } else {
                //if (print_pnt_in_box)
                //	cout << " - ";    //point not inside box (or near 56x56 boarder => no 14x14 integral grid available => no calc of fv possible)
                this->point_inside_box_grid[nr_roll][nr_tilt][i][j] = false;
                cnt_out++;
            }
        }
    }

    //print pnt_in_box if variable print_pnt_in_box is set
    if (print_pnt_in_box){
    	cout << "points_all: " << cnt_all << "  points in: " << cnt_in << "  and points out: " << cnt_out << endl;
    	for (int i = HEIGHT-1; i >= 0; i--){
    		for (int j = HEIGHT-1; j >= 0; j--){	//(2 times HEIGHT works because HEIGHT and WIDTH are equal!!)
    			if (this->point_inside_box_grid[nr_roll][nr_tilt][i][j] == true){
    				cout << " + ";    //point inside box AND (new 6.2.2012) not in boarder of 56x56 grid
    			} else {
    				cout << " - ";    //point not inside box (or near 56x56 boarder => no 14x14 integral grid available => no calc of fv possible)
    			}
    	    }
            cout << endl;
    	}
    }
}




void CCalc_Grasppoints::predict_bestgp_withsvm(bool svm_with_probability){
	//executes:
	//	./svm-scale -r ./tools/range /tmp/features.txt > /tmp/features.txt.scale
	//  ./svm-predict /tmp/features.txt.scale ./tools/mixedmanual_features_1000_rand.txt.model output
	//scale feature vectors

	try{
		string pkg_path = ros::package::getPath("haf_grasping");

		//scale: use existing scaling file to scale feature values

		stringstream ss, ss2;
		ss << pkg_path << "/libsvm-3.12/svm-scale -r " << pkg_path << "/data/range21062012_allfeatures /tmp/features.txt > /tmp/features.txt.scale";
		string command = ss.str();
		int i = system(command.c_str());
		cout << "\n\n i (scale): " << i << endl;

		if (!svm_with_probability){
			//predict grasping points

			//	trained with all examples
			ss2 << pkg_path << "/libsvm-3.12/svm-predict /tmp/features.txt.scale " << pkg_path << "/data/all_features.txt.scale.model /tmp/output_calc_gp.txt";
			string command2 = ss2.str();
			i = system(command2.c_str());

		} else {
			//	trained with all manual examples, using probability as output
			i = system("/usr/lib/libsvm/libsvm-3.1/svm-predict -b 1 /tmp/features.txt.scale /usr/lib/libsvm/libsvm-3.1/tools/allmanualfeatures.txt.scale.p.model /tmp/output_calc_gp.txt");
		}
		cout << "\n\n i (predict): " << i << endl;
	} catch (int j) {
		cerr << "ERROR in calc_grasppoints.cpp: predict_bestgp_withsvm()" << endl;
	}


}


void CCalc_Grasppoints::show_predicted_gps(int nr_roll, int tilt, bool svm_with_probability){
	//PARAMETERS
	string path_svm_output =  	"/tmp/output_calc_gp.txt";

	//open file
	ifstream file_in(path_svm_output.c_str());


	//define x,y,and roll for top grasp
	int id_row_top_all = -1;
	int id_col_top_all = -1;
	int nr_roll_top_all = -1;
	int nr_tilt_top_all = -1;
	int topval_gp_all = -1000;

	string line;
	getline(file_in, line);

	bool printgraspsgrid = false;
	bool printgraspseval = false;
	float graspsgrid[HEIGHT][WIDTH];
	float graspseval[HEIGHT][WIDTH];

	for (int row = 0; row < HEIGHT; row++){
		for (int col = 0; col < WIDTH; col++){
			//check if gp is possible => if featurevector was calculated; no calculation of fv before=> take -1
			if (this->point_inside_box_grid[nr_roll][tilt][row][col] == false){
				graspsgrid[row][col] = -1;
			} else {
				if (svm_with_probability){
					int start=0,end=0, res;
					res = atof(line.substr(0,2).c_str());
					start = line.find(" ",0);
					end = line.find(" ",start+1);
					if (res>0){  //=> is gp => have to take second probability value in file (otherwise first)
						start = end;
						end = line.find(" ", start+1);
					}
					float prob = atof(line.substr(start,end).c_str());
					graspsgrid[row][col] = res*prob;
				} else {
					graspsgrid[row][col] = atoi(line.substr(0,2).c_str());
				}

				getline(file_in, line);
			}
		}
	}

	//print graspgrid in intuitive way (if variable printgraspsgrid is set true)

	if (printgraspsgrid){
		cout << "\n Graspsgrid: intuitiv! \n";
		for (int row = HEIGHT-1; row >= 0; row--){
			cout << row << ") \t";
			for (int col = WIDTH-1; col >= 0; col--){
				cout << graspsgrid[row][col] << "\t";
			}
			cout << "\n";
		}
	}


	if (printgraspseval) cout << "graspseval[row][col]" << "\n\n";
	int w1=1, w2=2,w3=3,w4=4,w5=55; //weights: direct grasp neighbors have more impact
	int topval_gp = -1000; //top rated value of all grasping points (same rating => take first gp)
	int id_row_top = -1, id_col_top = -1;
	for (int row = 0; row < HEIGHT; row++){
		for (int col = 0; col < WIDTH; col++){
			if ( graspsgrid[row][col] < 0 ){
				graspseval[row][col] = 0;
			} else {
				graspseval[row][col] =
						w1*graspsgrid[row-2][col-2]+w2*graspsgrid[row-2][col-1]+w3*graspsgrid[row-2][col]+w2*graspsgrid[row-2][col+1]+w1*graspsgrid[row-2][col+2]+ //row-2
						w2*graspsgrid[row-1][col-2]+w3*graspsgrid[row-1][col-1]+w4*graspsgrid[row-1][col]+w3*graspsgrid[row-1][col+1]+w2*graspsgrid[row-1][col+2]+ //row -1
						w2*graspsgrid[row][col-4] + w2*graspsgrid[row][col-3] + w3*graspsgrid[row][col-2] + w4*graspsgrid[row][col-1] + w5*graspsgrid[row][col] + w4*graspsgrid[row][col+1] + w3*graspsgrid[row][col+2] + w2*graspsgrid[row][col+3] +w2*graspsgrid[row][col+4] + //row
						w2*graspsgrid[row+1][col-2]+w3*graspsgrid[row+1][col-1]+w4*graspsgrid[row+1][col]+w3*graspsgrid[row+1][col+1]+w2*graspsgrid[row+1][col+2]+ //row +1
						w1*graspsgrid[row+2][col-2]+w2*graspsgrid[row+2][col-1]+w3*graspsgrid[row+2][col]+w2*graspsgrid[row+2][col+1]+w1*graspsgrid[row+2][col+2]; //row +2

			}
			if (printgraspseval) cout << graspseval[row][col] << "\t";
			if (graspseval[row][col] > topval_gp){
				topval_gp = graspseval[row][col];
				id_row_top = row;
				id_col_top = col;
				if (topval_gp > topval_gp_all){
					cout << "\n new overall grasp evaluation with value: " << topval_gp << "  in row,col: [" << id_row_top << ", " << id_col_top <<"] and rotation: " << nr_roll*ROLL_STEPS_DEGREE << endl;
					id_row_top_all = id_row_top;
					id_col_top_all = id_col_top;
					nr_roll_top_all = nr_roll;
					nr_tilt_top_all = tilt;
					topval_gp_all = topval_gp;
				}
			}
		}
		if (printgraspseval) cout << "\n";
	}


	//publish grasp grid as ros marker message (visualization_msgs::ArrayMarker) for rviz
	marker_cnt = 1;
	publish_grasp_grid(nr_roll, tilt, graspseval, 1);

	//choose best gp out of top rated ones (the one in the middle)
	int best_topval_row = -1, best_topval_col = -1;
	int longest_topval_len = 0;		//indicates the length of longest found sequence of topvals in one row
	int cur_topval_len = 0;			//indicates the length of the current run/sequence
	bool count_topval_len = false;
	for (int row = 0; row < HEIGHT; row++){
		cur_topval_len = 0;
		count_topval_len = false;
		for (int col = 0; col < WIDTH; col++){
			if (graspseval[row][col] == topval_gp){
				count_topval_len = true;
				cur_topval_len++;
				if (cur_topval_len > longest_topval_len){	//new longest run of topvals
					longest_topval_len = cur_topval_len;
					best_topval_row = row;
					best_topval_col = col - cur_topval_len/2;
					if (topval_gp == topval_gp_all){
						cout << "\n better topval gp still with topval: " << topval_gp << "  in row,col: [" << best_topval_row << ", " << best_topval_col <<"] and rotation: " << nr_roll*ROLL_STEPS_DEGREE << endl;
						id_row_top_all = best_topval_row;
						id_col_top_all = best_topval_col;
					}
				}
			}
			else {
				count_topval_len = false;
				cur_topval_len = 0;
			}
		}
	}


	//show grasp predicted in mirrored (more intuitive) way
	if (printgraspseval) {
		cout << "\n grasps predicted more intuitiv: \n";
		for (int row = HEIGHT-1; row >= 0; row--){
			for (int col = WIDTH-1; col >= 0; col--){
				cout << setw(3) << setprecision(3) <<graspseval[row][col];
				if (col > 6 and col < WIDTH-7 )
					cout << " ";
			}
			if (row > 7 and row < HEIGHT - 7)
				cout << "\n";
			cout << endl;
		}
	}
	cout << "\n best graspvalue so far: " << topval_gp_all << endl;

	file_in.close();

	if (topval_gp_all > this->topval_gp_overall){	//new best grasp hypothesis!
		cout << "\n new overall (overall!) grasp evaluation with value: " << topval_gp_all << "  in row,col: [" << id_row_top_all << ", " << id_col_top_all <<"] and rotation: " << nr_roll*ROLL_STEPS_DEGREE << endl;
		this->id_row_top_overall = id_row_top_all;
		this->id_col_top_overall = id_col_top_all;
		this->nr_roll_top_overall = nr_roll_top_all;
		this->nr_tilt_top_overall = nr_tilt_top_all;
		this->topval_gp_overall = topval_gp_all;
	}

	if (this->return_only_best_gp){
		cout << "\n Loop finished. Only best grasp is published after full gp calculation! \n";
	} else if ((topval_gp_all > this->graspval_th)  ){
		//calculate coordinates for best grasp points with roll in world
		//transfer validation to value between 10 and 99 (if smaler 26, then value is set to 10)
		int scaled_gp_eval = topval_gp_all-20;
		if (scaled_gp_eval < 10) scaled_gp_eval = 10;	//should not be neccessary since graspval_th is high enough
		transform_gp_in_wcs_and_publish(id_row_top_all, id_col_top_all,nr_roll_top_all, nr_tilt_top_all, scaled_gp_eval);
	} else { //better grasps would be published but were not found for this loop
		cout << "\n No grasp point found above treshold for last loop!!! \n";
	}
}





void CCalc_Grasppoints::publish_grasp_grid(int nr_roll, int tilt, float graspsgrid[][WIDTH], int gripperwidth=1){
	visualization_msgs::MarkerArray ma;
	visualization_msgs::MarkerArray ma_params;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker marker_graps_params;
	cout << "publish_grasp_grid: input parameter gripperwidth: " << gripperwidth << endl;
	float x,y,z;
	x = this->box_center_x - 0.01*HEIGHT/2/gripperwidth;
	y = this->box_center_y - 0.01*WIDTH/2;
	cout << "x: " << x << endl;
	z=0.15;


	//show grasp params (grasp search field size, gripper closing direction)
	for (int i = 1; i <= 3; i++){
		grasp_area_to_marker(&marker_graps_params, this->box_center_x,this->box_center_y, z, gripperwidth, nr_roll, i /*param_id*/, false /*top_grasp*/ );   //add grasping direction to marker
		ma_params.markers.push_back(marker_graps_params);
	}



	for (int row = 0; row <= HEIGHT-1; row++){
		for (int col = 0; col <= WIDTH-1; col++){
			if (this->point_inside_box_grid[nr_roll][tilt][row][col] == true){
				gp_to_marker(&marker, x+0.01/gripperwidth*row,y+0.01*col,z, graspsgrid[row][col], false, gripperwidth, nr_roll, false); // before 0 istead of nr_roll
				ma.markers.push_back(marker);
			}
		}
	}
	// show (add) grasping direction:
	gp_to_marker(&marker, this->box_center_x,this->box_center_y,z, graspsgrid[0][0], false, gripperwidth, nr_roll, true);   //add grasping direction to marker
	ma.markers.push_back(marker);	// add gripper opening direction to marker array
    //vis_pub.publish( marker );
    vis_pub_ma.publish(ma);
    vis_pub_ma_params.publish(ma_params);  //publish grasp params as marker array
    //ros::spinOnce();
}



void CCalc_Grasppoints::gp_to_marker(visualization_msgs::Marker *marker, float x, float y, float z, float green, bool pubmarker, int gripperwidth=1, int nr_roll=0,bool pub_grip_open_dir=false){

	// broadcast helper tf-frame for correct visualization of grasping points
	string tmp_tf_help = "tf_help";
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(this->box_center_x, this->box_center_y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, -1.0*nr_roll*15*3.14159/180);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", tmp_tf_help));

	std::stringstream ss;
	ss << tmp_tf_help ;
	(*marker).header.frame_id = ss.str();  //"base_link";
	(*marker).header.stamp = ros::Time();
	(*marker).ns = "my_namespace";
	(*marker).id = marker_cnt++;
	(*marker).type = visualization_msgs::Marker::SPHERE;
	(*marker).action = visualization_msgs::Marker::ADD;
	(*marker).pose.position.x = x-this->box_center_x;
	(*marker).pose.position.y = y-this->box_center_y;
	(*marker).pose.position.z = z;
	(*marker).pose.orientation.x = 0.0;
	(*marker).pose.orientation.y = 0.0;
	(*marker).pose.orientation.z = 0.0;
	(*marker).pose.orientation.w = 1.0;
	(*marker).scale.x = 0.002;
	(*marker).lifetime = ros::Duration(2);
	(*marker).scale.y = 0.002;
	(*marker).scale.z = 0.002;
	(*marker).color.a = 1.0;
	if (green > 0){
		(*marker).color.r = 0.0;
		(*marker).color.g = 1.0;
		(*marker).scale.x = 0.005;
		(*marker).scale.y = 0.005;
		(*marker).scale.z = 0.001*green;
		(*marker).pose.position.z = z+(*marker).scale.z/2;
	} else {
		(*marker).color.r = 1.0;
		(*marker).color.g = 0.0;
	}
	(*marker).color.b = 0.0;
}

//this function publishes the area where grasps are searched for specific roll for visualization
void CCalc_Grasppoints::grasp_area_to_marker(visualization_msgs::Marker *marker, float x, float y, float z, int gripperwidth=1, int nr_roll=0, int param_id=1, bool top_grasp=false){

	int fix_marker_id_gripper_dir = 10000;
	int fix_marker_id_gripper_appr_dir = 10001;
	int fix_marker_id_grasp_search_area = 10002;
	int fix_marker_id_grasp_search_area2 = 10003; //inner part where feature vectors are really calculated

	(*marker).header.frame_id = "base_link";
	(*marker).header.stamp = ros::Time();
	(*marker).ns = "haf_grasping_parameters";

	//select which marker part is returned
	switch(param_id) {
	    case 1 : {  //publish (outer) grasping search area
					(*marker).id = fix_marker_id_grasp_search_area;
					(*marker).type = visualization_msgs::Marker::CUBE;
					(*marker).action = visualization_msgs::Marker::ADD;
					(*marker).color.a = 0.05;
					(*marker).color.r = 0.4;
					(*marker).color.g = 0.4;
					(*marker).color.b = 0.4;
					(*marker).pose.position.x = this->box_center_x;
					(*marker).pose.position.y = this->box_center_y;
					(*marker).pose.position.z = z;
					(*marker).pose.orientation.x = 0.0;
					(*marker).pose.orientation.y = 0.0;
					(*marker).pose.orientation.z = 0.0;
					(*marker).pose.orientation.w = 1.0;
					(*marker).scale.x = 0.01*this->grasp_search_area_size_x_dir/ gripperwidth;
					(*marker).scale.y = 0.01*this->grasp_search_area_size_y_dir;
					(*marker).scale.z = 0.001;
					(*marker).lifetime = ros::Duration();
					break;
	    		 }

	    case 2 : {  //pub search area, pub inner area:
					(*marker).id = fix_marker_id_grasp_search_area2;
					(*marker).type = visualization_msgs::Marker::CUBE;
					(*marker).action = visualization_msgs::Marker::ADD;
					(*marker).color.a = 0.4;
					(*marker).color.r = 0.2;
					(*marker).color.g = 0.2;
					(*marker).color.b = 0.2;
					(*marker).pose.position.x = this->box_center_x;
					(*marker).pose.position.y = this->box_center_y;
					(*marker).pose.position.z = z;
					(*marker).pose.orientation.x = 0.0;
					(*marker).pose.orientation.y = 0.0;
					(*marker).pose.orientation.z = 0.0;
					(*marker).pose.orientation.w = 1.0;
					(*marker).scale.x = 0.01*(this->grasp_search_area_size_x_dir-14)/ gripperwidth;
					(*marker).scale.y = 0.01*(this->grasp_search_area_size_y_dir-14);
					(*marker).scale.z = 0.001;
					(*marker).lifetime = ros::Duration();
					break;
	    		 }
	    case 3 : {  // draw gripper closing direction (red line)
					(*marker).id = fix_marker_id_gripper_dir;
					(*marker).type = visualization_msgs::Marker::CUBE;
					(*marker).action = visualization_msgs::Marker::ADD;
					(*marker).color.a = 0.8;
					(*marker).color.r = 1.0;
					(*marker).color.g = 0.0;
					(*marker).color.b = 0.0;
					if (top_grasp){
						(*marker).pose.position.x = x;
						(*marker).pose.position.y = y;
					} else {
						(*marker).pose.position.x = this->box_center_x;
						(*marker).pose.position.y = this->box_center_y;
					}
					(*marker).pose.position.z = z;
					(*marker).scale.x = 0.5 * 1 / gripperwidth;
					(*marker).scale.y = 0.002;
					(*marker).scale.z = 0.002;
					(*marker).lifetime = ros::Duration(3);
					tf::Vector3 marker_gripper_open_dir_axis(0, 0, 1);
					tf::Quaternion qt_gripper_open_dir(marker_gripper_open_dir_axis, -nr_roll*ROLL_STEPS_DEGREE*PI/180);
					geometry_msgs::Quaternion quat_msg_gripper_open_dir;
					tf::quaternionTFToMsg(qt_gripper_open_dir, quat_msg_gripper_open_dir);
					(*marker).pose.orientation = quat_msg_gripper_open_dir;
					(*marker).lifetime = ros::Duration();
					break;
	    		 }
	    case 4 : {  // draw grasp approach direction (black arrow)
					(*marker).id = fix_marker_id_gripper_appr_dir;
					(*marker).type = visualization_msgs::Marker::ARROW;
					(*marker).action = visualization_msgs::Marker::ADD;
					(*marker).color.a = 0.8;
					(*marker).color.r = 0.0;
					(*marker).color.g = 0.0;
					(*marker).color.b = 0.0;
					(*marker).pose.position.x = x;	//	df 15.4.2015
					(*marker).pose.position.y = y;	//	df 15.4.2015
					(*marker).pose.position.z = z+0.10;
					(*marker).scale.x = 0.10;
					(*marker).scale.y = 0.003;
					(*marker).scale.z = 0.003;
					(*marker).lifetime = ros::Duration(3);
					tf::Vector3 marker_axis(0, 1, 0);
					tf::Quaternion qt(marker_axis, PI/2);
					geometry_msgs::Quaternion quat_msg;
					tf::quaternionTFToMsg(qt, quat_msg);
					(*marker).pose.orientation = quat_msg;
					(*marker).lifetime = ros::Duration();
					break;
	    		 }
	    default: cout << "CCalc_Grasppoints::grasp_area_to_marker: WRONG INPUT FOR param_id " << endl;
	    break;
	}
}

//input: best row and col in virtual rotated box and rotation angle
//output: calculates grasp point in world coordinate system and publishes it
void CCalc_Grasppoints::transform_gp_in_wcs_and_publish(int id_row_top_all, int id_col_top_all,int nr_roll_top_all, int nr_tilt_top_all, int scaled_gp_eval){
	cout << "\n -> transform_gp_in_wcs_and_publish() " << endl;
	Eigen::Matrix4f mat_sh_orig = Eigen::Matrix4f::Identity(); //matrix which defines shift of point cloud from center to origin
	Eigen::Matrix4f mat_rot = Eigen::Matrix4f::Identity(); //matrix which rotates pc
	Eigen::Matrix4f mat_tilt = Eigen::Matrix4f::Identity(); //matrix which tilts pc
	Eigen::Matrix4f mat_sh_fr_orig = Eigen::Matrix4f::Identity(); //matrix which defines shift of point cloud from origin to center
	Eigen::Matrix4f mat_transform = Eigen::Matrix4f::Identity(); //matrix for transformation of pointcloud (rotation about box center)
	Eigen::Matrix3f mat_rot_for_appr_vec = Eigen::Matrix3f::Identity(); //matrix which makes the rotation for the approximation vector

	mat_sh_orig(0,3) = -this->box_center_x;  //shift x value
	mat_sh_orig(1,3) = -this->box_center_y;	 //shift y-value
	mat_sh_fr_orig(0,3) = this->box_center_x;
	mat_sh_fr_orig(1,3) = this->box_center_y;

	//float angle = -nr_roll_top_all*ROLL_STEPS_DEGREE*PI/180 - this->boxrot_angle_init;
	float angle = -(nr_roll_top_all)*ROLL_STEPS_DEGREE*PI/180 - this->boxrot_angle_init;
	mat_rot(0,0) = cos(angle);
	mat_rot(0,1) = -sin(angle);
	mat_rot(1,0) = sin(angle);
	mat_rot(1,1) = cos(angle);

	//david : set mat_tilt correctly, rewrite matrizes and bring mat_tilt in correct order with the other ones
	//NEW
	float beta = nr_tilt_top_all*TILT_STEPS_DEGREE*PI/180; //angle for tilt in Rad, vorzeichen richtig??
	mat_tilt(0,0) = cos(beta);
	mat_tilt(0,2) = -sin(beta);
	mat_tilt(2,0) = sin(beta);
	mat_tilt(2,2) = cos(beta);
	//NEW_END

	mat_transform = mat_sh_fr_orig*mat_tilt*mat_rot*mat_sh_orig;

	cout << "\n id_row_top_all: " << id_row_top_all << "\t" << "id_col_top_all: " << id_col_top_all << "\t and rotation: " << nr_roll_top_all*ROLL_STEPS_DEGREE << "  tilt_top: " << nr_tilt_top_all <<endl;
	//publish best grasp point (with roll):
	float x_gp_roll = this->box_center_x - ((float)(HEIGHT/2 - id_row_top_all))/100;	//x-coordinate of the graspcenterpoint if the box would be rotated by nr_roll_top_all*ROLL_STEPS_DEGREE degree
	float y_gp_roll = this->box_center_y - ((float) (WIDTH/2 - id_col_top_all))/100;  //y-coordinate of the graspcenterpoint if the box would be rotated by nr_roll_top_all*ROLL_STEPS_DEGREE degree
	//detect z-value for grasping (max height in area around grasp center - 0 cm
	float h_locmax_roll = -10;
	for (int row_z = -4; row_z < 5; row_z++){	//row_z defines which rows are taken into account for the calc. of max z
		for (int col_z = -4; col_z < 4; col_z++){
			//cout << "this->heightsgridroll[" << nr_roll_top_all << "][" << nr_tilt_top_all << "][" << id_row_top_all+row_z << "][" << id_col_top_all+col_z << "]: " << this->heightsgridroll[nr_roll_top_all][nr_tilt_top_all][id_row_top_all+row_z][id_col_top_all+col_z] << endl;
			if (id_row_top_all+row_z >= 0 and id_col_top_all+col_z>=0 and h_locmax_roll < this->heightsgridroll[nr_roll_top_all][nr_tilt_top_all][id_row_top_all+row_z][id_col_top_all+col_z]){
				h_locmax_roll = this->heightsgridroll[nr_roll_top_all][nr_tilt_top_all][id_row_top_all+row_z][id_col_top_all+col_z];
				//cout << "this->heightsgridroll[" << nr_roll_top_all <<"][" << nr_tilt_top_all<<"][" <<id_row_top_all+row_z<<"][" << id_col_top_all+col_z<<"]: " << this->heightsgridroll[nr_roll_top_all][nr_tilt_top_all][id_row_top_all+row_z][id_col_top_all+col_z] << endl;
			}
		}
	}


	h_locmax_roll -= 0.02;
	float z_gp_roll = h_locmax_roll;
	cout << "x_roll: "<< x_gp_roll << "\t y_roll: " << y_gp_roll << "\t z_gp_roll: " << z_gp_roll << endl;

	//make rotation
	//pcl::transformPointCloud(pcl_cloud_in, pcl_cloud_in_roll, mat_transform);
    float x_gp_dis = 0.03; //distance from grasp center to "real" grasp point

	Eigen::Vector4f gp1(x_gp_roll-x_gp_dis, y_gp_roll,z_gp_roll,1.0);
	Eigen::Vector4f gp2(x_gp_roll+x_gp_dis, y_gp_roll,z_gp_roll,1.0);
	Eigen::Vector4f gp1_wcs, gp2_wcs;
	Eigen::Vector3f appr_vec(0,0,1);

	gp1_wcs = mat_transform*gp1;
	gp2_wcs = mat_transform*gp2;

	mat_rot_for_appr_vec << mat_tilt(0,0),mat_tilt(0,1),mat_tilt(0,2),   mat_tilt(1,0),mat_tilt(1,1),mat_tilt(1,2),   mat_tilt(2,0),mat_tilt(2,1),mat_tilt(2,2);
	appr_vec = mat_rot_for_appr_vec*appr_vec;

	cout << "\n gp1: x_roll_wcs: "<< gp1_wcs[0] << "\t y_roll_wcs: " << gp1_wcs[1] << "\t z_roll_wcs: " << gp1_wcs[2] << endl;
	cout << " gp2: x_roll_wcs: "<< gp2_wcs[0] << "\t y_roll_wcs: " << gp2_wcs[1] << "\t z_roll_wcs: " << gp2_wcs[2] << endl;


	//Publish grasp points
	std_msgs::String msgStrPoints;
    std::stringstream ss;

    //David: achtung, jetzt noch mit eval!!
    ss << scaled_gp_eval << " " << gp1_wcs[0] <<" "<< gp1_wcs[1] << " "<< gp1_wcs[2] << " "<< gp2_wcs[0] << " " << gp2_wcs[1] << " " << gp2_wcs[2] << " "<< appr_vec(0) << " "<< appr_vec(1) << " "<< appr_vec(2) << " " << (gp1_wcs[0]+gp2_wcs[0])/2.0 <<" " << (gp1_wcs[1]+gp2_wcs[1])/2.0 <<" " << (gp1_wcs[2]+gp2_wcs[2])/2.0 <<" " << nr_roll_top_all*ROLL_STEPS_DEGREE;
 	msgStrPoints.data = ss.str();
 	this->gp_result = ss.str();		//new 3.12.2014

 	//publish best grasp as rviz visualization_msgs::Marker
 	visualization_msgs::Marker marker_best_params; //dddddd
	visualization_msgs::MarkerArray ma_best_params;
	//show grasp params (grasp search field size, gripper closing direction)
	for (int i = 3; i <= 4; i++){
		grasp_area_to_marker(&marker_best_params, (gp1_wcs[0]+gp2_wcs[0])/2,(gp1_wcs[1]+gp2_wcs[1])/2,(gp1_wcs[2]+gp2_wcs[2])/2, 1 /*gripperwidth*/, nr_roll_top_all, i /*param_id*/, true /*top_grasp*/ );   //add best grasping direction to marker
		ma_best_params.markers.push_back(marker_best_params);
	}
	vis_pub_ma_params.publish(ma_best_params);	//publish best grasp parameter

 	//gp_to_marker(&marker, (gp1_wcs[0]+gp2_wcs[0])/2,(gp1_wcs[1]+gp2_wcs[1])/2,(gp1_wcs[2]+gp2_wcs[2])/2, false, true, 1, nr_roll_top_all, true);
 	//gp_to_marker(&marker, 0.01*this->id_row_top_overall,0.01*this->id_col_top_overall,(gp1_wcs[2]+gp2_wcs[2])/2, false, true, 1, nr_roll_top_all, true);
 	//pubGraspPoints.publish(msgStrPoints);
	cout << "\n scaled_gp_eval: " << scaled_gp_eval << endl;
	pubGraspPointsEval.publish(msgStrPoints);

}



int main (int argc, char** argv)
{
  ROS_INFO("ROS NODE calc_grasppoints_svm_action_server (from package haf_grasping) started");
  ros::init(argc, argv, "calc_grasppoints_svm_action_server");
  CCalc_Grasppoints * calc_gps = new CCalc_Grasppoints(ros::this_node::getName());
  ros::spin();
  return (0);
}
