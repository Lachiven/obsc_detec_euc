#include "ros/ros.h" 
#include <boost/thread.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include "std_msgs/Float32MultiArray.h"

using namespace std;
#define column 5         //60 240 po
#define row 20           //20 40
#define radiaus 50
#define step 1          //1 0.25

int calcu = 0;
int cal[50000];
float left_rec = 1;
float right_rec = -1;

sensor_msgs::PointCloud2 pc2;
visualization_msgs::Marker marker;

vector<int>save_vec;
vector<pcl::PointXYZ> save_aabb_max;
vector<pcl::PointXYZ> save_aabb_min;


class GRID
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud {new pcl::PointCloud<pcl::PointXYZ>};
	pcl::PointIndices::Ptr grid_inliers {new pcl::PointIndices};
	bool low_emp=true;
	float min_height;
};

class SubscribeAndPublish  
{
public:  
  SubscribeAndPublish()  
  {  
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obj", 10);  
	pub2_ = nh.advertise<visualization_msgs::Marker>("/box1", 10 );
	pub_msg_ = nh.advertise<std_msgs::Float32MultiArray>("/xychatter", 10);
    sub_ = nh.subscribe("/sync_lidar_points", 10, &SubscribeAndPublish::callback2, this); 
	//sub2_ = n_.subscribe("/filtered_points_no_ground", 10, &SubscribeAndPublish::callback2, this);
  }  
  void callback2(const sensor_msgs::PointCloud2::ConstPtr& msg2); 
private:  
  ros::NodeHandle nh;   
  ros::Publisher pub_;  
  ros::Publisher pub2_;
  ros::Publisher pub_msg_;
  ros::Subscriber sub_;
};//End of class SubscribeAndPublish  
double leff(double f)
{ 
  double t1 = f - (int)f;
  t1 = t1 * 100;//小数点左移。
  if(t1 - (int)t1 >= 0.5) t1 += 1; //四舍五入
  t1 = (int)t1; //取整数部分。
  t1 = t1 / 100;//小数点右移。
  t1+=(int)f;//加上原本的整数部分
  return t1;
} 
void bub(vector<pcl::PointXYZ>& a,vector<pcl::PointXYZ>& A,int b){
  int i,j;
  double Temp1,Temp2,Temp3;
  for (i=0;i<b-1;i++){
	  for (j=0;j<b-1-i;j++){
		  double x = leff(a[j].x); double x1 = leff(a[j+1].x);
		  double y = leff(a[j].y); double y1 = leff(a[j+1].y); double Y1 = leff(A[j+1].y);
		  if (x > x1){
			  Temp1 = x1; Temp2 = y1; Temp3 = Y1;
			  x1 = x; y1 = y; Y1 = y;
			  x = Temp1;y = Temp2; y = Temp3;
			}
		}
    } 
}
void marker_(vector<pcl::PointXYZ> &save_aabb_max,vector<pcl::PointXYZ> &save_aabb_min,visualization_msgs::Marker &marker,int n,int i){
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = n;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (save_aabb_max[i].x + save_aabb_min[i].x)/2;
    marker.pose.position.y = (save_aabb_max[i].y + save_aabb_min[i].y)/2;
    marker.pose.position.z = (save_aabb_max[i].z + save_aabb_min[i].z)/2;
    cout<<"x "<<save_aabb_min[i].x<<"y "<<save_aabb_min[i].y<<endl;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = save_aabb_max[i].x - save_aabb_min[i].x;
    marker.scale.y = save_aabb_max[i].y - save_aabb_min[i].y;
    marker.scale.z = save_aabb_max[i].z - save_aabb_min[i].z;
    marker.color.a = 1; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.1);
}
void add_cube_fun(pcl::PointCloud<pcl::PointXYZ>::Ptr cube_raw,vector<pcl::PointXYZ> &a,vector<pcl::PointXYZ> &b){
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (cube_raw);
	feature_extractor.compute ();
	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;
	feature_extractor.getMomentOfInertia (moment_of_inertia);
	feature_extractor.getEccentricity (eccentricity);
	feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter (mass_center);
	a.push_back(max_point_AABB);
	b.push_back(min_point_AABB);
}

void SubscribeAndPublish::callback2(const sensor_msgs::PointCloud2::ConstPtr& msg2){	  //point.size() 64896 height 32;width 2028;
	srand (time(NULL));  //seeds rand() with the system time time(NULL)这个函数的返回值是作为srand函数的参数的！意思是以现在的系统时间作为随机数的种子来产生随机数！至于NULL这个参数。只有设置成NULL才能获得系统的时间！
	time_t begin,end;
	begin = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg2,pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	pcl::PassThrough<pcl::PointXYZ> passx;
	passx.setInputCloud(temp_cloud);
	passx.setFilterFieldName("x");
	passx.setFilterLimits(0.3, 4); 
	passx.setFilterLimitsNegative (false);
	passx.filter(*temp_cloud);
	std::vector<int> indicesx; 
	pcl::removeNaNFromPointCloud(*temp_cloud,*temp_cloud, indicesx); 

	pcl::PassThrough<pcl::PointXYZ> passy;
	passy.setInputCloud(temp_cloud);
	passy.setFilterFieldName("y");
	passy.setFilterLimits(-1.8, 1.8); 
	passy.setFilterLimitsNegative (false);
	passy.filter(*temp_cloud);
	std::vector<int> indicesy; 
	pcl::removeNaNFromPointCloud(*temp_cloud,*temp_cloud, indicesy); 

	pcl::PassThrough<pcl::PointXYZ> passz;
	passz.setInputCloud(temp_cloud);
	passz.setFilterFieldName("z");
	passz.setFilterLimits(-0.5, 0.8); 
	passz.setFilterLimitsNegative (false);
	passz.filter(*temp_cloud);
	std::vector<int> indicesz; 
	pcl::removeNaNFromPointCloud(*temp_cloud,*temp_cloud, indicesz); 


	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
	voxel_grid_filter.setLeafSize(0.05, 0.05, 0.05);
	voxel_grid_filter.setInputCloud(temp_cloud);
	voxel_grid_filter.filter(*temp_cloud);
	//pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
	//--------GRID-------//
	/*GRID grid[radiaus];
	for( int count = 0; count < temp_cloud->points.size(); count++ ){
		for( int i = 0; i < radiaus; i++ ){
			if( i * i < temp_cloud->points[count].x * temp_cloud->points[count].x + temp_cloud->points[count].y * temp_cloud->points[count].y < (i + 1) * (i + 1) ){
				grid[i].grid_inliers->indices.push_back(count);
				grid[i].grid_cloud->points.push_back(temp_cloud->points[count]);//压入GRID派生的grid_cloud中
			}
		}
	}
	for( int i = 0; i < radiaus; i++ ){
		for( int k = 0; k < grid[i].grid_cloud->points.size(); k++ ){
			if(grid[i].grid_cloud->points[k].z < -0.14){
				save_vec.push_back(grid[i].grid_inliers->indices[k]);
			}
		}
	}	*/
    /*GRID grid[column][row];
    for( int count = 0; count < temp_cloud->points.size(); count++ ){ //第count个点进来
		for( int i = - column / 2; i < column / 2; i++ ){												 //-60 - 60
			if( temp_cloud->points[count].x > i * step && temp_cloud->points[count].x < (i + 1) * step ){//这两句循环看看符合哪个x区间
				for(int j = - row / 2; j < row / 2; j++){
					if( temp_cloud->points[count].y > j * step && temp_cloud->points[count].y < (j + 1) * step ){//这两句循环看看符合哪个y区间
						grid[i + column/2][j + row / 2].grid_inliers->indices.push_back(count);
						grid[i + column/2][j + row / 2].grid_cloud->points.push_back(temp_cloud->points[count]);//压入GRID派生的grid_cloud中
						if(grid[ i+ column/2][j + row / 2].low_emp){
							grid[ i+ column/2][j + row / 2].min_height = temp_cloud->points[count].z;
							grid[ i+ column/2][j + row / 2].low_emp=false;//记录最低点z坐标为min_height
						}
						else{
							//if(temp_cloud->points[count].z<grid[i][j+row/2].min_height){}
								grid[ i+ column/2][j+row/2].min_height = temp_cloud->points[count].z;
						}
					}
				}
			}
		}
	}
	//--------GRID-------//
	for(int i = 0; i < column; i++){
		for(int j = 0; j < row; j++){
			int point_num = grid[i][j].grid_cloud->points.size();
			if(grid[i][j].min_height < 0){
				for(int k=0; k < point_num; k++){
					///if( grid[i][j].grid_cloud->points[k].z > (grid[i][j].min_height+0.14) ){      //0.20
						//if( grid[i][j].grid_cloud->points[k].z < (grid[i][j].min_height+1.6) ){     //2.2地面滤出 高于车身滤出
						//	if((right_rec + 0.4 < grid[i][j].grid_cloud->points[k].y ) && (grid[i][j].grid_cloud->points[k].y < left_rec-0.4) ){
									save_vec.push_back(grid[i][j].grid_inliers->indices[k]);//车身滤出
						//	}
						//}	
					//}
				}
			}
		}
	}*/
	GRID grid[column];
    for(int count=0;count<temp_cloud->points.size();count++){
		for(int i=0;i<column;i++){
			if((sqrt(temp_cloud->points[count].x*temp_cloud->points[count].x+temp_cloud->points[count].y*temp_cloud->points[count].y) > i*step)&&(sqrt(temp_cloud->points[count].x*temp_cloud->points[count].x+temp_cloud->points[count].y*temp_cloud->points[count].y) < (i+1)*step)){
				grid[i].grid_inliers->indices.push_back(count);
				grid[i].grid_cloud->points.push_back(temp_cloud->points[count]);
				//if(grid[i].low_emp){
				//	grid[i].min_height = temp_cloud->points[count].z;
				//	grid[i].low_emp=false;
				//}
				//else{
				//	if(temp_cloud->points[count].z<grid[i].min_height){grid[i].min_height = temp_cloud->points[count].z;}
				//}
			}
		}
	}
	for(int i=0;i<column;i++){//每环检测一次
		int grid_num = grid[i].grid_cloud->points.size();
		//std::cout<<"i and it's number"<<i<<"and"<<grid_num<<std::endl;
		for(int k=0;k<grid_num;k++){
			save_vec.push_back(grid[i].grid_inliers->indices[k]);
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr all_piece (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*temp_cloud, save_vec, *all_piece);
		save_vec.clear();
		pcl::toROSMsg(*all_piece,pc2);
		pub_.publish(pc2);
		std::vector<int> indices2; 	
		pcl::removeNaNFromPointCloud(*all_piece,*all_piece, indices2); 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		if(all_piece->points.size() > 5){
			tree->setInputCloud (all_piece);
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance (0.07); //点之间查找的最小范围！！！！最重要参数 //0.07
			ec.setMinClusterSize (10);
			ec.setMaxClusterSize (120); //300
			ec.setSearchMethod (tree);
			ec.setInputCloud (all_piece);
			ec.extract (cluster_indices);
			if(cluster_indices.size() > 0){
				cout<<"test_size"<<cluster_indices.size()<<endl;
				int ecnumber = cluster_indices.size();
				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
					auto number = (int) it->indices.size();
					std::cout<<"   cloud size is : "<<number<<std::endl;
					cal[calcu] = number;
					calcu = calcu + 1;
					for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
						cloud_cluster->points.push_back (all_piece->points[*pit]); 
					}
					cloud_cluster->width = cloud_cluster->points.size ();
					cloud_cluster->height = 1;
					cloud_cluster->is_dense = true;
					add_cube_fun(cloud_cluster,save_aabb_max,save_aabb_min);
				}
			}
		}
	}

	/*//--------GRID-------//
	for(int i=0;i<column;i++){
		int grid_num = grid[i].grid_cloud->points.size();
		std::cout<<"i and it's number"<<i<<"and"<<grid_num<<std::endl;
		if(grid[i].min_height<0){
			for(int k=0;k<grid_num;k++){
				if( (grid[i].grid_cloud->points[k].z > (grid[i].min_height + 0.2)) ){      //0.20
					if(grid[i].grid_cloud->points[k].z < (grid[i].min_height + 1.2)){     //2.2
						if((right_rec < grid[i].grid_cloud->points[k].y )&&(grid[i].grid_cloud->points[k].y < left_rec)){	//左右滤波0.6m
							save_vec.push_back(grid[i].grid_inliers->indices[k]);
						}
					}	
				}
			}
		}
	}*/
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr all_piece (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*temp_cloud, save_vec, *all_piece);
  	save_vec.clear();
  	pcl::toROSMsg(*all_piece,pc2);
	pub_.publish(pc2);
  	std::vector<int> indices2; 	
  	pcl::removeNaNFromPointCloud(*all_piece,*all_piece, indices2); 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (all_piece);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.15); //点之间查找的最小范围
	ec.setMinClusterSize (5);
	ec.setMaxClusterSize (150); //300
	ec.setSearchMethod (tree);
	ec.setInputCloud (all_piece);
	ec.extract (cluster_indices);
	cout<<"test_size"<<cluster_indices.size()<<endl;
	int ecnumber = cluster_indices.size();
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		auto number = (int) it->indices.size();
		std::cout<<"   cloud size is : "<<number<<std::endl;
		cal[calcu] = number;
		calcu = calcu + 1;
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			cloud_cluster->points.push_back (all_piece->points[*pit]); 
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		add_cube_fun(cloud_cluster,save_aabb_max,save_aabb_min);
	}*/
	/*for( int i = calcu - ecnumber; i < ecnumber; i++ ){
		std::cout<<calcu<<std::endl;
		std::cout<<cal[i]<<std::endl;
		if(cal[i] < 20){
			for( int c = 0; c < ecnumber; c++ ){
				if( abs(cal[calcu - ecnumber - c] - cal[i]) < 5 )
				std::cout<<"exist 1"<<std::endl;
			}
		}
		else if(cal[i] < 50){
			for( int c = 0; c < calcu; c++ ){
				if( abs(cal[calcu - ecnumber - c] - cal[i]) < 8 )
				std::cout<<"exist 2"<<std::endl;
			}
		}
		else if(cal[i] < 120){
			for( int c = 0; c < calcu; c++ ){
				if( abs(cal[calcu - ecnumber - c] - cal[i]) < 20 )
				std::cout<<"exist 3"<<std::endl;
			}
		}
		else{
			for( int c = 0; c < calcu; c++ ){
				if( abs(cal[calcu - ecnumber - c] - cal[i]) < 25 )
				std::cout<<"exist 4"<<std::endl;
			}
		}
	}*/
	int n = 0;
	int size = save_aabb_max.size();
	std::cout<<"size"<<size<<std::endl;
	for(int i=0;i<size;i++){
		marker_(save_aabb_max,save_aabb_min,marker,n,i);
		pub2_.publish( marker );
		n++;
	}
	char buffhd[80];
	char* Send_msg;
	double a1, a2, a3;
	double b1, b2, b3;
	string buffhd_str;
	if(size == 0){
  		sprintf(buffhd,"$,%01d,!",0);
  		buffhd_str = buffhd;
  		Send_msg = const_cast<char*>(buffhd_str.c_str());
	} 

	if((size < 4)&&(size >= 1)){
		bub(save_aabb_min,save_aabb_max,size);
		if(size ==1 ){
			a1=save_aabb_min[0].x;a2=(save_aabb_min[0].y+save_aabb_max[0].y)/2; a3=save_aabb_max[0].y-save_aabb_min[0].y;
			sprintf(buffhd,"$,%01d,%05.1f,%05.1f,%04.1f,!",1,a1,a2,a3);
		}

		if(size ==2 ){
			a1=save_aabb_min[0].x; a2=(save_aabb_min[0].y+save_aabb_max[0].y)/2; a3=save_aabb_max[0].y-save_aabb_min[0].y;
			float b1=save_aabb_min[1].x;float b2=(save_aabb_min[1].y+save_aabb_max[1].y)/2;float b3=save_aabb_max[1].y-save_aabb_min[1].y;
			sprintf(buffhd,"$,%01d,%05.1f,%05.1f,%04.1f,%05.1f,%05.1f,%04.1f,!",2,a1,a2,a3,b1,b2,b3);
		}

		if(size ==3 ){
			a1=save_aabb_min[0].x; a2=(save_aabb_min[0].y+save_aabb_max[0].y)/2; a3=save_aabb_max[0].y-save_aabb_min[0].y;
			float b1=save_aabb_min[1].x;float b2=(save_aabb_min[1].y+save_aabb_max[1].y)/2;float b3=save_aabb_max[1].y-save_aabb_min[1].y;
			float c1=save_aabb_min[2].x;float c2=(save_aabb_min[2].y+save_aabb_max[2].y)/2;float c3=save_aabb_max[2].y-save_aabb_min[2].y;
			sprintf(buffhd,"$,%01d,%05.1f,%05.1f,%04.1f,%05.1f,%05.1f,%04.1f,%05.1f,%05.1f,%04.1f,!",3,a1,a2,a3,b1,b2,b3,c1,c2,c3);
		}
  		buffhd_str = buffhd;
  		Send_msg = const_cast<char*>(buffhd_str.c_str());
	}
	if(size >=4){
		bub(save_aabb_min,save_aabb_max,size);
		a1=save_aabb_min[0].x; a2=(save_aabb_min[0].y+save_aabb_max[0].y)/2; a3=save_aabb_max[0].y-save_aabb_min[0].y;
		float b1=save_aabb_min[1].x;float b2=(save_aabb_min[1].y+save_aabb_max[1].y)/2;float b3=save_aabb_max[1].y-save_aabb_min[1].y;
		float c1=save_aabb_min[2].x;float c2=(save_aabb_min[2].y+save_aabb_max[2].y)/2;float c3=save_aabb_max[2].y-save_aabb_min[2].y;
		float d1=save_aabb_min[3].x;float d2=(save_aabb_min[3].y+save_aabb_max[3].y)/2;float d3=save_aabb_max[3].y-save_aabb_min[3].y;
  		sprintf(buffhd,"$,%01d,%05.1f,%05.1f,%04.1f,%05.1f,%05.1f,%04.1f,%05.1f,%05.1f,%04.1f,%05.1f,%05.1f,%04.1f,!",4,a1,a2,a3,b1,b2,b3,c1,c2,c3,d1,d2,d3);
  		buffhd_str = buffhd;
  		Send_msg = const_cast<char*>(buffhd_str.c_str());
	}
	save_aabb_max.clear();
	save_aabb_min.clear();
	
	std_msgs::Float32MultiArray msg;
	b1 = leff(a1); b2 = leff(a2); b3 = leff(a3);
	std::cout<<"!!!!!!!!!!!!!!!!!!!!!"<<b1<<","<<b2<<","<<b3<<"!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
	msg.data.push_back(b1);
	msg.data.push_back(b2);
	msg.data.push_back(b3);
	pub_msg_.publish(msg);
  	end = clock(); 
	double Times =  double(end - begin) / CLOCKS_PER_SEC;
	std::cout<<"time: "<<Times<<"s"<<std::endl;
}

int main(int argc, char **argv)  
{  
	//My_Serial = new SerialPort("/dev/ttyUSB0",115200,8,1,"N");
	ros::init(argc, argv, "obj_detection_node");  //subscribe_and_publish
	SubscribeAndPublish test;  
	//ros::MultiThreadedSpinner s(2); 
	ros::spin();  
	return 0;  
}  

