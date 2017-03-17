#include "ros/ros.h"
#include "std_msgs/String.h"
#include "multi_robot_slam/Scenenode.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>

class multiRobWorkstation
{
    private:
	ros::NodeHandle n;
//	ros::Subscriber map_sub;
	ros::Subscriber sceneNode_sub;

	cv::of2::ChowLiuTree treeBuilder;
//	cv::of2::FabMap2 fabmap;
//	cv::Ptr<cv::of2::FabMap> fabmap;
	bool emptyTree;
	cv::Mat tree;
	int scene_frames;
	std::vector<cv::Mat> traindatas;
	cv::Mat result_display;

    public:
	multiRobWorkstation();
	~multiRobWorkstation();
    void sceneNodeCallback(const multi_robot_slam::Scenenode::ConstPtr& msg);
};

multiRobWorkstation::multiRobWorkstation()
	:emptyTree(true),
	 scene_frames(0)
{
	sceneNode_sub = n.subscribe("/image_converter/scene_node", 1, &multiRobWorkstation::sceneNodeCallback,this);
//	map_sub = n.subscribe("/map", 1, &sceneNodeCallback);
	 result_display = cv::Mat::zeros(500, 500, CV_8UC3);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "worstation");
  multiRobWorkstation mk;
  ros::spin();
  return 0;
}
multiRobWorkstation::~multiRobWorkstation()
{

}

void multiRobWorkstation::sceneNodeCallback(const multi_robot_slam::Scenenode::ConstPtr& msg)
{
//  ROS_INFO("I heard sceneNode message: [%lf]", msg->px);
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
	 cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
	 ROS_ERROR("cv_bridge exception: %s", e.what());
	 return;
  }
//  if(emptyTree)
//  {
//	  //create Chou-Liu tree
//	  ROS_INFO("Training Chow-liu tree");
//	//  cv::of2::ChowLiuTree treeBuilder;
//
//	  treeBuilder.add(cv_ptr->image);
//	  ROS_INFO("Add data");
//	  tree = treeBuilder.make();
//	  emptyTree = false;
//	  cv::Ptr<cv::of2::FabMap> fabmap = new cv::of2::FabMap(tree,0.39,0,cv::of2::FabMap::MEAN_FIELD | cv::of2::FabMap::CHOW_LIU);
//	  cv::Mat data;
//	  cv_ptr->image.convertTo(data, CV_32F);
//	  fabmap->addTraining(data);
//	  data.release();
//  }
//  cv::FileStorage fs("tree.yml",cv::FileStorage::WRITE);
//  ROS_INFO("Open file");
//  fs<<"tree"<<tree;
//  ROS_INFO("writting");
//  fs.release();
//  fs.writeObj("tree", tree);

//	  ROS_INFO("Make");
	  //run fabmap
//	  ROS_INFO("Running FabMap!");
	//  cv::Ptr<cv::of2::FabMap> fabmap;
	//  fabmap = new cv::of2::FabMap(tree,0.39,0,cv::of2::FabMap::SAMPLED | cv::of2::FabMap::CHOW_LIU);
//	  if(!fabmap)
//		  fabmap = new cv::of2::FabMap(tree,0.39,0,cv::of2::FabMap::MEAN_FIELD | cv::of2::FabMap::CHOW_LIU);
//	  ROS_INFO("Running FabMap!");
	//  cv::imshow("trainImage", cv_ptr->image);
	//  cv::waitKey();
	//  cv::Mat data(cv_ptr->image.rows,cv_ptr->image.cols,CV_32F);
//	  cv::imshow("8BITIMAGE", cv_ptr->image);
	  cv::Mat data;
	  cv_ptr->image.convertTo(data, CV_32F);
//	  cv::imshow("32BITIMAGE", data);

	//  cv::convertScaleAbs(cv_ptr->image, data);
	//  fabmap->addTraining(data);
//	  ROS_INFO("Running FabMap!%d",cv_ptr->image.rows);
	//  std::vector<cv::of2::IMatch> matches;
	  if((scene_frames%20)==0&&scene_frames!=0)
	  {
		  std::vector<cv::Mat>::iterator i;
		  for(i=traindatas.begin();i!=traindatas.end();i++)
			  treeBuilder.add(*i);
		  cv::Mat tree = treeBuilder.make();
		  cv::Ptr<cv::of2::FabMap> fabmap = new cv::of2::FabMap(tree,0.39,0,cv::of2::FabMap::MEAN_FIELD | cv::of2::FabMap::CHOW_LIU);
		  std::vector<cv::Mat>::iterator j;
		  for(j=traindatas.begin();j!=traindatas.end();j++)
			  fabmap->addTraining(*j);
		  std::vector<cv::of2::IMatch> matches;
		  fabmap->compare(data,matches,true);

	//	  ROS_INFO("Running FabMap!");
		  std::vector<cv::of2::IMatch>::iterator m;
		  double max_likelihood=0;
		  double max_match=0;
		  for(m=matches.begin();m!=matches.end();m++)
		  {
			  if(m->likelihood<max_likelihood)
				  max_likelihood=m->likelihood;
			  if(m->match>max_match)
			  	max_match=m->match;
		  }
		  ROS_INFO("Frame%d Max_likelihood:[%lf]Match:[%lf]",scene_frames,max_likelihood,max_match);
		  //display result
//		  if(scene_frames<5000)
//		  {
//			  cv::line(result_display, cv::Point(0,250), cv::Point(500,250),cv::Scalar(0,255,255), 1, CV_AA, 0);
//			  cv::line(result_display, cv::Point(scene_frames/10,500.0+max_likelihood*100.0*500.0), cv::Point(scene_frames/10,500.0),cv::Scalar(255,0,0), 1, CV_AA, 0);
//		  }
//		  cv::imshow("Confusion Matrix", result_display);
	  }
//	  treeBuilder.add(cv_ptr->image);
//	  tree = treeBuilder.make();
	  traindatas.push_back(data);

//	  fabmap->addTraining(data);

//	  max_likelihood=0;
//	      std::vector<cv::of2::IMatch>::iterator l;
//
//	      for(l = matches.begin(); l != matches.end(); l++) {
////	    	  cv::line(result_small, cv::Point(), pt2, color, thickness, lineType, shift)
//	              if(l->imgIdx < 0) {
//	                  result_small.at<char>(l->queryIdx, l->queryIdx) =
//	                      (char)(l->match*255);
//
//	              } else {
//	                  result_small.at<char>(l->queryIdx, l->imgIdx) =
//	                      (char)(l->match*255);
//	              }
//	      }
//
//	      cv::Mat result_large(100, 100, CV_8UC1);
//	      cv::resize(result_small, result_large, cv::Size(500, 500), 0, 0, CV_INTER_NN);
//	      cv::imshow("Confusion Matrix", result_large);
//	      cv::waitKey();
//	  cv::imshow("Confusion Matrix", result_display);
//	  ROS_INFO("Training Chow-liu tree");

//	  ROS_INFO("Add data");
	  scene_frames++;
	  data.release();
//	  tree = treeBuilder.make();
//	  ROS_INFO("Make");
//	  cv::waitKey(0);

}

