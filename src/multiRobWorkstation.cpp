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
	std::vector<cv::of2::IMatch> matches;
	cv::of2::ChowLiuTree treeBuilder;
//	cv::of2::FabMap2 fabmap;

    public:
	multiRobWorkstation();
	~multiRobWorkstation();
    void sceneNodeCallback(const multi_robot_slam::Scenenode::ConstPtr& msg);
};

multiRobWorkstation::multiRobWorkstation()
{

	sceneNode_sub = n.subscribe("/image_converter/scene_node", 1, &multiRobWorkstation::sceneNodeCallback,this);
//	map_sub = n.subscribe("/map", 1, &sceneNodeCallback);


}

multiRobWorkstation::~multiRobWorkstation()
{

}

void multiRobWorkstation::sceneNodeCallback(const multi_robot_slam::Scenenode::ConstPtr& msg)
{
  ROS_INFO("I heard sceneNode message: [%lf]", msg->px);
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

  //create Chou-Liu tree
  ROS_INFO("Training Chow-liu tree");
//  cv::of2::ChowLiuTree treeBuilder;

  treeBuilder.add(cv_ptr->image);
  ROS_INFO("Add data");
  cv::Mat tree = treeBuilder.make();
  ROS_INFO("Make");
//  cv::FileStorage fs("tree.yml",cv::FileStorage::WRITE);
//  ROS_INFO("Open file");
//  fs<<"tree"<<tree;
//  ROS_INFO("writting");
//  fs.release();
//  fs.writeObj("tree", tree);

  //run fabmap
  ROS_INFO("Running FabMap!");
  cv::Ptr<cv::of2::FabMap> fabmap;
  fabmap = new cv::of2::FabMap(tree,0.39,0,cv::of2::FabMap::SAMPLED | cv::of2::FabMap::CHOW_LIU);
  ROS_INFO("Running FabMap!");
//  cv::imshow("trainImage", cv_ptr->image);
//  cv::waitKey();
//  cv::Mat data(cv_ptr->image.rows,cv_ptr->image.cols,CV_32F);
  cv::Mat data;
  cv_ptr->image.convertTo(data, CV_32F);
//  cv::convertScaleAbs(cv_ptr->image, data);
  fabmap->addTraining(data);
  ROS_INFO("Running FabMap!%d",cv_ptr->image.rows);
//  std::vector<cv::of2::IMatch> matches;
  fabmap->compare(data,matches,true);
  ROS_INFO("Running FabMap!");
  std::vector<cv::of2::IMatch>::iterator l;
  for(l=matches.begin();l!=matches.end();l++)
	  ROS_INFO("match probilities:[%lf]",l->match);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "worstation");
  multiRobWorkstation mk;
  ros::spin();
  return 0;
}
