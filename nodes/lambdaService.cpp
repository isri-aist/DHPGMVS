#include "ros/console.h"
#include <dhpgmvs/diff_lambda.h>
#include <iostream>
#include <ros/ros.h>
#include <cstdlib>
#include <cstdio> 
#include <DifferentiableImage/DifferentiableImage.h>
#include <string>

DifferentiableImage diff_image;

bool callFunction(dhpgmvs::diff_lambda::Request &req, dhpgmvs::diff_lambda::Response &res) {
  std::cout << "Received request" << std::endl;
    res.lambda_g = diff_image.start(req.desired_image_path_right, req.init_image_path_right, req.mask, req.desired_image_path_left, req.init_image_path_left);
    //res.lambda_g = diff_image.compute();
    return true;
}

// bool callFunction(ros_dvs_bridge::diff_lambda::Request &req, ros_dvs_bridge::diff_lambda::Response &res)
// {
//   res.init_lambda_g = 0.1;
//   return true;
// }


int main(int argc, char **argv)
{
  std::cout << "Starting lambdaServiceDiff node" << std::endl;
  ros::init(argc, argv, "lambdaServiceDiff");
  std::cout << "Node initialized" << std::endl;
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("diff_lambda", callFunction);
  std::cout << "Service advertised" << std::endl;

  ros::spin();

  return 0;
}
