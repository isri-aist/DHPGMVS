/*
 * DHPGMVS: Dual-Hemispherical Photometric Gaussian Mixtures-based Visual
 * Servoing Authors: Guillaume Caron (guillaume.caron@u-picardie.fr) Nathan
 * Crombez (nathan.crombez@utbm.fr) Institutions: CNRS-AIST JRL / UTBM CIAD
 * Date: April 2024, January 2024
 */

////ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

////BOOST
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

////ViSP
#include <visp/vpDisplayX.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>
#include <visp/vpPlot.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
////libPeR
// Camera type considered
#include <per/prOmni.h>
// Acquisition model
#include <per/prRegularlySampledCPImage.h>
// Visual features
#include <per/prFeaturesSet.h>
#include <per/prPhotometricnnGMS.h>
#include <per/prSSDCmp.h>
// Generic Visual Servo Control tools
#include <per/prCameraPoseEstim.h>
#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

////etc. (evs/dynamic reconfigure)
#include <dhpgmvs/lambda_gConfig.h>
#include <dynamic_reconfigure/server.h>
#include <filesystem>
#include <image_transport/image_transport.h>
#define MAX_STAGNATION_ITER 10
#include <filesystem>
#include <signal.h>
#include <std_msgs/Float32.h>

#include "dhpgmvs/diff_lambda.h"
#include <iostream>
#include <sstream>

////ROBOT
ros::Publisher robotVelocityPub, robotPosePub;

////CAMERA
typedef struct {
  int height, width;
  /*float au, av;
  float u0, v0;
  float xi;*/
  prOmni cam;
  vpHomogeneousMatrix cMf;
} cameraParameters;
cameraParameters rightCameraParameters, leftCameraParameters;

////IMAGES
vpImage<unsigned char> rightI, leftI, rightId, leftId, rightIdiff, leftIdiff;

////DISPLAYS
vpDisplayX rightDispI, leftDispI, rightDispId, leftDispId, rightDispIdiff,
    leftDispIdiff;
vpPlot plot;

////VISUAL SERVOING
vpPoseVector desiredRobotPose, currentRobotPose, initialRobotPose;
vpMatrix L, L_right, L_left;
vpColVector e, e_right, e_left, v;
double Z;
int iter;
double gain;
bool vsStarted;

// libPeR's desired visual features for visual servoing
prRegularlySampledCPImage<unsigned char> *IP_des_right, *IP_des_left;
prRegularlySampledCPImage<float> *GP_right, *GP_left;
prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>,
              prRegularlySampledCPImage>
    fSet_des_right, fSet_des_left;
prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample_des_right,
    *GP_sample_des_left;

// libPeR's current visual features for visual servoing
prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample_right, *GP_sample_left;
prRegularlySampledCPImage<unsigned char> *IP_cur_right, *IP_cur_left;
prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>,
              prRegularlySampledCPImage>
    fSet_cur_right, fSet_cur_left;
prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample_cur_right,
    *GP_sample_cur_left;

// libPeR's visual servoing tasks (will be merged Crombez' style)
prCameraPoseEstim<
    prFeaturesSet<prCartesian2DPointVec,
                  prPhotometricnnGMS<prCartesian2DPointVec>,
                  prRegularlySampledCPImage>,
    prSSDCmp<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>>>
    servo_right, servo_left;

// other LibPeR's variables
double lambda_g;
bool updateSampler;
bool poseJacobianCompute;
bool robust; // to activate the M-Estimator

// for robots controlled in the base frame and publishing their effector pose as
// a tf2
vpVelocityTwistMatrix bVt;
std::mutex mutex_bVt;
bool controlInBaseFrame;
ros::Subscriber camPose_sub;
void toolPoseCallback(const tf2_msgs::TFMessage &tf);
vpHomogeneousMatrix toVispHomogeneousMatrix(const tf2_msgs::TFMessage &trans);

////OTHERS
ros::Time t;
bool verbose;
int qSize;
void stopProgramm();

// evs
bool rosbagForEVS;
std::string desiredPoseTopicForEVS;
std::mutex mutex_bMt;
std::mutex mutex_bVc;
vpHomogeneousMatrix m_bMt;
vpHomogeneousMatrix m_tMc;
vpHomogeneousMatrix m_bMc;
vpVelocityTwistMatrix m_bVc;
rosbag::Bag vsbag;
std::string currentPoseTopicForEVS;
std::string logs_path;
std::string bagFilePath;

// dynamic reconfigure and twoStepVS and saveExperimentData
std::mutex lambda_mutex;
bool twoStepVS;
bool flagSecondStepVS;
double DIFF_VELO = 0;
double RESIDUAL_THRESHOLD = 0;
double test = 0;
double lastVelocities[MAX_STAGNATION_ITER];
int stagnationCount;
int iterCount;
void updateParameters(double new_lambda_g, double new_gain, double new_Z);
void callbackServer(dhpgmvs::lambda_gConfig &config, uint32_t level);
bool saveExperimentData;
void copyData();

// rqt and other topics
ros::ServiceClient client;
ros::ServiceServer service;
image_transport::Publisher
    left_cur_feat_pub; // current left image with GaussianM.
image_transport::Publisher
    right_cur_feat_pub;                   // current right image with GaussianM.
image_transport::Publisher left_des_pub;  // desired left image
image_transport::Publisher right_des_pub; // desired right image
image_transport::Publisher
    left_des_feat_pub; // desired left image with GaussianM.
image_transport::Publisher
    right_des_feat_pub;                   // desired right image with GaussianM.
image_transport::Publisher left_diff_pub; // difference in images left
image_transport::Publisher right_diff_pub; // difference in images right
image_transport::Publisher
    left_diff_feat_pub; // difference in images left with GaussianM.
image_transport::Publisher
    right_diff_feat_pub; // difference in images right with GaussianM.
vpImage<float> left_des_feat_float, right_des_feat_float, left_cur_feat_float,
    right_cur_feat_float;
vpImage<unsigned char> left_des_feat_char, right_des_feat_char,
    left_cur_feat_char, right_cur_feat_char, left_diff_feat_char,
    right_diff_feat_char;
vpPoseVector pp;
void publish4rqt(const vpImage<unsigned char> &image,
                 const image_transport::Publisher &pub);
template <typename F, typename C>
void resizeAndConvert4rqt(vpImage<float> &feat_float,
                          vpImage<unsigned char> &feat_char,
                          const vpImage<unsigned char> &image, F &fSet, C &cam);
template <typename F, typename C>
void convertAndPublish4rqt(F &fSet, vpImage<float> &feat_float,
                           vpImage<unsigned char> &feat_char, C &cam,
                           const vpImage<unsigned char> &image,
                           const image_transport::Publisher &pub);
bool pub_cost;
ros::Publisher cost_pub;
std_msgs::Float32 cost;
std::string costTopic;
std::ofstream residuals;
std::ofstream velocities;
std::stringstream string;
void mySigintHandler(int sig);

////Main VS loop (callback)
void camerasImageRobotPoseCallback(
    const sensor_msgs::Image::ConstPtr &rightImsg,
    const sensor_msgs::Image::ConstPtr &leftImsg,
    const geometry_msgs::PoseStamped::ConstPtr &robotPoseMsg, double diff,
    double tresh);
////DHPGM error
void computeDHPGMErrorVector(vpColVector &e_right, vpColVector &e_left,
                             vpColVector &e);
////DHPGM interaction matrix
void computeDHPGMInteractionMatrix(vpMatrix &L_right,
                                   cameraParameters rightCamParam,
                                   vpMatrix &L_left,
                                   cameraParameters leftCamParam, vpMatrix &L);

void initVisualServoTasks();

////Camera's poses initialization
void cameraPosesInitialization();

////VISP <--> ROS Messages
vpHomogeneousMatrix vpHomogeneousMatrixFromROSTransform(std::string frame_i,
                                                        std::string frame_o);
geometry_msgs::Twist geometryTwistFromvpColVector(vpColVector vpVelocity);
cameraParameters cameraInfoToCameraParam(sensor_msgs::CameraInfo msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "DHPGMVS", ros::init_options::NoSigintHandler);
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh("~");

  signal(SIGINT, mySigintHandler);

  qSize = 30;
  vsStarted = false;
  v.resize(6);

  updateSampler = true;
  poseJacobianCompute = true;

  ////Get & set parameters
  verbose = nh.param("verbose", 1);
  gain = nh.param("gain", 1.0);
  Z = nh.param("Z", 1.0);
  lambda_g = nh.param("lambda_g", 1.0);
  robust = nh.param("robust", false);
  nh.param("twoStepVS", twoStepVS, bool(false));
  nh.getParam("twoStepVS", twoStepVS);
  flagSecondStepVS = false;
  nh.param("costTopic", costTopic, std::string(""));
  nh.getParam("costTopic", costTopic);
  DIFF_VELO = 1e-3;
  RESIDUAL_THRESHOLD = 1e-4;
  nh.param("saveExperimentData", saveExperimentData, bool(false));
  nh.getParam("saveExperimentData", saveExperimentData);
  nh.param("logs", logs_path, std::string(""));
  nh.getParam("logs", logs_path);

  ////rqt
  image_transport::ImageTransport it(nh);
  left_des_pub = it.advertise("left_des", 1);
  right_des_pub = it.advertise("right_des", 1);
  left_diff_pub = it.advertise("left_diff", 1);
  right_diff_pub = it.advertise("right_diff", 1);
  left_cur_feat_pub = it.advertise("left_cur_feat", 1);
  right_cur_feat_pub = it.advertise("right_cur_feat", 1);
  left_des_feat_pub = it.advertise("left_des_feat", 1);
  right_des_feat_pub = it.advertise("right_des_feat", 1);
  left_diff_feat_pub = it.advertise("left_diff_feat", 1);
  right_diff_feat_pub = it.advertise("right_diff_feat", 1);

  string.str("");
  string << logs_path << "/residuals.txt";
  residuals.open(string.str().c_str());

  string.str("");
  string << logs_path << "/velocities.txt";
  velocities.open(string.str().c_str());

  dynamic_reconfigure::Server<dhpgmvs::lambda_gConfig> server(nh);
  dynamic_reconfigure::Server<dhpgmvs::lambda_gConfig>::CallbackType f;

  ////Get right and left cameras intrinsic parameters
  rightCameraParameters = cameraInfoToCameraParam(
      *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
          "/dhpgmvs/right_camera/camera_info", nh));
  leftCameraParameters = cameraInfoToCameraParam(
      *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
          "/dhpgmvs/left_camera/camera_info", nh));
  ////Get right and left cameras extrinsic parameters
  rightCameraParameters.cMf =
      vpHomogeneousMatrixFromROSTransform("/flange", "/right_camera");
  leftCameraParameters.cMf =
      vpHomogeneousMatrixFromROSTransform("/flange", "/left_camera");

  ////Robot velocities publisher
  robotVelocityPub =
      nh.advertise<geometry_msgs::Twist>("/dhpgmvs/robot/set_velocity", 1);

  ////Cameras and robot synchronizer
  message_filters::Subscriber<sensor_msgs::Image> rightCameraSub, leftCameraSub;
  message_filters::Subscriber<geometry_msgs::PoseStamped> robotPoseSub;
  rightCameraSub.subscribe(nh, "/dhpgmvs/right_camera/image_raw", qSize);
  leftCameraSub.subscribe(nh, "/dhpgmvs/left_camera/image_raw", qSize);
  robotPoseSub.subscribe(nh, "/dhpgmvs/robot/get_pose", qSize);
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped>>
      camerasSynchronizer(message_filters::sync_policies::ApproximateTime<
                              sensor_msgs::Image, sensor_msgs::Image,
                              geometry_msgs::PoseStamped>(qSize),
                          rightCameraSub, leftCameraSub, robotPoseSub);
  camerasSynchronizer.registerCallback(
      boost::bind(&camerasImageRobotPoseCallback, _1, _2, _3,
                  boost::ref(DIFF_VELO), boost::ref(RESIDUAL_THRESHOLD)));

  // to get UR pose from tf
  std::string cameraPoseTopic;
  nh.param("cameraPoseTopic", cameraPoseTopic, std::string(""));
  nh.param("controlInBaseFrame", controlInBaseFrame, false);
  nh.param("rosbagForEVS", rosbagForEVS, false);
  nh.param("desiredPoseTopicForEVS", desiredPoseTopicForEVS, std::string(""));
  nh.param("currentPoseTopicForEVS", currentPoseTopicForEVS, std::string(""));

  nh.getParam("controlInBaseFrame", controlInBaseFrame);
  nh.getParam("rosbagForEVS", rosbagForEVS);
  nh.getParam("desiredPoseTopicForEVS", desiredPoseTopicForEVS);
  nh.getParam("currentPoseTopicForEVS", currentPoseTopicForEVS);

  if (controlInBaseFrame || rosbagForEVS) {
    nh.getParam("cameraPoseTopic", cameraPoseTopic);

    camPose_sub = nh.subscribe(cameraPoseTopic, 1, &toolPoseCallback);
  }

  if (costTopic.compare("") != 0) {
    cost_pub = nh.advertise<std_msgs::Float32>(costTopic, 1);
    pub_cost = true;
  }

  ////Image displayers
  rightI.resize(rightCameraParameters.height, rightCameraParameters.width);
  leftI.resize(leftCameraParameters.height, leftCameraParameters.width);
  rightId.resize(rightCameraParameters.height, rightCameraParameters.width);
  leftId.resize(leftCameraParameters.height, leftCameraParameters.width);
  rightIdiff.resize(rightCameraParameters.height, rightCameraParameters.width);
  leftIdiff.resize(leftCameraParameters.height, leftCameraParameters.width);
  leftDispI.init(leftI, 0, 27, "LEFT I");
  leftDispId.init(leftId, 0, 27 + 37 + leftCameraParameters.height, "LEFT I*");
  leftDispIdiff.init(leftIdiff, 0, 27 + (37 + leftCameraParameters.height) * 2,
                     "LEFT DIFF");
  rightDispI.init(rightI, rightCameraParameters.width, 27, "RIGHT I");
  rightDispId.init(rightId, rightCameraParameters.width,
                   27 + 37 + rightCameraParameters.height, "RIGHT I*");
  rightDispIdiff.init(rightIdiff, rightCameraParameters.width,
                      27 + (37 + rightCameraParameters.height) * 2,
                      "RIGHT DIFF");

  ////Plots
  plot.init(4, (37 + rightCameraParameters.height) * 3 - 37,
            rightCameraParameters.width, rightCameraParameters.width * 2, 27);
  plot.initGraph(0, 1);
  plot.setTitle(0, "Feature error");
  plot.setLegend(0, 0, "error");
  plot.initGraph(1, 6);
  plot.setTitle(1, "Velocities");
  plot.setLegend(1, 0, "v_x");
  plot.setLegend(1, 1, "v_y");
  plot.setLegend(1, 2, "v_z");
  plot.setLegend(1, 3, "w_x");
  plot.setLegend(1, 4, "w_y");
  plot.setLegend(1, 5, "w_z");
  plot.initGraph(2, 3);
  plot.setTitle(2, "Translation error");
  plot.setLegend(2, 0, "dtx_x");
  plot.setLegend(2, 1, "dt_y");
  plot.setLegend(2, 2, "dt_z");
  plot.initGraph(3, 3);
  plot.setTitle(3, "Orientation error");
  plot.setLegend(3, 0, "dw_x");
  plot.setLegend(3, 1, "dw_y");
  plot.setLegend(3, 2, "dw_z");

  ////Camera desired and initial pose initialization
  cameraPosesInitialization();

  f = boost::bind(&callbackServer, _1, _2);
  server.setCallback(f);

  client = nh.serviceClient<dhpgmvs::diff_lambda>("/diff_lambda");
  dhpgmvs::diff_lambda srv;
  srv.request.desired_image_path_right = logs_path + "/Id_des_right.png";
  srv.request.init_image_path_right = logs_path + "/Id_init_right.png";
  srv.request.mask = logs_path + "/mask_92x92.png"; //change path to mask 
  srv.request.desired_image_path_left = logs_path + "/Id_des_left.png";
  srv.request.init_image_path_left = logs_path + "/Id_init_left.png";

  if (client.call(srv)) {
    double initial_lambda_g = srv.response.lambda_g;
    std::cout << "------------------------------lambda_g: " << initial_lambda_g
              << std::endl;
    updateParameters(initial_lambda_g, gain, Z);
  } else {
    ROS_ERROR("Failed to call service diff_lambda");
  }

  ROS_WARN("initVisualServoTasks");
  ////libPeR's objects initializations
  initVisualServoTasks();

  ROS_WARN("initVisualServoTasks done");
  ////Actually start DHPGMVS
  iter = 0;
  vsStarted = true;
  t = ros::Time::now();
  ros::spin();
  return 0;
}

void cameraPosesInitialization() {
  for (int i = 3; i > 0; i--) {
    ROS_WARN("%d", i);
    sleep(1);
  }

  ROS_WARN("Move the camera to the desired pose then click on LEFT I");
  do {
    rightId = rightI;
    leftId = leftI;
    vpDisplay::display(rightId);
    vpDisplay::flush(rightId);
    vpDisplay::display(leftId);
    vpDisplay::flush(leftId);
    ros::spinOnce();
  } while (!vpDisplay::getClick(leftI, false));
  desiredRobotPose = currentRobotPose;

  vpImageIo::write(rightId,
                   logs_path + "/Id_des_right.png"); // store the desired image
  vpImageIo::write(leftId, logs_path + "/Id_des_left.png");

  if (rosbagForEVS) {
    geometry_msgs::PoseStamped desiredRobotPoseStamped;
    // vpHomogeneousMatrix bMc;
    // mutex_bMt.lock();
    // bMc = m_bMt * m_tMc;
    // mutex_bMt.unlock();
    vpTranslationVector t = m_bMt.getTranslationVector();
    vpQuaternionVector q = vpQuaternionVector(m_bMt.getRotationMatrix());

    desiredRobotPoseStamped.header.stamp = ros::Time::now();
    desiredRobotPoseStamped.pose.position.x = t[0];
    desiredRobotPoseStamped.pose.position.y = t[1];
    desiredRobotPoseStamped.pose.position.z = t[2];
    desiredRobotPoseStamped.pose.orientation.x = q.x();
    desiredRobotPoseStamped.pose.orientation.y = q.y();
    desiredRobotPoseStamped.pose.orientation.z = q.z();
    desiredRobotPoseStamped.pose.orientation.w = q.w();

    std::stringstream ss_bag;
    ss_bag << logs_path << "/desiredAndCurrentPoses.bag";
    vsbag.open(ss_bag.str().c_str(), rosbag::bagmode::Write);
    vsbag.write(desiredPoseTopicForEVS, ros::Time::now(),
                desiredRobotPoseStamped);
    vsbag.close();
  }

  ROS_WARN("Move the camera to the initial pose then click on LEFT I");
  do {
    vpImageTools::imageDifference(leftI, leftId, leftIdiff);
    vpImageTools::imageDifference(rightI, rightId, rightIdiff);
    vpDisplay::display(leftI);
    vpDisplay::flush(leftI);
    vpDisplay::display(rightI);
    vpDisplay::flush(rightI);
    vpDisplay::display(leftIdiff);
    vpDisplay::flush(leftIdiff);
    vpDisplay::display(rightIdiff);
    vpDisplay::flush(rightIdiff);
    ros::spinOnce();
  } while (!vpDisplay::getClick(leftI, false));
  initialRobotPose = currentRobotPose;

  vpImageIo::write(rightI,
                   logs_path + "/Id_init_right.png"); // store the initial image
  vpImageIo::write(leftI, logs_path + "/Id_init_left.png");

  if (rosbagForEVS) {
    std::stringstream ss_bag, dst_bag;
    ss_bag << logs_path << "/desiredAndCurrentPoses.bag";
    dst_bag << logs_path << "/desiredAndCurrentPoses"
            << ((unsigned)ros::Time::now().toSec()) << "dhpgm.bag";
    std::filesystem::copy(ss_bag.str().c_str(), dst_bag.str().c_str());
    bagFilePath = dst_bag.str().c_str();
    vsbag.open(dst_bag.str().c_str(), rosbag::bagmode::Append);
  }

  ROS_WARN("Cick on LEFT I to start VS");
  do {
    ros::spinOnce();
  } while (!vpDisplay::getClick(leftI, false));
}

void initVisualServoTasks() {
  ////RIGHT CAMERA
  // 2. VS objects initialization, considering the pose control of a perspective
  // camera from the feature set of photometric non-normalized Gaussian mixture
  // 2D samples compared thanks to the SSD
  servo_right.setdof(true, true, true, true, true, true);
  servo_right.setSensor(&(rightCameraParameters.cam));

  // desired visual feature built from the image
  // prepare the desired image
  IP_des_right = new prRegularlySampledCPImage<unsigned char>(
      rightCameraParameters.height,
      rightCameraParameters
          .width); // the regularly sample planar image to be set from the
                   // acquired/loaded perspective image
  IP_des_right->setInterpType(INTERPTYPE);
  IP_des_right->buildFrom(rightId, &(rightCameraParameters.cam));

  GP_right = new prRegularlySampledCPImage<float>(
      rightCameraParameters.height,
      rightCameraParameters.width); // contient tous les pr2DCartesianPointVec
                                    // (ou prFeaturePoint) u_g et fera
                                    // GS_sample.buildFrom(IP_des, u_g);

  GP_sample_des_right = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

  fSet_des_right.buildFrom(*IP_des_right, *GP_right, *GP_sample_des_right,
                           false, true); // Goulot !

  servo_right.buildFrom(fSet_des_right);

  servo_right.initControl(gain, Z);

  // current visual feature built from the image
  GP_sample_right = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

  // Current features set setting from the current image
  IP_cur_right = new prRegularlySampledCPImage<unsigned char>(
      rightCameraParameters.height, rightCameraParameters.width);
  IP_cur_right->setInterpType(INTERPTYPE);
  IP_cur_right->buildFrom(rightI, &(rightCameraParameters.cam));

  GP_sample_cur_right = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

  fSet_cur_right.buildFrom(*IP_cur_right, *GP_right, *GP_sample_right,
                           poseJacobianCompute, updateSampler); // Goulot !

  ////LEFT CAMERA
  // 2. VS objects initialization, considering the pose control of a perspective
  // camera from the feature set of photometric non-normalized Gaussian mixture
  // 2D samples compared thanks to the SSD
  servo_left.setdof(true, true, true, true, true, true);
  servo_left.setSensor(&(leftCameraParameters.cam));

  // desired visual feature built from the image
  // prepare the desired image
  IP_des_left = new prRegularlySampledCPImage<unsigned char>(
      leftCameraParameters.height,
      leftCameraParameters
          .width); // the regularly sample planar image to be set from the
                   // acquired/loaded perspective image
  IP_des_left->setInterpType(INTERPTYPE);
  IP_des_left->buildFrom(leftId, &(leftCameraParameters.cam));

  GP_left = new prRegularlySampledCPImage<float>(
      leftCameraParameters.height,
      leftCameraParameters.width); // contient tous les pr2DCartesianPointVec
                                   // (ou prFeaturePoint) u_g et fera
                                   // GS_sample.buildFrom(IP_des, u_g);

  GP_sample_des_left = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

  fSet_des_left.buildFrom(*IP_des_left, *GP_left, *GP_sample_des_left, false,
                          true); // Goulot !

  servo_left.buildFrom(fSet_des_left);

  servo_left.initControl(gain, Z);

  // current visual feature built from the image
  GP_sample_left = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

  // Current features set setting from the current image
  IP_cur_left = new prRegularlySampledCPImage<unsigned char>(
      leftCameraParameters.height, leftCameraParameters.width);
  IP_cur_left->setInterpType(INTERPTYPE);
  IP_cur_left->buildFrom(leftI, &(leftCameraParameters.cam));

  GP_sample_cur_left = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

  fSet_cur_left.buildFrom(*IP_cur_left, *GP_left, *GP_sample_left,
                          poseJacobianCompute, updateSampler); // Goulot !

  ////rqt
  resizeAndConvert4rqt(left_des_feat_float, left_des_feat_char, leftI,
                       fSet_des_left, leftCameraParameters.cam);
  resizeAndConvert4rqt(right_des_feat_float, right_des_feat_char, rightI,
                       fSet_des_right, rightCameraParameters.cam);
  resizeAndConvert4rqt(left_cur_feat_float, left_cur_feat_char, leftI,
                       fSet_cur_left, leftCameraParameters.cam);
  resizeAndConvert4rqt(right_cur_feat_float, right_cur_feat_char, rightI,
                       fSet_cur_right, rightCameraParameters.cam);
}

void camerasImageRobotPoseCallback(
    const sensor_msgs::Image::ConstPtr &rightImsg,
    const sensor_msgs::Image::ConstPtr &leftImsg,
    const geometry_msgs::PoseStamped::ConstPtr &robotPoseMsg, double diff,
    double tresh) {
  ////ROS to VISP
  rightI = visp_bridge::toVispImage(*rightImsg);
  leftI = visp_bridge::toVispImage(*leftImsg);
  currentRobotPose =
      vpPoseVector(visp_bridge::toVispHomogeneousMatrix(robotPoseMsg->pose));
  lastVelocities[MAX_STAGNATION_ITER] = {0};
  stagnationCount = 0;
  iterCount = 0;

  if (vsStarted) {
    if (robotPoseMsg->header.stamp <= t) // Prevent ROS synchro issues
      return;

    ////Displays
    vpImageTools::imageDifference(rightI, rightId, rightIdiff);
    vpImageTools::imageDifference(leftI, leftId, leftIdiff);

    vpDisplay::display(leftI);
    vpDisplay::flush(leftI);
    vpDisplay::display(rightI);
    vpDisplay::flush(rightI);
    vpDisplay::display(leftIdiff);
    vpDisplay::flush(leftIdiff);
    vpDisplay::display(rightIdiff);
    vpDisplay::flush(rightIdiff);

    ////rqt
    publish4rqt(leftId, left_des_pub);
    publish4rqt(rightId, right_des_pub);
    publish4rqt(leftIdiff, left_diff_pub);
    publish4rqt(rightIdiff, right_diff_pub);
    publish4rqt(left_des_feat_char, left_des_feat_pub);
    publish4rqt(right_des_feat_char, right_des_feat_pub);
    convertAndPublish4rqt(fSet_cur_left, left_cur_feat_float,
                          left_cur_feat_char, leftCameraParameters.cam,
                          left_cur_feat_char, left_cur_feat_pub);
    convertAndPublish4rqt(fSet_cur_right, right_cur_feat_float,
                          right_cur_feat_char, rightCameraParameters.cam,
                          right_cur_feat_char, right_cur_feat_pub);

    vpImageTools::imageDifference(left_cur_feat_char, left_des_feat_char,
                                  left_diff_feat_char);
    publish4rqt(left_diff_feat_char, left_diff_feat_pub);

    vpImageTools::imageDifference(right_cur_feat_char, right_des_feat_char,
                                  right_diff_feat_char);
    publish4rqt(right_diff_feat_char, right_diff_feat_pub);

    ////Update left and right camera features
    IP_cur_right->buildFrom(rightI, &(rightCameraParameters.cam));
    fSet_cur_right.updateMeasurement(*IP_cur_right, *GP_right,
                                     *GP_sample_cur_right, poseJacobianCompute,
                                     updateSampler);
    servo_right.interactionAndError(fSet_cur_right, L_right, e_right, robust);

    IP_cur_left->buildFrom(leftI, &(leftCameraParameters.cam));
    fSet_cur_left.updateMeasurement(*IP_cur_left, *GP_left, *GP_sample_cur_left,
                                    poseJacobianCompute, updateSampler);
    servo_left.interactionAndError(fSet_cur_left, L_left, e_left, robust);

    ////Compute interaction matrix
    computeDHPGMInteractionMatrix(L_right, rightCameraParameters, L_left,
                                  leftCameraParameters, L);
    ////Compute error vector
    computeDHPGMErrorVector(e_right, e_left, e);
    ////Compute Gauss-newton control law
    v = gain * L.pseudoInverseEigen3() *
        e; // Velocities expressed in the robot's flange... or their opposite?..

    ////-------------checking extrinsic camera calibration (slow velocity
    ///commands in camera frame)
    // v = 0;
    // v[2] = 0.005;
    // vpVelocityTwistMatrix cVf_left(leftCameraParameters.cMf.inverse());
    // //here for left camera v = cVf_left * v; // Transform velocity to camera
    // frame
    ////--------------------------------------------------

    if (controlInBaseFrame) {
      mutex_bVt.lock();
      v = bVt * v;
      mutex_bVt.unlock();
    }

    // rosbagForEVS
    if (rosbagForEVS) {
      // vpHomogeneousMatrix bMc;
      // mutex_bMt.lock();
      // bMc = m_bMt * m_tMc;
      // mutex_bMt.unlock();

      geometry_msgs::PoseStamped currentRobotPoseStamped;
      vpTranslationVector t = m_bMt.getTranslationVector();
      vpQuaternionVector q = vpQuaternionVector(m_bMt.getRotationMatrix());

      currentRobotPoseStamped.header.stamp = ros::Time::now();
      currentRobotPoseStamped.pose.position.x = t[0];
      currentRobotPoseStamped.pose.position.y = t[1];
      currentRobotPoseStamped.pose.position.z = t[2];
      currentRobotPoseStamped.pose.orientation.x = q.x();
      currentRobotPoseStamped.pose.orientation.y = q.y();
      currentRobotPoseStamped.pose.orientation.z = q.z();
      currentRobotPoseStamped.pose.orientation.w = q.w();

      vsbag.write(currentPoseTopicForEVS, ros::Time::now(),
                  currentRobotPoseStamped);
    }

    if (twoStepVS) {
      lastVelocities[iterCount % MAX_STAGNATION_ITER] = sqrt(v.sumSquare());
      iterCount++;

      bool stagnant = true;
      for (int i = 1; i < MAX_STAGNATION_ITER; ++i) {
        if (fabs(lastVelocities[i] - lastVelocities[i - 1]) > DIFF_VELO) {
          stagnant = false;
          std::cout << fabs(lastVelocities[i] - lastVelocities[i - 1])
                    << std::endl;
          break;
        }
      }

      std::cout << "stagant : " << stagnant << std::endl;
      std::cout << "velo : " << sqrt(v.sumSquare()) << std::endl;
      std::cout << "flagSecondStepVS : " << flagSecondStepVS << std::endl;
      std::cout << "RESIDUAL_THRESHOLD : " << tresh << std::endl;
      std::cout << "DIFF_VELO : " << diff << std::endl;

      if (stagnant && sqrt(v.sumSquare()) < RESIDUAL_THRESHOLD &&
          !flagSecondStepVS) {
        std::cout << "--------------- Initializing second step of vs "
                     "-------------------------"
                  << std::endl;
        // stopRobot();
        double new_lambda_g = 1; // updating parameters for 2. step
        double new_gain = 0.5;
        updateParameters(new_lambda_g, new_gain, Z);
        flagSecondStepVS = true; // set flag true
        return;
      }
    }

    if (pub_cost) {
      cost.data = e.sumSquare();
      cost_pub.publish(cost);
    }

    ////Send velocity to robot
    robotVelocityPub.publish(geometryTwistFromvpColVector(v));

    t = ros::Time::now();

    ////PLOTS
    plot.plot(0, 0, iter, e.sumSquare());
    for (int i = 0; i < 3; i++)
      plot.plot(2, i, iter, currentRobotPose[i] - desiredRobotPose[i]);
    for (int i = 0; i < 3; i++)
      plot.plot(3, i, iter,
                vpMath::deg(currentRobotPose[i + 3] - desiredRobotPose[i + 3]));
    for (int i = 0; i < 6; i++)
      plot.plot(1, i, iter, v[i]);

    ////VERBOSE
    if (verbose) {
      ROS_DEBUG("Iteration: %d", iter);
      ROS_DEBUG("Velocities: %f %f %f %f %f %f", v[0], v[1], v[2], v[3], v[4],
                v[5]);
      ROS_DEBUG("Current Pose: %f %f %f %f %f %f", currentRobotPose[0],
                currentRobotPose[1], currentRobotPose[2], currentRobotPose[3],
                currentRobotPose[4], currentRobotPose[5]);
      ROS_DEBUG("Desired Pose: %f %f %f %f %f %f", desiredRobotPose[0],
                desiredRobotPose[1], desiredRobotPose[2], desiredRobotPose[3],
                desiredRobotPose[4], desiredRobotPose[5]);
      ROS_DEBUG("Error Pose: %f %f %f %f %f %f",
                fabs(currentRobotPose[0] - desiredRobotPose[0]),
                fabs(currentRobotPose[1] - desiredRobotPose[1]),
                fabs(currentRobotPose[2] - desiredRobotPose[2]),
                vpMath::deg(fabs(currentRobotPose[3] - desiredRobotPose[3])),
                vpMath::deg(fabs(currentRobotPose[4] - desiredRobotPose[4])),
                vpMath::deg(fabs(currentRobotPose[5] - desiredRobotPose[5])));
      ROS_DEBUG("Photometric error: %f", e.sumSquare());
    }

    ////logs
    residuals << e.sumSquare() << std::endl;
    velocities << v << std::endl;

    iter++;
  }
}

void computeDHPGMErrorVector(vpColVector &e_right, vpColVector &e_left,
                             vpColVector &e) {
  e.resize(e_right.getRows() + e_left.getRows());

  ////RIGHT CAMERA
  e.insert(0, e_right);

  ////LEFT CAMERA
  e.insert(e_right.getRows(), e_left);
}

void computeDHPGMInteractionMatrix(vpMatrix &L_right,
                                   cameraParameters rightCamParam,
                                   vpMatrix &L_left,
                                   cameraParameters leftCamParam, vpMatrix &L) {
  L.clear();
  L.resize(L_right.getRows() + L_left.getRows(), 6);
  vpVelocityTwistMatrix V;

  ////RIGHT CAMERA
  V.buildFrom(rightCamParam.cMf.getRotationMatrix());

  L.insert(L_right * V, 0, 0);

  ////LEFT CAMERA
  V.buildFrom(leftCamParam.cMf.getRotationMatrix());

  L.insert(L_left * V, L_right.getRows(), 0);
}

void toolPoseCallback(const tf2_msgs::TFMessage &tf) {
  if (tf.transforms[0].child_frame_id.compare("tool0_controller") == 0) {
    vpHomogeneousMatrix bMc = /*visp_bridge::*/ toVispHomogeneousMatrix(tf);
    bMc[0][3] = bMc[1][3] = bMc[2][3] = 0;

    // //m_logfile << bMc << std::endl;
    mutex_bVt.lock();
    bVt.buildFrom(bMc);
    mutex_bVt.unlock();
    // }
    // 	if(tf.transforms[0].child_frame_id.compare("tool0_controller") == 0)
    // {
    mutex_bMt.lock();
    m_bMt = /*visp_bridge::*/ toVispHomogeneousMatrix(tf);
    mutex_bMt.unlock();

    //   vpHomogeneousMatrix bMt = m_bMt;
    //   bMt[0][3] = bMt[1][3] = bMt[2][3] = 0;

    //   //m_logfile << bMc << std::endl;
    //   mutex_bVc.lock();
    //   m_bVc.buildFrom(bMt);
    //   mutex_bVc.unlock();
  }
}

void updateParameters(double new_lambda_g, double new_gain, double new_Z) {

  if (new_lambda_g != lambda_g) {
    const std::lock_guard<std::mutex> lock(lambda_mutex);
    std::cout << "Updating lambda_g from " << lambda_g << " to " << new_lambda_g
              << std::endl;
    lambda_g = new_lambda_g;
    vsStarted = false;
    initVisualServoTasks();
    // vsStarted = true;
  }

  if (new_gain != gain) {
    std::cout << "Updating gain from " << gain << " to " << new_gain
              << std::endl;
    gain = new_gain;
    vsStarted = false;
    initVisualServoTasks();
  }

  if (new_Z != Z) {
    std::cout << "Updating Z from " << Z << " to " << new_Z << std::endl;
    Z = new_Z;
    vsStarted = false;
    initVisualServoTasks();
  }

  vsStarted = true;
}
void publish4rqt(const vpImage<unsigned char> &image,
                 const image_transport::Publisher &pub) {
  pub.publish(visp_bridge::toSensorMsgsImage(image));
}

template <typename F, typename C>
void resizeAndConvert4rqt(vpImage<float> &feat_float,
                          vpImage<unsigned char> &feat_char,
                          const vpImage<unsigned char> &image, F &fSet,
                          C &cam) {
  feat_float.resize(image.getHeight(), image.getWidth(), true);
  feat_char.resize(image.getHeight(), image.getWidth(), true);
  fSet.sampler.toImage(feat_float, pp, &cam);
  vpImageConvert::convert(feat_float, feat_char);
}

template <typename F, typename C>
void convertAndPublish4rqt(F &fSet, vpImage<float> &feat_float,
                           vpImage<unsigned char> &feat_char, C &cam,
                           const vpImage<unsigned char> &image,
                           const image_transport::Publisher &pub) {
  fSet.sampler.toImage(feat_float, pp, &cam);
  vpImageConvert::convert(feat_float, feat_char);
  pub.publish(visp_bridge::toSensorMsgsImage(image));
}

void mySigintHandler(int sig) {
  v = 0;
  vpDisplay::close(leftI);
  vpDisplay::close(rightI);
  vpDisplay::close(leftIdiff);
  vpDisplay::close(rightIdiff);
  robotVelocityPub.publish(geometryTwistFromvpColVector(v));
  if (rosbagForEVS) {
    vsbag.close();
  }
  if (saveExperimentData) {
    copyData();
  }
  ros::shutdown();
}

vpHomogeneousMatrix toVispHomogeneousMatrix(const tf2_msgs::TFMessage &trans) {
  vpHomogeneousMatrix mat;
  vpTranslationVector vec(trans.transforms[0].transform.translation.x,
                          trans.transforms[0].transform.translation.y,
                          trans.transforms[0].transform.translation.z);
  vpRotationMatrix rmat;

  double a = trans.transforms[0].transform.rotation.w; // x
  double b = trans.transforms[0].transform.rotation.x; // y
  double c = trans.transforms[0].transform.rotation.y; // z
  double d = trans.transforms[0].transform.rotation.z; // w
  rmat[0][0] = a * a + b * b - c * c - d * d;
  rmat[0][1] = 2 * b * c - 2 * a * d;
  rmat[0][2] = 2 * a * c + 2 * b * d;

  rmat[1][0] = 2 * a * d + 2 * b * c;
  rmat[1][1] = a * a - b * b + c * c - d * d;
  rmat[1][2] = 2 * c * d - 2 * a * b;

  rmat[2][0] = 2 * b * d - 2 * a * c;
  rmat[2][1] = 2 * a * b + 2 * c * d;
  rmat[2][2] = a * a - b * b - c * c + d * d;

  mat.buildFrom(vec, rmat);

  return mat;
}

void callbackServer(dhpgmvs::lambda_gConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f  %f  %f", config.lambda_g, config.lambda,
           config.sceneDepth);
  updateParameters(config.lambda_g, config.lambda, config.sceneDepth);
}

void copyData() {
  std::string user_folder_name;
  std::cout << "Enter the name for the folder: ";
  std::getline(std::cin, user_folder_name);

  std::string exp;
  std::stringstream exp_path;
  exp_path << logs_path << "/exp" << user_folder_name << "dual-fisheye";
  std::filesystem::create_directories(exp_path.str());

  std::filesystem::copy(logs_path + "/velocities.txt",
                        exp_path.str() + "/velocities.txt");
  std::filesystem::copy(logs_path + "/residuals.txt",
                        exp_path.str() + "/residuals.txt");
  std::filesystem::copy(logs_path + "/Id_des_right.png",
                        exp_path.str() + "/Id_des_right.png");
  std::filesystem::copy(logs_path + "/Id_init_right.png",
                        exp_path.str() + "/Id_init_right.png");
  std::filesystem::copy(logs_path + "/Id_des_left.png",
                        exp_path.str() + "/Id_des_left.png");
  std::filesystem::copy(logs_path + "/Id_init_left.png",
                        exp_path.str() + "/Id_init_left.png");
  std::filesystem::copy(bagFilePath, exp_path.str());

  std::cout << "Experiment Data saved" << std::endl;
}

/**************************************/
/**************ROS <-> VISP*************/
/**************************************/

vpHomogeneousMatrix vpHomogeneousMatrixFromROSTransform(std::string frame_i,
                                                        std::string frame_o) {
  geometry_msgs::Pose oMi;
  tf::StampedTransform oMi_tf;
  tf::TransformListener listener;
  listener.waitForTransform(frame_o, frame_i, ros::Time(0), ros::Duration(3.0));
  listener.lookupTransform(frame_o, frame_i, ros::Time(0), oMi_tf);
  tf::poseTFToMsg(oMi_tf, oMi);
  return visp_bridge::toVispHomogeneousMatrix(oMi);
}

geometry_msgs::Twist geometryTwistFromvpColVector(vpColVector vpVelocity) {
  geometry_msgs::Twist geoVelocity;
  geoVelocity.linear.x = vpVelocity[0];
  geoVelocity.linear.y = vpVelocity[1];
  geoVelocity.linear.z = vpVelocity[2];
  geoVelocity.angular.x = vpVelocity[3];
  geoVelocity.angular.y = vpVelocity[4];
  geoVelocity.angular.z = vpVelocity[5];
  return geoVelocity;
}

cameraParameters cameraInfoToCameraParam(sensor_msgs::CameraInfo msg) {
  cameraParameters cam;
  cam.height = msg.height;
  cam.width = msg.width;

  std::cout << msg.K[0] << " " << msg.K[4] << " " << msg.K[2] << " " << msg.K[5]
            << " " << msg.D[4] << std::endl;

  cam.cam.init(msg.K[0], msg.K[4], msg.K[2], msg.K[5], msg.D[4]);
  return cam;
}
