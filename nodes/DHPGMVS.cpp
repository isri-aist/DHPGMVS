/*
 * DHPGMVS: Dual-Hemispherical Photometric Gaussian Mixtures-based Visual Servoing
 * Authors: Guillaume Caron (guillaume.caron@u-picardie.fr)
 *          Nathan Crombez (nathan.crombez@utbm.fr)
 * Institutions: CNRS-AIST JRL / UTBM CIAD
 * Date: April 2024, January 2024
 */

////ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

////BOOST
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>

////ViSP
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpExponentialMap.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageTools.h>
#include <visp/vpPlot.h>
#include <visp3/core/vpImageFilter.h>

////libPeR
//Camera type considered
#include <per/prOmni.h>
//Acquisition model
#include <per/prRegularlySampledCPImage.h>
//Visual features
#include <per/prPhotometricnnGMS.h>
#include <per/prFeaturesSet.h>
#include <per/prSSDCmp.h>
//Generic Visual Servo Control tools 
#include <per/prCameraPoseEstim.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

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
vpDisplayX rightDispI, leftDispI, rightDispId, leftDispId, rightDispIdiff, leftDispIdiff;
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
prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > fSet_des_right, fSet_des_left;
prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample_des_right, *GP_sample_des_left;

// libPeR's current visual features for visual servoing
prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample_right, *GP_sample_left;
prRegularlySampledCPImage<unsigned char> *IP_cur_right, *IP_cur_left;
prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > fSet_cur_right, fSet_cur_left;
prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample_cur_right, *GP_sample_cur_left;

// libPeR's visual servoing tasks (will be merged Crombez' style)
prCameraPoseEstim<prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage >, 
                prSSDCmp<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec> > > servo_right, servo_left; 

// other LibPeR's variables
double lambda_g;
bool updateSampler;
bool poseJacobianCompute;
bool robust; //to activate the M-Estimator

////OTHERS
ros::Time t;
bool verbose;
int qSize;

////Main VS loop (callback)
void camerasImageRobotPoseCallback(const sensor_msgs::Image::ConstPtr &rightImsg, const sensor_msgs::Image::ConstPtr &leftImsg, const geometry_msgs::PoseStamped::ConstPtr &robotPoseMsg);
////DHPGM error
void computeDHPGMErrorVector(vpColVector &e_right, vpColVector &e_left, vpColVector &e);
////DHPGM interaction matrix
void computeDHPGMInteractionMatrix(vpMatrix &L_right, cameraParameters rightCamParam,  vpMatrix &L_left, cameraParameters leftCamParam, vpMatrix &L);

void initVisualServoTasks();

////Camera's poses initialization
void cameraPosesInitialization();

////VISP <--> ROS Messages
vpHomogeneousMatrix vpHomogeneousMatrixFromROSTransform(std::string frame_i, std::string frame_o);
geometry_msgs::Twist geometryTwistFromvpColVector(vpColVector vpVelocity);
cameraParameters cameraInfoToCameraParam(sensor_msgs::CameraInfo msg);


int main(int argc, char **argv){
    ros::init(argc, argv, "DHPGMVS", ros::init_options::NoSigintHandler);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle nh("~");

    qSize = 30;
    vsStarted = false;
    v.resize(6);

    updateSampler = true;
    poseJacobianCompute = true;

    ////Get parameters
    verbose = nh.param("verbose", 1);
    gain = nh.param("gain", 1.0);
    Z = nh.param("Z", 1.0);
    lambda_g = nh.param("lambda_g", 1.0);
    robust = nh.param("robust", false);

    ////Get right and left cameras intrinsic parameters
    rightCameraParameters = cameraInfoToCameraParam(*ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/dhpgmvs/right_camera/camera_info", nh));
    leftCameraParameters = cameraInfoToCameraParam(*ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/dhpgmvs/left_camera/camera_info", nh));
    ////Get right and left cameras extrinsic parameters
    rightCameraParameters.cMf = vpHomogeneousMatrixFromROSTransform("/flange", "/right_camera");
    leftCameraParameters.cMf = vpHomogeneousMatrixFromROSTransform( "/flange", "/left_camera");

    ////Robot velocities publisher
    robotVelocityPub = nh.advertise<geometry_msgs::Twist>("/dhpgmvs/robot/set_velocity", qSize);

    ////Cameras and robot synchronizer
    message_filters::Subscriber<sensor_msgs::Image> rightCameraSub, leftCameraSub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> robotPoseSub;
    rightCameraSub.subscribe(nh, "/dhpgmvs/right_camera/image_raw", qSize);
    leftCameraSub.subscribe(nh, "/dhpgmvs/left_camera/image_raw", qSize);
    robotPoseSub.subscribe(nh,"/dhpgmvs/robot/get_pose", qSize);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped>> camerasSynchronizer(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped>(qSize), rightCameraSub, leftCameraSub, robotPoseSub);
    camerasSynchronizer.registerCallback(boost::bind(&camerasImageRobotPoseCallback, _1, _2, _3));

    ////Image displayers
    rightI.resize(rightCameraParameters.height, rightCameraParameters.width);
    leftI.resize(leftCameraParameters.height, leftCameraParameters.width);
    rightId.resize(rightCameraParameters.height, rightCameraParameters.width);
    leftId.resize(leftCameraParameters.height, leftCameraParameters.width);
    rightIdiff.resize(rightCameraParameters.height, rightCameraParameters.width);
    leftIdiff.resize(leftCameraParameters.height, leftCameraParameters.width);
    leftDispI.init(leftI, 0, 27, "LEFT I");
    leftDispId.init(leftId, 0, 27+37+leftCameraParameters.height, "LEFT I*");
    leftDispIdiff.init(leftIdiff, 0,  27+(37+leftCameraParameters.height)*2, "LEFT DIFF");
    rightDispI.init(rightI, rightCameraParameters.width, 27, "RIGHT I");
    rightDispId.init(rightId, rightCameraParameters.width,  27+37+rightCameraParameters.height, "RIGHT I*");
    rightDispIdiff.init(rightIdiff, rightCameraParameters.width,  27+(37+rightCameraParameters.height)*2, "RIGHT DIFF");

    ////Plots
    plot.init(4,(37+rightCameraParameters.height)*3-37, rightCameraParameters.width, rightCameraParameters.width*2,27);
    plot.initGraph(0,1); plot.setTitle(0,"Feature error"); plot.setLegend(0,0,"error");
    plot.initGraph(1,6); plot.setTitle(1,"Velocities");
    plot.setLegend(1,0,"v_x");     plot.setLegend(1,1,"v_y");     plot.setLegend(1,2,"v_z");
    plot.setLegend(1,3,"w_x");     plot.setLegend(1,4,"w_y");     plot.setLegend(1,5,"w_z");
    plot.initGraph(2,3); plot.setTitle(2,"Translation error");
    plot.setLegend(2,0,"dtx_x");     plot.setLegend(2,1,"dt_y");     plot.setLegend(2,2,"dt_z");
    plot.initGraph(3,3); plot.setTitle(3,"Orientation error");
    plot.setLegend(3,0,"dw_x");     plot.setLegend(3,1,"dw_y");     plot.setLegend(3,2,"dw_z");

    ////Camera desired and initial pose initialization
    cameraPosesInitialization();

ROS_WARN("initVisualServoTasks");
    ////libPeR's objects initializations
    initVisualServoTasks();

ROS_WARN("initVisualServoTasks done");
    ////Actually start DHPGMVS
    iter=0;
    vsStarted = true;
    t = ros::Time::now();
    ros::spin();
}

void cameraPosesInitialization(){
    for(int i=3;i>0;i--){
        ROS_WARN("%d", i);
        sleep(1);
    }

    ROS_WARN("Move the camera to the desired pose then click on LEFT I");
    do{
        rightId = rightI;
        leftId = leftI;
        vpDisplay::display(rightId); vpDisplay::flush(rightId);
        vpDisplay::display(leftId);  vpDisplay::flush(leftId);
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
    desiredRobotPose = currentRobotPose;

    ROS_WARN("Move the camera to the initial pose then click on LEFT I");
    do{
        vpImageTools::imageDifference(leftI, leftId, leftIdiff);
        vpImageTools::imageDifference(rightI, rightId, rightIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
    initialRobotPose = currentRobotPose;

    ROS_WARN("Cick on LEFT I to start VS");
    do{
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
}

void initVisualServoTasks()
{
    ////RIGHT CAMERA
    // 2. VS objects initialization, considering the pose control of a perspective camera from the feature set of photometric non-normalized Gaussian mixture 2D samples compared thanks to the SSD
    servo_right.setdof(true, true, true, true, true, true);
    servo_right.setSensor(&(rightCameraParameters.cam));

    // desired visual feature built from the image
    //prepare the desired image 
    IP_des_right = new prRegularlySampledCPImage<unsigned char>(rightCameraParameters.height, rightCameraParameters.width); //the regularly sample planar image to be set from the acquired/loaded perspective image
    IP_des_right->setInterpType(INTERPTYPE);
    IP_des_right->buildFrom(rightId, &(rightCameraParameters.cam)); 

    GP_right = new prRegularlySampledCPImage<float>(rightCameraParameters.height, rightCameraParameters.width); //contient tous les pr2DCartesianPointVec (ou prFeaturePoint) u_g et fera GS_sample.buildFrom(IP_des, u_g);

    GP_sample_des_right = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

    fSet_des_right.buildFrom(*IP_des_right, *GP_right, *GP_sample_des_right, false, true); // Goulot !

    /*
    m_logfile << "des built" << endl;
    if(m_pub_diffFeaturesImage || m_pub_desiredFeaturesImage)
    {
        PGM_des_f.resize(m_height, m_width, true);
        PGM_des_u.resize(m_height, m_width, true);
        fSet_des.sampler.toImage(PGM_des_f, pp, _camera);
        vpImageConvert::convert(PGM_des_f, PGM_des_u);

        m_logfile << "convert des " << PGM_des_u.getWidth() << " " << PGM_des_u.getHeight() << endl;
    }

    if(m_pub_desiredFeaturesImage)
    {
        m_desiredFeaturesImage = visp_bridge::toSensorMsgsImage(PGM_des_u);
        m_desiredFeaturesImage_pub.publish(m_desiredFeaturesImage);
    }
    */

    servo_right.buildFrom(fSet_des_right);

    servo_right.initControl(gain, Z);

	// current visual feature built from the image
    GP_sample_right = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

    // Current features set setting from the current image
    IP_cur_right = new prRegularlySampledCPImage<unsigned char>(rightCameraParameters.height, rightCameraParameters.width);
    IP_cur_right->setInterpType(INTERPTYPE);
    IP_cur_right->buildFrom(rightI, &(rightCameraParameters.cam)); 
    
    fSet_cur_right.buildFrom(*IP_cur_right, *GP_right, *GP_sample_right, poseJacobianCompute, updateSampler); // Goulot !

    /*
    m_logfile << "cur built " << m_height << " " << m_width << endl;
    if(m_pub_diffFeaturesImage || m_pub_featuresImage)
    {
        PGM_cur_f.resize(m_height, m_width, true);
        PGM_cur_u.resize(m_height, m_width, true);
        fSet_cur.sampler.toImage(PGM_cur_f, pp, _camera);
        vpImageConvert::convert(PGM_cur_f, PGM_cur_u);

        m_logfile << "convert cur " << PGM_cur_u.getWidth() << " " << PGM_cur_u.getHeight() << endl;
    }

    if(m_pub_diffFeaturesImage)
    {
        m_difference_pgm.resize(m_height, m_width, true);
    }

    if(m_pub_diffImage)
    {
        m_difference_image.resize(m_height, m_width, true);
    }
    */

    //vsInitialized = true;

   ////LEFT CAMERA
    // 2. VS objects initialization, considering the pose control of a perspective camera from the feature set of photometric non-normalized Gaussian mixture 2D samples compared thanks to the SSD
    servo_left.setdof(true, true, true, true, true, true);
    servo_left.setSensor(&(leftCameraParameters.cam));

    // desired visual feature built from the image
    //prepare the desired image 
    IP_des_left = new prRegularlySampledCPImage<unsigned char>(leftCameraParameters.height, leftCameraParameters.width); //the regularly sample planar image to be set from the acquired/loaded perspective image
    IP_des_left->setInterpType(INTERPTYPE);
    IP_des_left->buildFrom(leftId, &(leftCameraParameters.cam)); 

    GP_left = new prRegularlySampledCPImage<float>(leftCameraParameters.height, leftCameraParameters.width); //contient tous les pr2DCartesianPointVec (ou prFeaturePoint) u_g et fera GS_sample.buildFrom(IP_des, u_g);

    GP_sample_des_left = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

    fSet_des_left.buildFrom(*IP_des_left, *GP_left, *GP_sample_des_left, false, true); // Goulot !

    /*
    m_logfile << "des built" << endl;
    if(m_pub_diffFeaturesImage || m_pub_desiredFeaturesImage)
    {
        PGM_des_f.resize(m_height, m_width, true);
        PGM_des_u.resize(m_height, m_width, true);
        fSet_des.sampler.toImage(PGM_des_f, pp, _camera);
        vpImageConvert::convert(PGM_des_f, PGM_des_u);

        m_logfile << "convert des " << PGM_des_u.getWidth() << " " << PGM_des_u.getHeight() << endl;
    }

    if(m_pub_desiredFeaturesImage)
    {
        m_desiredFeaturesImage = visp_bridge::toSensorMsgsImage(PGM_des_u);
        m_desiredFeaturesImage_pub.publish(m_desiredFeaturesImage);
    }
    */

    servo_left.buildFrom(fSet_des_left);

    servo_left.initControl(gain, Z);

	// current visual feature built from the image
    GP_sample_left = new prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);

    // Current features set setting from the current image
    IP_cur_left = new prRegularlySampledCPImage<unsigned char>(leftCameraParameters.height, leftCameraParameters.width);
    IP_cur_left->setInterpType(INTERPTYPE);
    IP_cur_left->buildFrom(leftI, &(leftCameraParameters.cam)); 
    
    fSet_cur_left.buildFrom(*IP_cur_left, *GP_left, *GP_sample_left, poseJacobianCompute, updateSampler); // Goulot !

    /*
    m_logfile << "cur built " << m_height << " " << m_width << endl;
    if(m_pub_diffFeaturesImage || m_pub_featuresImage)
    {
        PGM_cur_f.resize(m_height, m_width, true);
        PGM_cur_u.resize(m_height, m_width, true);
        fSet_cur.sampler.toImage(PGM_cur_f, pp, _camera);
        vpImageConvert::convert(PGM_cur_f, PGM_cur_u);

        m_logfile << "convert cur " << PGM_cur_u.getWidth() << " " << PGM_cur_u.getHeight() << endl;
    }

    if(m_pub_diffFeaturesImage)
    {
        m_difference_pgm.resize(m_height, m_width, true);
    }

    if(m_pub_diffImage)
    {
        m_difference_image.resize(m_height, m_width, true);
    }
    */

    //vsInitialized = true;
}

void camerasImageRobotPoseCallback(const sensor_msgs::Image::ConstPtr &rightImsg, const sensor_msgs::Image::ConstPtr &leftImsg, const geometry_msgs::PoseStamped::ConstPtr &robotPoseMsg) {
    ////ROS to VISP
    rightI = visp_bridge::toVispImage(*rightImsg);
    leftI = visp_bridge::toVispImage(*leftImsg);
    currentRobotPose = vpPoseVector(visp_bridge::toVispHomogeneousMatrix(robotPoseMsg->pose));

    ROS_WARN("VS");

    if (vsStarted) {
        ROS_WARN("VS");

        if(robotPoseMsg->header.stamp<=t)   //Prevent ROS synchro issues
             return;

        ////Displays
        vpImageTools::imageDifference(rightI, rightId,rightIdiff);
        vpImageTools::imageDifference(leftI, leftId,leftIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);

        ////Update left and right camera features
        IP_cur_right->buildFrom(rightI, &(rightCameraParameters.cam)); 
	    fSet_cur_right.updateMeasurement(*IP_cur_right, *GP_right, *GP_sample_cur_right, poseJacobianCompute, updateSampler); 
        servo_right.interactionAndError(fSet_cur_right, L_right, e_right, robust);

        IP_cur_left->buildFrom(leftI, &(leftCameraParameters.cam)); 
	    fSet_cur_left.updateMeasurement(*IP_cur_left, *GP_left, *GP_sample_cur_left, poseJacobianCompute, updateSampler); 
        servo_left.interactionAndError(fSet_cur_left, L_left, e_left, robust);

        ////Compute interaction matrix
        computeDHPGMInteractionMatrix(L_right,rightCameraParameters,L_left,leftCameraParameters,L);
        ////Compute error vector
        computeDHPGMErrorVector(e_right,  e_left, e);
        ////Compute Gauss-newton control law
        v = -gain * L.pseudoInverseEigen3() * e; //Velocities expressed in the robot's flange

        ////Send velocity to robot
        robotVelocityPub.publish(geometryTwistFromvpColVector(v));

        t = ros::Time::now();

        ////PLOTS
        plot.plot(0,0,iter, e.sumSquare());
        for(int i=0;i<3;i++)
            plot.plot(2,i,iter,currentRobotPose[i]-desiredRobotPose[i]);
        for(int i=0;i<3;i++)
            plot.plot(3, i, iter, vpMath::deg(currentRobotPose[i+3]-desiredRobotPose[i+3]));
        for(int i=0;i<6;i++)
            plot.plot(1,i,iter, v[i]);

        ////VERBOSE
        if(verbose){
            ROS_DEBUG("Iteration: %d", iter);
            ROS_DEBUG("Velocities: %f %f %f %f %f %f", v[0], v[1], v[2], v[3], v[4], v[5]);
            ROS_DEBUG("Current Pose: %f %f %f %f %f %f", currentRobotPose[0], currentRobotPose[1], currentRobotPose[2], currentRobotPose[3], currentRobotPose[4], currentRobotPose[5]);
            ROS_DEBUG("Desired Pose: %f %f %f %f %f %f", desiredRobotPose[0], desiredRobotPose[1], desiredRobotPose[2], desiredRobotPose[3], desiredRobotPose[4], desiredRobotPose[5]);
            ROS_DEBUG("Error Pose: %f %f %f %f %f %f", fabs(currentRobotPose[0]-desiredRobotPose[0]),
                                                       fabs(currentRobotPose[1]-desiredRobotPose[1]),
                                                       fabs(currentRobotPose[2]-desiredRobotPose[2]),
                                                       vpMath::deg(fabs(currentRobotPose[3]-desiredRobotPose[3])),
                                                       vpMath::deg(fabs(currentRobotPose[4]-desiredRobotPose[4])),
                                                       vpMath::deg(fabs(currentRobotPose[5]-desiredRobotPose[5])));
            ROS_DEBUG("Photometric error: %f", e.sumSquare());
        }

        iter++;
    }
}

void computeDHPGMErrorVector(vpColVector &e_right, vpColVector &e_left, vpColVector &e)
{
    e.resize(e_right.getRows()+e_left.getRows());
    
    ////RIGHT CAMERA
    e.insert(0, e_right);

    ////LEFT CAMERA
    e.insert(e_right.getRows(), e_left);
}


void computeDHPGMInteractionMatrix(vpMatrix &L_right, cameraParameters rightCamParam,  vpMatrix &L_left, cameraParameters leftCamParam, vpMatrix &L)
{
    L.clear();
    L.resize(L_right.getRows()+L_left.getRows(), 6);
    vpVelocityTwistMatrix V;

    ////RIGHT CAMERA
    V.buildFrom(rightCamParam.cMf.getRotationMatrix());
    
    L.insert(L_right*V, 0, 0);

    ////LEFT CAMERA
    V.buildFrom(leftCamParam.cMf.getRotationMatrix());

    L.insert(L_left*V, L_right.getRows(), 0);
}


/**************************************/
/**************ROS <-> VISP*************/
/**************************************/

vpHomogeneousMatrix vpHomogeneousMatrixFromROSTransform(std::string frame_i, std::string frame_o){
    geometry_msgs::Pose oMi;
    tf::StampedTransform oMi_tf;
    tf::TransformListener listener;
    listener.waitForTransform(frame_o, frame_i, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(frame_o, frame_i, ros::Time(0), oMi_tf);
    tf::poseTFToMsg(oMi_tf, oMi);
    return visp_bridge::toVispHomogeneousMatrix(oMi);
}

geometry_msgs::Twist geometryTwistFromvpColVector(vpColVector vpVelocity){
    geometry_msgs::Twist geoVelocity;
    geoVelocity.linear.x = vpVelocity[0];
    geoVelocity.linear.y = vpVelocity[1];
    geoVelocity.linear.z = vpVelocity[2];
    geoVelocity.angular.x = vpVelocity[3];
    geoVelocity.angular.y = vpVelocity[4];
    geoVelocity.angular.z = vpVelocity[5];
    return geoVelocity;
}

cameraParameters cameraInfoToCameraParam(sensor_msgs::CameraInfo msg){
    cameraParameters cam;
    cam.height = msg.height;
    cam.width = msg.width;
    cam.cam.init(msg.K[0], msg.K[4], msg.K[2], msg.K[5], msg.D[4]);
    return cam;
}






