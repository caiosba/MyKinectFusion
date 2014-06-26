#include "Reconstruction.h"
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#define PI 3.14159265

#define _CRT_SECURE_NO_DEPRECATE
#define NOMINMAX

typedef pcl::ScopeTime ScopeTimeT;

Reconstruction::Reconstruction(Eigen::Vector3i& volumeSize) {

  hasImage_ = false;
  hasIncrement_ = true;
  hasErrorVisualization_ = false;
  hasTsdfVolumeVisualization_ = false;

  isOnlyTrackingOn_ = false;
  stopTracking_ = false;

  headPoseEstimationOk = false;

  image_ = new Image(640, 480);
  tsdfVolume_ = new TsdfVolume(volumeSize);

  firstPointCloud_ = new MyPointCloud(640, 480);
  currentPointCloud_ = new MyPointCloud(640, 480);
  globalPreviousPointCloud_ = new MyPointCloud(640, 480);
  auxPointCloud_ = new MyPointCloud(640, 480);

	enableYawPitchRollFromGlasses = true;
	enableXYZFromGlasses = true;
	enableGlasses = true;

  float f = 525.f;
  image_->setDepthIntrinsics(f, f);
  image_->setTrancationDistance(tsdfVolume_->getVolumeSize());

  init_Rcam_ = Eigen::Matrix3f::Identity(); // * AngleAxisf(-180.f/180*3.1415926, Vector3f::UnitY());
        // init_tcam_ = Eigen::Vector3f::Zero();
  // init_tcam_ = { 1500, 1500, -300 };
        // The values above are the results from the expression below... the volume is a 3000-wide cube
  init_tcam_ = tsdfVolume_->getVolumeSize() * 0.5f - Vector3f (0, 0, tsdfVolume_->getVolumeSize() (2) / 2 * 1.2f);
  
  rmats_.reserve (30000);
  tvecs_.reserve (30000);

  reset();

  previousDepthData = new unsigned short[640 * 480];

}

void Reconstruction::transformCamera(std::vector<Matrix3frm>& Rcam, std::vector<Vector3f>& tcam, int globalTime) {
   Matrix3frm rmatz;
   int angle = -30;
   rmatz << cos(angle*PI/180), -sin(angle*PI/180), 0,
            sin(angle*PI/180), cos(angle*PI/180), 0,
            0, 0, 1;
   Vector3f tvecz;
   tvecz = { 0, 0, 0 };
   Rcam[globalTime] = rmatz * Rcam[globalTime];
   // tcam[globalTime] = tcam[globalTime] + tvecz; 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Reconstruction::getPCLPointCloud() {
  DeviceArray<pcl::PointXYZ> extractedCloudDevice;
  DeviceArray<PointXYZ> extracted =  tsdfVolume_->fetchCloud(extractedCloudDevice);
  pcl::PointCloud<pcl::PointXYZ>::Ptr hostCloud = pcl::PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
  extracted.download(hostCloud->points);
  hostCloud->width = (int)hostCloud->points.size ();
  hostCloud->height = 1;
  return hostCloud;
}

void Reconstruction::savePointCloud() {
  
  DeviceArray<pcl::PointXYZ> extractedCloudDevice;
  DeviceArray<PointXYZ> extracted =  tsdfVolume_->fetchCloud(extractedCloudDevice);

  pcl::PointCloud<pcl::PointXYZ>::Ptr hostCloud = pcl::PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
  extracted.download(hostCloud->points);
  hostCloud->width = (int)hostCloud->points.size ();
  hostCloud->height = 1;
  
  char fileToSave[1000];
  std::cout << "Write filename..." << std::endl;
  std::cin >> fileToSave;
  std::cout << fileToSave << std::endl;
  pcl::io::savePCDFile(fileToSave, *hostCloud); 
  std::cout << "Model saved..." << std::endl;

}

void Reconstruction::reset() {

  if(!isOnlyTrackingOn_) {
  
    rmats_.clear ();
    tvecs_.clear ();

    rmats_.push_back (init_Rcam_);
    tvecs_.push_back (init_tcam_);
    globalTime = 0;
    tsdfVolume_->reset();

    hasIncrement_ = false;

    std::cout << "Reset" << std::endl;
  }
}

void Reconstruction::readPoseFromFile() {
  // return;
  cout << "Using transformation from file" << endl;
  Matrix3frm rcurr = this->getCurrentRotation();
	Vector3f tcurr = this->getCurrentTranslation();
  Matrix3frm ri = this->getRotationMatrices()[0];
	Vector3f ti = this->getTranslationVectors()[0];

  cv::FileStorage fs;
  fs.open("mystereocalib.yml", cv::FileStorage::READ);
	cv::Mat r, t;
	Matrix3frm r2;
	Vector3f t2;
	fs["R"] >> r;
	fs["T"] >> t;
	cv2eigen(r, r2);
	cv2eigen(t, t2);
	fs.release();

  // Transform based on calibration file
	Matrix3frm rotation = r2.inverse() * rcurr;
	Vector3f translation = r2.inverse() * tcurr + t2;

  // Transformation based on glasses
	if (this->useGlasses()) {

	  // Rotation based on accelerometer from glasses
	  long yaw = this->getGlassesYaw();
	  long pitch = this->getGlassesPitch();
	  long roll = this->getGlassesRoll();
	  Matrix3frm yaw_rm, pitch_rm, roll_rm;
	  double y, p, ro;
	  y = yaw * PI / 180.0;
	  p = pitch * PI / 180.0;
	  ro = roll * PI / 180.0;
    Eigen::Quaternion<double> q = Eigen::AngleAxisd(ro, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitX()); 

    Matrix3d qm;
    Matrix3frm rm;
	  qm = q.matrix();
	  rm = qm.cast<float>();
	  rotation = rm.inverse() * rotation;
	  printf("Yaw: %ld Pitch: %ld Roll: %ld\n", (long)yaw, (long)pitch, (long)roll);
  
	  // Translation based on optical flow from glasses
	  double opx, opy, opz;
	  opx = this->getGlassesX();
	  opy = this->getGlassesY();
	  opz = this->getGlassesZ();
	  cv::Mat t3;
	  t3 = (cv::Mat_<double>(3,1) << opx, opy, opz);
	  Vector3f t4;
	  cv2eigen(t3, t4);
	  translation = rm.inverse() * translation + t4;
	  printf("X: %f Y: %f Z: %f\n", opx, opy, opz);

	}

  // Debug
	/*
	cout << "Rotation" << endl;
	cout << rotation << endl;
	cout << "Translation" << endl;
	cout << translation << endl;
	*/

	this->setPoseR(rotation);
  this->setPoseT(translation);
}

void Reconstruction::run(boost::shared_ptr<openni_wrapper::Image>& rgbImage, boost::shared_ptr<openni_wrapper::DepthImage>& depthImage) {
  depthMap = image_->getDepthMap();
  depthDevice = image_->getDepthDevice();
  rgbDevice = image_->getRgbDevice();

  depthMap.cols = depthImage->getWidth();
  depthMap.rows = depthImage->getHeight();
  depthMap.step = depthMap.cols * depthMap.elemSize();
  
  sourceDepthData.resize(depthMap.cols * depthMap.rows);
  depthImage->fillDepthImageRaw(depthMap.cols, depthMap.rows, &sourceDepthData[0]);
  depthMap.data = &sourceDepthData[0];

  rgbMap.cols = rgbImage->getWidth();
  rgbMap.rows = rgbImage->getHeight();
  rgbMap.step = rgbMap.cols * rgbMap.elemSize();

  sourceRgbData.resize(rgbMap.cols * rgbMap.rows);
  rgbImage->fillRGB(rgbMap.cols, rgbMap.rows, (unsigned char*)&sourceRgbData[0]);
  rgbMap.data = &sourceRgbData[0];
  
  rgbDevice.upload(rgbMap.data, rgbMap.step, rgbMap.rows, rgbMap.cols);
  depthDevice.upload(depthMap.data, depthMap.step, depthMap.rows, depthMap.cols);
  
  ScopeTimeT time ("total-frame");
  {
    image_->setDepthDevice(depthDevice);
    image_->setRgbDevice(rgbDevice);
    image_->applyBilateralFilter();
    image_->applyDepthTruncation(threshold_);
    image_->applyPyrDown();
    image_->convertToPointCloud(currentPointCloud_);
    image_->applyDepthTruncation(depthDevice, threshold_);
    pcl::device::sync();

    // First step
    if (globalTime == 0) {
      tsdfVolume_->integrateVolume(rmats_, tvecs_, depthDevice, image_->getIntrinsics(), image_->getTrancationDistance(), image_->getDepthRawScaled(), globalTime);
      currentPointCloud_->transformPointCloud(rmats_[0], tvecs_[0], globalPreviousPointCloud_->getVertexMaps(), globalPreviousPointCloud_->getNormalMaps());
    }

    // From the second step, on
    else {
      hasImage_ = currentPointCloud_->alignPointClouds(rmats_, tvecs_, globalPreviousPointCloud_, image_->getIntrinsics(), globalTime);

      if (!hasImage_) reset();
      else {
        tsdfVolume_->integrateVolume(rmats_, tvecs_, depthDevice, image_->getIntrinsics(), image_->getTrancationDistance(), image_->getDepthRawScaled(), globalTime);
        // if (changePose_) this->transformCamera(rmats_, tvecs_, globalTime);
        if (changePose_) {
          this->readPoseFromFile();
          tsdfVolume_->raycastFromPose(rmats_, tvecs_, image_->getIntrinsics(), image_->getTrancationDistance(), globalPreviousPointCloud_, globalTime, pose_rmats_, pose_tvecs_);
        }
        else {
          tsdfVolume_->raycast(rmats_, tvecs_, image_->getIntrinsics(), image_->getTrancationDistance(), globalPreviousPointCloud_, globalTime);
        }
        pcl::device::sync ();
      }
    }
  } // END LOOP
    
  // Increment or not
  if (hasIncrement_) {
    globalTime++;
    for (int pixel = 0; pixel < 640 * 480; pixel++)
      previousDepthData[pixel] = depthMap.data[pixel];
  } else hasIncrement_ = true;

}

void Reconstruction::changePose()
{
  if (changePose_) changePose_ = false;
  else changePose_ = true;
}

void Reconstruction::enableOnlyTracking(bool stopFaceDetection) 
{
  isOnlyTrackingOn_ = true;  
}

bool Reconstruction::reRunICP() 
{
  hasImage_ = currentPointCloud_->alignPointClouds(rmats_, tvecs_, globalPreviousPointCloud_, image_->getIntrinsics(), globalTime);
  if(hasImage_)
    std::cout << "Error: " << currentPointCloud_->computeFinalError() << std::endl;
  else
    std::cout << "ICP Failed" << std::endl;
  return hasImage_;

}

void Reconstruction::reRunRaycasting() 
{  
  tsdfVolume_->raycast(rmats_, tvecs_, image_->getIntrinsics(), image_->getTrancationDistance(), globalPreviousPointCloud_, globalTime);
}

void Reconstruction::transformGlobalPreviousPointCloud(Eigen::Matrix3f& Rinc, Eigen::Vector3f& tvec, Eigen::Vector3f& centerOfMass)
{

  globalPreviousPointCloud_->transformPointCloud(Rinc, tvec, globalPreviousPointCloud_->getVertexMaps(), globalPreviousPointCloud_->getNormalMaps(), init_tcam_, 
    centerOfMass);

}
 
unsigned char* Reconstruction::getRaycastImage() {
  int cols;
  Eigen::Vector3f cpuVolumeSize = tsdfVolume_->getVolumeSize();
  image_->getRaycastImage(viewDevice_, cpuVolumeSize, globalPreviousPointCloud_);
  viewDevice_.download (view_host_, cols);

  return (unsigned char*)view_host_.data();
}

unsigned char* Reconstruction::getRaycastImageFromPose() {
  int cols;
  Eigen::Vector3f cpuVolumeSize = tsdfVolume_->getVolumeSize();
  image_->getRaycastImageFromPose(viewDevice_, cpuVolumeSize, globalPreviousPointCloud_);
  viewDevice_.download (view_host_, cols);

  return (unsigned char*)view_host_.data();
}

void Reconstruction::getPointCloud(float *pointCloud, bool globalCoordinates) {
  
  int c;
  
  if(globalCoordinates)
    globalPreviousPointCloud_->getLastFrameCloud(cloudDevice_);
  else
    currentPointCloud_->getLastFrameCloud(cloudDevice_);

  cloudDevice_.download (cloudHost_, c);
  
  for(int point = 0; point < (640 * 480); point++)
  {
    pointCloud[point * 3 + 0] = cloudHost_[point].x;
    pointCloud[point * 3 + 1] = cloudHost_[point].y;
    pointCloud[point * 3 + 2] = cloudHost_[point].z;
  }
        
}

void Reconstruction::getNormalVector(float *normalVector, bool globalCoordinates) {
  
  int c;
  int inverse;

  if(globalCoordinates) {
    globalPreviousPointCloud_->getLastFrameNormals(normalsDevice_);
    inverse = 1;
  } else {
    currentPointCloud_->getLastFrameNormals(normalsDevice_);
    inverse = -1;
  }

  normalsDevice_.download (normalsHost_, c);
  
  for(int point = 0; point < (640 * 480); point++)
  {
    normalVector[point * 3 + 0] = inverse * normalsHost_[point].x;
    normalVector[point * 3 + 1] = inverse * normalsHost_[point].y;
    if((inverse * normalsHost_[point].z) > 0)
      normalVector[point * 3 + 2] = normalsHost_[point].z;
    else
      normalVector[point * 3 + 2] = inverse * normalsHost_[point].z;
  }
  
}

void Reconstruction::toggleGlasses() {
  this->toggleYawPitchRollFromGlasses();
	this->toggleXYZFromGlasses();
}

Reconstruction::~Reconstruction() {

  delete image_;
  delete tsdfVolume_;
  delete firstPointCloud_;
  delete currentPointCloud_;
  delete globalPreviousPointCloud_;

  delete [] previousDepthData;

}

