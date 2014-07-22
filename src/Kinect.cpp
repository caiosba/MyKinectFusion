#include "Kinect.h"

Kinect::Kinect(bool live, char *onifile)
{
  try {
		if (live) {
	    capture = new pcl::OpenNIGrabber();
		}
	  
    else {
      bool triggered_capture = false;
      bool repeat = true;
      capture = new pcl::ONIGrabber(onifile, repeat, !triggered_capture);
	  }

	  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> callbackFunction
	  	= boost::bind (&Kinect::imageCallBack, this, _1, _2, _3);
	  capture->registerCallback(callbackFunction);
	  capture->start();
  }
	catch (const pcl::PCLException&) { std::cout << "Can't open depth source" << std::endl, -1; }

}

Kinect::~Kinect()
{
	capture->stop();
	delete capture;
}

bool Kinect::grabFrame()
{
	boost::unique_lock<boost::mutex> lock(data_ready_mutex);
	return data_ready_cond.timed_wait (lock, boost::posix_time::millisec(100));
}

void Kinect::imageCallBack (const boost::shared_ptr<openni_wrapper::Image>& rgbImage, const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage, float constant) {

	boost::mutex::scoped_try_lock lock(data_ready_mutex);

	if (!lock)
		return;

	this->rgbImage = rgbImage;
	this->depthImage = depthImage;

	data_ready_cond.notify_one();

}
