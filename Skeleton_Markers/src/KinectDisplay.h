#ifndef KINECT_DISPLAY_
#define KINECT_DISPLAY_

#include <XnCppWrapper.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


void kinect_display_drawDepthMapGL(const xn::DepthMetaData& dmd,
                                   const xn::SceneMetaData& smd);
                                   
void kinect_display_drawSkeletonGL(xn::UserGenerator& userGenerator,
                                   xn::DepthGenerator& depthGenerator);

cv::Mat get_Joint_position();
                                   
#endif
