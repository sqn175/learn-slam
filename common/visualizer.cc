/*
 * @Author: Shi Qin 
 * @Date: 2017-09-22 16:18:35 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-22 17:48:04
 */

#include "visualizer.h"

#include <vector>

#include "map.h"
#include "keyframe.h"
#include "mappoint.h"

namespace lslam {

Visualizer::Visualizer(ThreadSafeQueue<VisualizedData>& queue, std::shared_ptr<Map> map)
  : frame_queue_(queue)
  , map_(map){

}

void Visualizer::Run() {
  // Opencv display image
  std::string window_name = "frame";
  cv::namedWindow(window_name);

  // Pangolin 3d visualize
  int w = 1024;
  int h = 768;
  
  pangolin::CreateWindowAndBind("Trajectory", w, h);
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("ui").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
  pangolin::Var<bool> settings_followcamera("ui.Follow Camera",true,true);
  pangolin::Var<bool> settings_showlandmarks("ui.Show Landmarks",true,true);
  pangolin::Var<bool> settings_showkeyframes("ui.Show KeyFrames",true,true);
  //pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
  //pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
  pangolin::Var<bool> menuReset("ui.Reset",false,false);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(w,h,500,500,512,389,0.1,1000),
    pangolin::ModelViewLookAt(0.0,-0.7,-1.8, 0,0,0, pangolin::AxisNegY)
    );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -(double)w/(double)h)
    .SetHandler(new pangolin::Handler3D(s_cam));

  VisualizedData vis;
  pangolin::OpenGlMatrix T_wc;
  T_wc.SetIdentity();

  while (!pangolin::ShouldQuit()) {
    if (frame_queue_.PopBlocking(&vis) == false)
      return;

    // OpenCv image display
    // Show current frame
    cv::imshow(window_name, vis.image);
    cv::waitKey(1);

    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Show current camera pose
    T_wc = GetCurrentOpenGlCameraMatrix(vis.frame);
    if (this->settings_followcamera) {
      s_cam.Follow(T_wc);
    } 

    d_cam.Activate(s_cam);
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    DrawCurrentFrame(T_wc);

    if (this->settings_showkeyframes) {
      DrawKeyFrames();
    }

    if (this->settings_showlandmarks) {
      DrawLandmarks();
    }

    pangolin::FinishFrame();

    // Update Parameters
    this->settings_followcamera = settings_followcamera.Get();
    this->settings_showkeyframes = settings_showkeyframes.Get();
    this->settings_showlandmarks = settings_showlandmarks.Get();
  }

}

pangolin::OpenGlMatrix Visualizer::GetCurrentOpenGlCameraMatrix(std::shared_ptr<Frame> frame) {
  pangolin::OpenGlMatrix M;
  M.SetIdentity();

  // We get the current OpenGL camera matrix from current frame camera pose
  cv::Mat T_wc = frame->T_wc();
  CHECK(!T_wc.empty()) << "Null camera pose.";

  cv::Mat R_wc = T_wc.rowRange(0,3).colRange(0,3);
  cv::Mat t_wc = T_wc.rowRange(0,3).col(3);

  M.m[0] = R_wc.at<double>(0,0);
  M.m[1] = R_wc.at<double>(1,0);
  M.m[2] = R_wc.at<double>(2,0);
  M.m[3]  = 0.0;

  M.m[4] = R_wc.at<double>(0,1);
  M.m[5] = R_wc.at<double>(1,1);
  M.m[6] = R_wc.at<double>(2,1);
  M.m[7]  = 0.0;

  M.m[8] = R_wc.at<double>(0,2);
  M.m[9] = R_wc.at<double>(1,2);
  M.m[10] = R_wc.at<double>(2,2);
  M.m[11]  = 0.0;

  M.m[12] = t_wc.at<double>(0);
  M.m[13] = t_wc.at<double>(1);
  M.m[14] = t_wc.at<double>(2);
  M.m[15]  = 1.0;

  return M;
}

void Visualizer::DrawCurrentFrame(pangolin::OpenGlMatrix Twc) {
  double w = 0.08;
  double h = w * 0.75;
  double z = w * 0.6;
  double line_width = 3;

  glPushMatrix();
  
  glMultMatrixd(Twc.m);

  glLineWidth(line_width);
  glColor3d(0.0, 1.0, 0.0);
  glBegin(GL_LINES);
  glVertex3d(0,0,0);
  glVertex3d(w,h,z);
  glVertex3d(0,0,0);
  glVertex3d(w,-h,z);
  glVertex3d(0,0,0);
  glVertex3d(-w,-h,z);
  glVertex3d(0,0,0);
  glVertex3d(-w,h,z);

  glVertex3d(w,h,z);
  glVertex3d(w,-h,z);

  glVertex3d(-w,h,z);
  glVertex3d(-w,-h,z);

  glVertex3d(-w,h,z);
  glVertex3d(w,h,z);

  glVertex3d(-w,-h,z);
  glVertex3d(w,-h,z);
  glEnd();

  glPopMatrix();
}

void Visualizer::DrawKeyFrames() {
  double w = 0.05;
  double h = w * 0.75;
  double z = w * 0.6;
  double KeyFrameLineWidth = 1;

  std::vector<std::shared_ptr<KeyFrame>> keyframes = map_->keyframes();
  for (auto& keyframe : keyframes) {
    cv::Mat T_wc = keyframe->T_wc().t();

    CHECK(!T_wc.empty()) << "Null T_wc";

    glPushMatrix();
    
    glMultMatrixd(T_wc.ptr<GLdouble>(0));

    glLineWidth(KeyFrameLineWidth);
    glColor3d(0.0,0.0,1.0);
    glBegin(GL_LINES);
    glVertex3d(0,0,0);
    glVertex3d(w,h,z);
    glVertex3d(0,0,0);
    glVertex3d(w,-h,z);
    glVertex3d(0,0,0);
    glVertex3d(-w,-h,z);
    glVertex3d(0,0,0);
    glVertex3d(-w,h,z);

    glVertex3d(w,h,z);
    glVertex3d(w,-h,z);

    glVertex3d(-w,h,z);
    glVertex3d(-w,-h,z);

    glVertex3d(-w,h,z);
    glVertex3d(w,h,z);

    glVertex3d(-w,-h,z);
    glVertex3d(w,-h,z);
    glEnd();

    glPopMatrix();
  }
}

void Visualizer::DrawLandmarks() {
  std::vector<std::shared_ptr<MapPoint>> landmarks = map_->mappoints();
  
  double mPointSize = 2;
  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(0.0,0.0,0.0);

  for (auto& landmark : landmarks) {
    // TODO: check if this landmark is valid
    cv::Mat pos = landmark->pt_world();
    glVertex3d(pos.at<double>(0),pos.at<double>(1),pos.at<double>(2));
  }

  glEnd();
}


} // namespace LSLAM
  
