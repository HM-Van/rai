
#include "cameraview.h"

#include <Kin/frame.h>
#include <Gui/viewer.h>

//===========================================================================

void TEST(CameraView){
  rai::KinematicWorld K;
  K.addFile("../../../../rai-robotModels/pr2/pr2.g");
  K.addFile("../../../../rai-robotModels/objects/kitchen.g");
  K.optimizeTree();

  rai::CameraView V(K, true, 2);

  V.addSensor("kinect", "endeffKinect", 640, 480, 580./480., -1., {.1, 50.} );
//  V.selectSensor("kinect");

  Var<byteA> image;
  Var<arr> depth;
  Var<byteA> segmentation;
  Var<arr> pts;

  PointCloudViewer v(pts, image);
  ImageViewer v2(segmentation);

  V.computeImageAndDepth(image.set(), depth.set());
  V.computeSegmentation(segmentation.set());
  V.computePointCloud(pts.set(), depth.get());

  rai::wait();

//  V.addCamera("default", "")

//  K.gl().camera.setKinect();
//  K.gl().camera.X = K.getFrameByName("endeffEyes")->X * K.gl().camera.X;
//  K.gl().watch(); //if commented, glut/gtk is never initiated
//  byteA indexRgb, depth;
//  K.glGetMasks(580, 480);
//  write_ppm(K.gl().captureImage, "z.rgb.ppm");
//  write_ppm(convert<byte>(255.f*K.gl().captureDepth), "z.depth.ppm");


}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testCameraView();

  return 0;
}
