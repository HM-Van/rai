#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

const char *USAGE=
"\n\
Usage:  kinEdit <g-filename>\n\
\n\
Iterate between editing the file (with an external editor) and\n\
viewing the model in the OpenGL window (after pressing ENTER).\n\
\n\
Use the number keys 1 2 3 4 5 to toggle display options.\n\
";


int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

    cout <<USAGE <<endl;

    rai::String file=rai::getParameter<rai::String>("file",STRING("test.ors"));
    if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
    cout <<"opening file `" <<file <<"'" <<endl;

    rai::KinematicWorld K;
    for(;;){
    Inotify ino(file);
    try {
      rai::lineCount=1;
      K.init(file);
      K.report();
      break;
    } catch(std::runtime_error& err) {
      cout <<"line " <<rai::lineCount <<": " <<err.what() <<" -- please check the file and press ENTER" <<endl;
      for(;;) {
        if(ino.poll(false, true)) break;
        rai::wait(.02);
      }
    }
    }

    K.checkConsistency();
    K >>FILE("z.g");
    //some optional manipulations
    K.optimizeTree();
    K.calc_q();
    K.checkConsistency();
    if(K.fwdActiveSet.N == K.frames.N) K.sortFrames();
    K >>FILE("z.g");
//    makeConvexHulls(G.frames);
//    computeOptimalSSBoxes(G.shapes);
//    G >>FILE("z.ors");
//    K.watch(true);
//    return;

    if(rai::checkParameter<bool>("dot")) K.displayDot();
    K.writeURDF(FILE("z.urdf"));

    if(rai::checkParameter<bool>("cleanOnly")) return 0;

    editConfiguration(file, K);

  return 0;
}
