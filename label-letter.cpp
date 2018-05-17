#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/chrono.hpp>
#include <ctime>

#include <sys/time.h>
#include <stdio.h>
#include <cstdlib>
#include <fstream>

#include <time.h>

#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pthread.h>
#include <vector>
#include <string>

#include "args.hxx"

#include "Image.cpp"


#ifdef HAVE_OPENCL
#include <opencv2/core/ocl.hpp>
#endif



boost::format isotimeformat("%04d-%02d-%02d %02d:%02d:%02d");
boost::format insertfile("insert into sv_images_extended (id,createdate,server,filename) values ('%s','%s','%s','%s') on duplicate key update id=values(id), createdate=values(createdate), server=values(server), filename=values(filename) ");
boost::format insertcode("insert into sv_images_extended_codes (id,code) values ('%s','%s') on duplicate key update code=values(code) ");

std::string version="1.0.001";


double debug_start_time = (double)cv::getTickCount();
double debug_last_time = (double)cv::getTickCount();
double debug_window_offset = 0;

bool bDebugTime=false;
void debugTime(std::string str){
  if (bDebugTime){
    double time_since_start = ((double)cv::getTickCount() - debug_start_time)/cv::getTickFrequency();
    double time_since_last = ((double)cv::getTickCount() - debug_last_time)/cv::getTickFrequency();
    std::cout << str << ": " << time_since_last << "s " << "(total: " << time_since_start  << "s)" << std::endl;
  }
  debug_last_time = (double)cv::getTickCount();
}

int main( int argc, char** argv ){


  args::ArgumentParser parser("Ocrs reconize barcodes and text.", "Take care depending on speed Pixel per CM Y can vary");
  args::HelpFlag help(parser, "help", "Display this help menu", { "help"});

  args::Flag debug(parser, "debug", "Show debug messages", {'d', "debug"});
  args::Flag debugwindow(parser, "debugwindow", "Show debug window", {'w', "debugwindow"});
  args::Flag debugtime(parser, "debugtime", "Show times", {'t', "debugtime"});
  args::Flag disableopencl(parser, "disableopencl", "disable opencl", {"disableopencl"});
  args::ValueFlag<std::string> filename(parser, "filename", "The filename", {'f',"file"});

  args::Flag save_barcode_regions(parser, "save_barcode_regions", "save every found rgion as image", {"save_barcode_regions"});
  

  try
  {
      parser.ParseCLI(argc, argv);
      if (filename==0){
        std::cout << parser;
        return 0;
      }
  }
  catch (args::Help)
  {

      std::cout << parser;
      return 0;
  }
  catch (args::ParseError e)
  {
      std::cerr << e.what() << std::endl;
      std::cerr << parser;
      return 1;
  }
  if (debug){
    std::cout << "processing image: " << args::get(filename) << std::endl;
  }


  bDebugTime = (debugtime==1);
  debugTime("Start");

  if (disableopencl==1){
    #ifdef HAVE_OPENCL
    cv::ocl::setUseOpenCL(false);
    #endif
  }




  Image* im=new Image();
  im->setDebug(debug);
  im->setDebugTime(bDebugTime);
  im->setDebugWindow(debugwindow==1);
  im->setSaveBarcodeRegions(save_barcode_regions==1);
  im->open((args::get(filename)).c_str());
  im->barcode();
  

  debugTime("Stop");

  return 0;
}
