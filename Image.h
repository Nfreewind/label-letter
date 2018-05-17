// Image.h
#ifndef IMAGE_H
#define IMAGE_H




#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include <list>
#include <vector>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>


#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/asio.hpp>
#include <boost/thread/scoped_thread.hpp>


#include <leptonica/allheaders.h>
#include <tesseract/baseapi.h>
#include <tesseract/strngs.h>
#include <zbar.h>



#include <dmtx.h>


class Image{
public:
  Image();
  ~Image();

  void open(const char* filename);
  void setImage(cv::Mat mat);
  void setSaveBarcodeRegions(bool val);
  void barcode();
  std::vector<cv::RotatedRect> regions();
  std::vector<cv::RotatedRect> dmregions();
  cv::Mat cropRotated(cv::RotatedRect rect);
  void barcode_internal(cv::Mat &part);
  void barcode_internal_thread(cv::Mat &part,int bsize,int means);
  void dm_barcode_internal_thread(cv::Mat &part,int bsize,int means);
  void barcode_internal_zbar(cv::Mat grayo);

  DmtxImage* dmtxImage(cv::Mat &image);
  std::vector<cv::Mat> findDMTXRectangles(cv::Mat &gray);
  void datamatrix(cv::Mat &image);
  void datamatrix_internal(cv::Mat &image);

  /* DEBUG */
  void showImage();
  void showImage(cv::Mat& src);
  void showImage(cv::Mat& src,int ww);
  void setDebug(bool val);
  void setDebugWindow(bool val);
  void setDebugTime(bool val);
  void _debugTime(std::string str);
  /* DEBUG */
  

private:
  cv::Mat originalImage;
  cv::Mat debugImage;

  int showDebugWindowWaitTime;
  bool showDebug;
  bool showDebugWindow;
  bool showDebugTime;
  bool debugRegions;
  double debug_last_time;
  double dpi;
  const char* fileName;

  std::vector<std::string> codes;
  std::vector<cv::Rect> interestingRects;
  std::vector<cv::Rect> interestingDmRects;
  boost::mutex mutex;

  bool windowOpen;
  bool bSaveBarcodeRegions;
  int int_saved_barcode_internal;
};
#endif
