#include "Image.h"
#include "image/rectangle.cpp"
#include "image/dmtx.cpp"

Image::Image() :
  showDebug(false),
  showDebugWindow(false),
  showDebugTime(false),
  debugRegions(true),
  debug_last_time((double)cv::getTickCount()) {
  showDebugWindowWaitTime=1000;
  // 28cm/2.54
  // 2048
  windowOpen=false;
  bSaveBarcodeRegions=false;
  int_saved_barcode_internal=0;
  dpi=185.7828;
}

Image::~Image() {
  //tess->End();
}



void Image::open(const char* filename){
  fileName = filename;
  try{
    cv::setUseOptimized(true);
    cv::Mat mat = cv::imread( filename, cv::IMREAD_GRAYSCALE );
    setImage(mat);
  } catch (cv::Exception& e) {
      std::cerr << "Error opening file \"" << filename << "\". Reason: " << e.msg << std::endl;
      exit(1);
  }
}

void Image::setImage(cv::Mat mat){
  originalImage = mat.clone();
  cv::cvtColor(originalImage,debugImage, cv::COLOR_GRAY2BGR);
}


cv::Mat Image::cropRotated(cv::RotatedRect rect){
  // rect is the RotatedRect (I got it from a contour...)
  // matrices we'll use
  cv::Mat M;
  cv::Mat rotated;
  cv::Mat cropped;
  // get angle and size from the bounding box
  float angle = rect.angle;
  cv::Size rect_size = rect.size;
  // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
  if (rect.angle < -45.) {
      angle += 90.0;
      cv::swap(rect_size.width, rect_size.height);
  }
  // get the rotation matrix
  M = cv::getRotationMatrix2D(rect.center, angle, 1.0);
  // perform the affine transformation
  cv::warpAffine(originalImage, rotated, M, originalImage.size(), cv::INTER_CUBIC);
  // crop the resulting image
  cv::getRectSubPix(rotated, rect_size, rect.center, cropped);
  return cropped;
}

std::vector<cv::RotatedRect> Image::regions(){

  std::vector<cv::RotatedRect> result;
  /**/  
  if (debugRegions)
    showImage();

  // compute the Scharr gradient magnitude representation of the images
	// in both the x and y direction
  cv::Mat gradX;
  cv::Mat gradY;
  cv::Mat gradient;
  cv::Mat blurred;
  cv::Mat thresh;
  cv::Mat closed;
  cv::Mat kernel;
  cv::Mat bwImage;


  int dx = 1;
  int dy = 0;
  cv::Size ksize = cv::Size(5,5);

	cv::Sobel(originalImage, gradX, CV_32F, dx , dy, -1);
  //showImage(gradX);

	cv::Sobel(originalImage, gradY, CV_32F, 0, 1, -1);
  //showImage(gradY);

  cv::subtract(gradX,gradY,gradient);
  //showImage(gradient);
  
  cv::blur(gradient,blurred,ksize);
  //showImage(blurred);


  cv::threshold(blurred,thresh,125,255,cv::THRESH_BINARY);
  //showImage(thresh);


  kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(31,31));
  //showImage(kernel);

  cv::morphologyEx(thresh,closed,cv::MORPH_CLOSE,kernel);
  if (debugRegions)
    showImage(closed);

  // perform a series of erosions and dilations
	//cv::erode(closed, closed, cv::Mat(),cv::Point(-1,-1), 24);
  std::vector<int> erodeList = {1,5,15,35,45};  
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  double erode_contours = 0;
  double dilate_contours = 0;
  double qoute = 0;

  for( size_t i = 0; i< erodeList.size()&&qoute<0.3; i++ )
  {
    cv::erode(closed, closed, cv::Mat(),cv::Point(-1,-1), erodeList.at(i));
    //if (debugRegions)
    //  showImage(closed,100);


    closed.convertTo(bwImage, CV_8UC1);
    cv::findContours(bwImage, contours, hierarchy, cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    erode_contours = (double)contours.size();
    std::cout << "contours: " << contours.size() << std::endl;

    //cv::dilate(closed, closed, cv::Mat(),cv::Point(-1,-1), 50);
    cv::dilate(closed, closed, cv::Mat(),cv::Point(-1,-1), 20);
    //if (debugRegions)
    // showImage(closed,100);



    closed.convertTo(bwImage, CV_8UC1);

    cv::findContours(bwImage, contours, hierarchy, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<cv::RotatedRect> minRect( contours.size() );

    dilate_contours = (double)contours.size();
    std::cout << "contours: " << contours.size() << std::endl;
    std::cout << "erode: " << erodeList.at(i)<< std::endl;
    std::cout << "qoute: " << dilate_contours/erode_contours << std::endl;
    if ((dilate_contours==1) && (erode_contours==1)){
    qoute = 0;
    }else{
    qoute = dilate_contours/erode_contours;
    }
  }

  for( size_t i = 0; i< contours.size(); i++ )
  {
    cv::Scalar blue_color = cv::Scalar(255.0, 0.0, 0.0); 
    cv::Scalar green_color = cv::Scalar(0.0, 255.0, 0.0);
    cv::RotatedRect rotatedRect = cv::minAreaRect(contours.at(i));
    cv::Rect boundingRect = cv::boundingRect(contours.at(i));

    cv::Point2f vertices2f[4];
    rotatedRect.points(vertices2f);
    
    cv::Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }


    cv::Point2f rect_points[4]; 
    rotatedRect.points( rect_points );
    for( int j = 0; j < 4; j++ )
      cv::line( debugImage, rect_points[j], rect_points[(j+1)%4], blue_color, 3, 8 );

    /*
    cv::fillConvexPoly(debugImage,
                    vertices,
                    4,
                    blue_color);
    */
    
    cv::Point inflationPoint(-20, -20);
    cv::Size inflationSize(40, 40);

    boundingRect += inflationPoint;
    boundingRect += inflationSize;
    if (boundingRect.x<0)boundingRect.x=0;
    if (boundingRect.y<0)boundingRect.y=0;
    if (boundingRect.x+boundingRect.width>originalImage.cols){
      boundingRect.width -= boundingRect.x+boundingRect.width- originalImage.cols;
    }
    if (boundingRect.y+boundingRect.height>originalImage.rows){
      boundingRect.height -= boundingRect.y+boundingRect.height- originalImage.rows;
    }

    cv::rectangle(debugImage,boundingRect,green_color,3);
    result.push_back(rotatedRect);
    interestingRects.push_back(boundingRect);
  }

  showDebugWindowWaitTime=5000;
  //if (debugRegions)
  showImage(debugImage,3000);
  return result;
}

void Image::barcode(){
  _debugTime("start barcode");
  std::vector<cv::RotatedRect> regions = this->regions();
  /*
  for( size_t i = 0; i< regions.size(); i++ )
  {
    cv::Mat img = cropRotated(regions.at(i));
    std::cout << "X (inch): " << img.cols/dpi << std::endl;
    std::cout << "Y (inch): " << img.rows/dpi << std::endl;
    std::cout << "Area (inch): " << (img.cols/dpi * img.rows/dpi)  << std::endl;
    barcode_internal(img);
    showImage(img,1000);
  }
  */
  std::vector<int> blurSizes = {0,};
  for( size_t b = 0; b< blurSizes.size(); b++ )
  {
    cv::Mat bimg = originalImage.clone();
    if (blurSizes.at(b)>0){
      cv::Size ksize = cv::Size(blurSizes.at(b),blurSizes.at(b));
      cv::blur(bimg,bimg,ksize);
    }
  
    for( size_t i = 0; i< interestingRects.size(); i++ )
    {
      cv::Mat img = bimg(interestingRects.at(i));
      /*
      std::cout << "X (inch): " << img.cols/dpi << std::endl;
      std::cout << "Y (inch): " << img.rows/dpi << std::endl;
      std::cout << "Area (inch): " << (img.cols/dpi * img.rows/dpi)  << std::endl;
      */
      barcode_internal(img);
      /*std::cout << "Codes: " << "B("<< blurSizes.at(b) << ")" << " ";
      for (std::vector<std::string>::iterator ubarcodes = codes.begin(); ubarcodes != codes.end(); ++ubarcodes) {
          std::cout << *ubarcodes << " ";
      }*/
      //cv::Mat imgdm = bimg(interestingRects.at(i));
      //datamatrix(imgdm);
    }
 
  }
  _debugTime("stop barcode");
  _debugTime("start dm barcode");
  std::vector<cv::RotatedRect> dmregions = this->dmregions();

  for( size_t i = 0; i< interestingDmRects.size(); i++ )
    {
      cv::Mat img = originalImage(interestingDmRects.at(i));
      /*
      std::cout << "X (inch): " << img.cols/dpi << std::endl;
      std::cout << "Y (inch): " << img.rows/dpi << std::endl;
      std::cout << "Area (inch): " << (img.cols/dpi * img.rows/dpi)  << std::endl;
      */
     showImage(img,2000);
      datamatrix(img);
      /*std::cout << "Codes: " << "B("<< blurSizes.at(b) << ")" << " ";
      for (std::vector<std::string>::iterator ubarcodes = codes.begin(); ubarcodes != codes.end(); ++ubarcodes) {
          std::cout << *ubarcodes << " ";
      }*/
      //cv::Mat imgdm = bimg(interestingRects.at(i));
      //datamatrix(imgdm);
    }

  _debugTime("stop dm barcode");

  std::cout << "(" << fileName << ")" <<  " Codes: ";
  for (std::vector<std::string>::iterator ubarcodes = codes.begin(); ubarcodes != codes.end(); ++ubarcodes) {
      std::cout << *ubarcodes << " ";
  }
  std::cout << std::endl;

}

void Image::barcode_internal_zbar(cv::Mat grayo){
  std::string code;

  zbar::Image* _image;
  zbar::ImageScanner* _imageScanner;
  _image = new zbar::Image(grayo.cols, grayo.rows, "Y800", nullptr, 0);
  _imageScanner = new zbar::ImageScanner();
  _imageScanner->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  
  _imageScanner->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  _imageScanner->set_config(zbar::ZBAR_CODE128, zbar::ZBAR_CFG_ENABLE, 1);
  _imageScanner->set_config(zbar::ZBAR_CODE128, zbar::ZBAR_CFG_ADD_CHECK, 1);
  _imageScanner->set_config(zbar::ZBAR_CODE128, zbar::ZBAR_CFG_EMIT_CHECK, 0);


  _imageScanner->set_config(zbar::ZBAR_CODE39, zbar::ZBAR_CFG_ENABLE, 1);
  _imageScanner->set_config(zbar::ZBAR_CODE39, zbar::ZBAR_CFG_ADD_CHECK, 0);
  _imageScanner->set_config(zbar::ZBAR_CODE39, zbar::ZBAR_CFG_EMIT_CHECK, 0);

  _imageScanner->set_config(zbar::ZBAR_I25, zbar::ZBAR_CFG_ENABLE, 1);
  _imageScanner->set_config(zbar::ZBAR_I25, zbar::ZBAR_CFG_ADD_CHECK, 1);
  _imageScanner->set_config(zbar::ZBAR_I25, zbar::ZBAR_CFG_EMIT_CHECK, 0);

  _image->set_data((uchar *)grayo.data, grayo.cols * grayo.rows);
    
  _imageScanner->scan(*_image);
  for(zbar::Image::SymbolIterator symbol = _image->symbol_begin(); symbol != _image->symbol_end(); ++symbol) {
    code = std::string(symbol->get_data().c_str());
    std::string type = std::string(symbol->get_type_name().c_str());

    mutex.lock();
    codes.push_back(code);
    mutex.unlock();

  }
}

void Image::setSaveBarcodeRegions(bool val){
  bSaveBarcodeRegions=val;
}

void Image::barcode_internal_thread(cv::Mat &part,int bsize,int means) {
  cv::Mat grayo;

  cv::adaptiveThreshold(part, grayo, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, bsize, means);
	//cv::dilate(grayo, grayo, cv::Mat(),cv::Point(-1,-1), 1);
  //cv::erode(grayo, grayo, cv::Mat(),cv::Point(-1,-1), 1);
  //mutex.lock();
  //showImage(grayo,1);
  //mutex.unlock();
  barcode_internal_zbar(grayo);
}

void Image::barcode_internal(cv::Mat &part) {
  if (part.channels()>1){
    throw std::runtime_error("Error: Image::barcode_internal not a gray image");
  }
  /*
  cv::Scalar mean = cv::mean(part);
  mean[0]=mean[0]*1.1;
  if (mean[0]>255)mean[0]=255;
  mean[1]=mean[0];
  mean[2]=mean[0];
  cv::Mat M(part.rows*1.2,part.cols*1.2, CV_8UC1,mean);
  int y_start = part.rows*0.1+20;
  int x_start = part.cols*0.1-1;
  if (y_start<0)y_start=0;
  if (x_start<0)x_start=0;
  
  part.copyTo( M(  ));
  part=M.clone();
  */
  
  if (bSaveBarcodeRegions){
    std::vector<int> params;
    std::string name = std::string(fileName);
    name = boost::str( boost::format("%s.%s.jpg") % name % int_saved_barcode_internal  );
    int_saved_barcode_internal++;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    cv::imwrite(name.c_str(),part,params);
  }

  std::vector<int> bsizes = {3,57,81,151,181,201,221};
  std::vector<int> smeans = {3,5,9,19,37,55};
  std::vector<boost::thread*> threads;
  barcode_internal_zbar(part);
  
  for( size_t ms = 0; ms< smeans.size();ms++ )
  {
    for( size_t bs = 0; bs< bsizes.size(); bs++ )
    {
      int bsize = bsizes.at(bs);
      int means = smeans.at(ms);
      if (showDebugWindow){
        barcode_internal_thread( part,bsize,means );
      }else{
        threads.push_back(new boost::thread(&Image::barcode_internal_thread, this , part,bsize,means));
      } 
    }
  }

  for( size_t ms = 0; ms< threads.size();ms++ )
  {
    threads.at(ms)->join();
  }

  std::vector<std::string>::iterator ubarcodes = std::unique(codes.begin(), codes.end());
  codes.resize(std::distance(codes.begin(), ubarcodes));
}


/*DEBUG*/
void Image::showImage(){
  showImage(debugImage,showDebugWindowWaitTime);
}


void Image::showImage(cv::Mat& src){
  showImage(src,showDebugWindowWaitTime);
}

void Image::showImage(cv::Mat& src,int ww){
  if (showDebugWindow){
    cv::Mat r=src.clone();
    if (src.cols>600){
      int x=src.cols /5;
      int y=src.rows /5;
      //cv::Mat res = cv::Mat(x, y, CV_8UC1);
      cv::resize(r, r, cv::Size(x, y), 0, 0, 3);
    }
    cv::namedWindow("DEBUG", cv::WINDOW_NORMAL );
    cv::imshow("DEBUG", r );
    cv::resizeWindow("DEBUG", 600,1000);
    cv::waitKey(ww);
  }
}


void Image::setDebug(bool val){
  showDebug=val;
}

void Image::setDebugWindow(bool val){
  showDebugWindow=val;
}

void Image::setDebugTime(bool val){
  showDebugTime=val;
}

void Image::_debugTime(std::string str){
  if (showDebugTime){
    double time_since_last = ((double)cv::getTickCount() - debug_last_time)/cv::getTickFrequency();
    std::cout << "(ImageRecognizeEx::_debugTime)\t" << str << ": " << time_since_last << "s " << std::endl;
  }
  debug_last_time = (double)cv::getTickCount();
}

/*DEBUG*/







std::vector<cv::RotatedRect> Image::dmregions(){

  std::vector<cv::RotatedRect> result;
  /**/  
  debugImage = originalImage.clone();
  if (debugRegions)
    showImage();

  // compute the Scharr gradient magnitude representation of the images
	// in both the x and y direction
  cv::Mat gradX;
  cv::Mat gradY;
  cv::Mat gradient;
  cv::Mat blurred;
  cv::Mat thresh;
  cv::Mat closed;
  cv::Mat kernel;
  cv::Mat bwImage;


  int dx = 1;
  int dy = 0;
  cv::Size ksize = cv::Size(3,3);

  std::cout << "------------------" << std::endl;

	cv::Sobel(originalImage, gradX, CV_32F, dx , dy, -1);
  //showImage(gradX);

	cv::Sobel(originalImage, gradY, CV_32F, 0, 1, -1);
  //showImage(gradY);

  cv::subtract(gradX,gradY,gradient);
  //showImage(gradient);
  
  cv::blur(gradient,blurred,ksize);
  //showImage(blurred);


  cv::threshold(blurred,thresh,125,255,cv::THRESH_BINARY);
  //showImage(thresh);


  kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
  //showImage(kernel);

  cv::morphologyEx(thresh,closed,cv::MORPH_CLOSE,kernel);
  if (debugRegions)
    showImage(closed);

  // perform a series of erosions and dilations
	//cv::erode(closed, closed, cv::Mat(),cv::Point(-1,-1), 24);
  std::vector<int> erodeList = {1,5,15,35,45};  
  std::vector<int> dilateList = {1,3,5,10,15};  
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  double erode_contours = 0;
  double dilate_contours = 0;
  double qoute = 0;

  for( size_t i = 0; i< erodeList.size()&&qoute<0.5; i++ )
  {
  for( size_t di = 0; di< dilateList.size()&&qoute<0.5; di++ )
  {
    cv::erode(closed, closed, cv::Mat(),cv::Point(-1,-1), erodeList.at(i));
    if (debugRegions)
      showImage(closed,100);


    closed.convertTo(bwImage, CV_8UC1);
    cv::findContours(bwImage, contours, hierarchy, cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    erode_contours = (double)contours.size();
    std::cout << "contours: " << contours.size() << std::endl;

    //cv::dilate(closed, closed, cv::Mat(),cv::Point(-1,-1), 50);
    cv::dilate(closed, closed, cv::Mat(),cv::Point(-1,-1), dilateList.at(di));
    if (debugRegions)
      showImage(closed,100);



    closed.convertTo(bwImage, CV_8UC1);

    cv::findContours(bwImage, contours, hierarchy, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<cv::RotatedRect> minRect( contours.size() );

    dilate_contours = (double)contours.size();
    std::cout << "contours: " << contours.size() << std::endl;
    std::cout << "erode: " << erodeList.at(i)<< std::endl;
    std::cout << "qoute: " << dilate_contours/erode_contours << std::endl;
    if ((dilate_contours==1) && (erode_contours==1)){
    qoute = 0;
    }else{
    qoute = dilate_contours/erode_contours;
    }
  }
  }

  for( size_t i = 0; i< contours.size(); i++ )
  {
    cv::Scalar blue_color = cv::Scalar(255.0, 0.0, 0.0); 
    cv::Scalar green_color = cv::Scalar(0.0, 255.0, 0.0);
    cv::RotatedRect rotatedRect = cv::minAreaRect(contours.at(i));
    cv::Rect boundingRect = cv::boundingRect(contours.at(i));

    cv::Point2f vertices2f[4];
    rotatedRect.points(vertices2f);
    
    cv::Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }


    cv::Point2f rect_points[4]; 
    rotatedRect.points( rect_points );
    for( int j = 0; j < 4; j++ )
      cv::line( debugImage, rect_points[j], rect_points[(j+1)%4], blue_color, 3, 8 );

    /*
    cv::fillConvexPoly(debugImage,
                    vertices,
                    4,
                    blue_color);
    */
    
    cv::Point inflationPoint(-20, -20);
    cv::Size inflationSize(40, 40);

    boundingRect += inflationPoint;
    boundingRect += inflationSize;
    if (boundingRect.x<0)boundingRect.x=0;
    if (boundingRect.y<0)boundingRect.y=0;
    if (boundingRect.x+boundingRect.width>originalImage.cols){
      boundingRect.width -= boundingRect.x+boundingRect.width- originalImage.cols;
    }
    if (boundingRect.y+boundingRect.height>originalImage.rows){
      boundingRect.height -= boundingRect.y+boundingRect.height- originalImage.rows;
    }


    double ratio = (double)boundingRect.width / (double)boundingRect.height;
    double one_cm = 163.0;
    if ( (ratio>0.5) && (ratio<1.5)){
      if ( ((boundingRect.width/one_cm)>0.8) && ((boundingRect.width/one_cm)<2) ){
      std::cout << "contour size " << boundingRect.width << "*" << boundingRect.height << " r " << ratio <<  " cm " << (boundingRect.width/one_cm) <<"*"<<  (boundingRect.height/one_cm) << std::endl;
      cv::rectangle(debugImage,boundingRect,green_color,3);
        result.push_back(rotatedRect);
        interestingDmRects.push_back(boundingRect);
      }
    }
  }

  showDebugWindowWaitTime=5000;
  //if (debugRegions)
  showImage(debugImage,3000);
  return result;
}



void Image::datamatrix(cv::Mat &image){
 

  std::cout << "X: " << image.cols << "Y: " << image.rows << std::endl;

std::vector<int> bsizes = {3,7,15,57,81,151,181,201,221};
  std::vector<int> smeans = {1,3,5,9,19,37,55,85};
  std::vector<boost::thread*> threads;
  datamatrix_internal(image);
  
  for( size_t ms = 0; ms< smeans.size();ms++ )
  {
    for( size_t bs = 0; bs< bsizes.size(); bs++ )
    {
      int bsize = bsizes.at(bs);
      int means = smeans.at(ms);
      if (showDebugWindow){
        dm_barcode_internal_thread( image,bsize,means );
      }else{
//        threads.push_back(new boost::thread(&Image::dm_barcode_internal_thread, this , image,bsize,means));
      } 
    }
  }
/*
  for( size_t ms = 0; ms< threads.size();ms++ )
  {
    threads.at(ms)->join();
  }
*/
  std::vector<std::string>::iterator ubarcodes = std::unique(codes.begin(), codes.end());
  codes.resize(std::distance(codes.begin(), ubarcodes));
}

void Image::dm_barcode_internal_thread(cv::Mat &part,int bsize,int means) {
  cv::Mat grayo;

  cv::adaptiveThreshold(part, grayo, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, bsize, means);
	//cv::dilate(grayo, grayo, cv::Mat(),cv::Point(-1,-1), 1);
  //cv::erode(grayo, grayo, cv::Mat(),cv::Point(-1,-1), 1);
  //mutex.lock();
  showImage(grayo,1);
  //mutex.unlock();
  datamatrix_internal(grayo);
}

void Image::datamatrix_internal(cv::Mat &image){
   DmtxTime timeout;
  DmtxImage *img;
  DmtxDecode *dec;
  DmtxRegion *reg;
  DmtxMessage *msg;
  UserOptions opt;

  int err;
  int imgScanCount, pageScanCount;

    imgScanCount = 0;
    pageScanCount = 0;
    opt = GetDefaultOptions();
    img = dmtxImage(image);


    dmtxImageSetProp(img, DmtxPropImageFlip, DmtxFlipNone);

    /* Initialize scan */
    dec = dmtxDecodeCreate(img, opt.shrinkMin);
    if(dec == NULL) {
      //CleanupMagick(&wand, DmtxFalse);
      //FatalError(EX_SOFTWARE, "decode create error");
    }

    err = SetDecodeOptions(dec, img, &opt);
    if(err != DmtxPass) {
      //CleanupMagick(&wand, DmtxFalse);
      //FatalError(EX_SOFTWARE, "decode option error");
    }

    /* Find and decode every barcode on page */
    pageScanCount = 0;
    for(;;) {
      /* Find next barcode region within image, but do not decode yet */
      if(opt.timeoutMS == DmtxUndefined)
          reg = dmtxRegionFindNext(dec, NULL);
      else
          reg = dmtxRegionFindNext(dec, &timeout);

      /* Finished file or ran out of time before finding another region */
      if(reg == NULL)
          break;

      /* Decode region based on requested barcode mode */
      if(opt.mosaic == DmtxTrue)
          msg = dmtxDecodeMosaicRegion(dec, reg, opt.correctionsMax);
      else
          msg = dmtxDecodeMatrixRegion(dec, reg, opt.correctionsMax);

      if(msg != NULL) {
          //PrintStats(dec, reg, msg, imgPageIndex, &opt);
          //PrintMessage(reg, msg, &opt);

          std::string decodedText = std::string((char*) msg->output);
          std::cout << ">" << decodedText << std::endl;
          codes.push_back(decodedText);
          /*
          for(i = 0; i < msg->codeSize; i++) {
            std::cout << "" << msg->code[i];
          } 
          */
          //std::cout << "" << msg->code[i] << std::endl;
          pageScanCount++;
          imgScanCount++;
          dmtxMessageDestroy(&msg);
      }
      dmtxRegionDestroy(&reg);
    }
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&img);
}



/**
 * Converts an OpenCV image to a DmtxImage.
 *
 * NOTE: The caller must detroy the image.
 */
DmtxImage* Image::dmtxImage(cv::Mat &image) {
    if (image.type() != CV_8UC1) {
        throw std::logic_error("invalid image type");// + image.type());
    }
    DmtxImage * dmtxImage = dmtxImageCreate( image.data, image.cols, image.rows, DmtxPack8bppK);
    dmtxImageSetProp(dmtxImage, DmtxPropRowPadBytes, image.step1() - image.cols);
    return dmtxImage;
}
