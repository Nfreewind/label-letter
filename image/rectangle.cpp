std::vector<cv::Mat> Image::findDMTXRectangles(cv::Mat &gray){
    std::vector<cv::Mat> results={};
    // compute mask (you could use a simple threshold if the image is always as good as the one you provided)
    cv::Mat mask;
    cv::Mat tgray;


    cv::Size ksize(15,15);
    cv::GaussianBlur(gray, tgray, ksize, 0);

/*
    adaptiveThreshold(tgray, mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 85, 1); 
    dilate(mask, mask, 0, cv::Point(-1, -1), 2, 1, 1);
*/

    cv::threshold(tgray, mask, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);

    showImage(mask);

    // find contours (if always so easy to segment as your image, you could just add the black/rect pixels to a vector)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    
//  cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(mask,contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    /// Draw contours and find biggest contour (if there are other contours in the image, we assume the biggest one is the desired rect)
    // drawing here is only for demonstration!
    int biggestContourIdx = -1;
    float biggestContourArea = 0;
    cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(0, 100, 0);
        drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );

        float ctArea= cv::contourArea(contours[i]);
        
        if ((ctArea > 1.2)&&ctArea*2<(gray.cols*gray.rows)){
            if(ctArea > biggestContourArea)
            {
                biggestContourArea = ctArea;
                biggestContourIdx = i;
            }
            
            // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
            cv::RotatedRect boundingBox = cv::minAreaRect(contours[i]);



            
            // one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines
            // draw the rotated rect
            cv::Point2f corners[4];
            boundingBox.points(corners);
            cv::line(drawing, corners[0], corners[1], cv::Scalar(255,255,255));
            cv::line(drawing, corners[1], corners[2], cv::Scalar(255,255,255));
            cv::line(drawing, corners[2], corners[3], cv::Scalar(255,255,255));
            cv::line(drawing, corners[3], corners[0], cv::Scalar(255,255,255));
            
            try{
                cv::Rect brect = boundingBox.boundingRect();

                /*
                std::cout << " brect.x " << brect.x << std::endl;
                std::cout << " brect.y " << brect.y << std::endl;
                std::cout << " brect.width " << brect.width << std::endl;
                std::cout << " brect.height " << brect.height << std::endl;
                */
                if (brect.x<0) brect.x = 0;
                if (brect.y<0) brect.y = 0;
                if (brect.width>=gray.cols)  brect.width=gray.cols;
                if (brect.height>=gray.rows) brect.height=gray.rows;

                
                /*
                if (0 < brect.x || 0 <= brect.width || brect.x + brect.width > gray.cols || 0 < brect.y ||  brect.y + brect.height > gray.rows){

                }else{
                */    
                results.push_back(gray(brect));
                //}
            }catch(cv::Exception cv_error){
                // std::cerr << "findDMTXRectangles()" << cv_error.msg << std::endl;
            }
        }
    }

    // if no contour found
    if(biggestContourIdx < 0)
    {
        std::cout << "no contour found" << std::endl;
        return results;
    }

    
    // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
    cv::RotatedRect boundingBox = cv::minAreaRect(contours[biggestContourIdx]);
    // one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines


    // draw the rotated rect
    cv::Point2f corners[4];
    boundingBox.points(corners);
    cv::line(drawing, corners[0], corners[1], cv::Scalar(255,255,255));
    cv::line(drawing, corners[1], corners[2], cv::Scalar(255,255,255));
    cv::line(drawing, corners[2], corners[3], cv::Scalar(255,255,255));
    cv::line(drawing, corners[3], corners[0], cv::Scalar(255,255,255));

   
    showImage(drawing,1000);
    
    return results;
}