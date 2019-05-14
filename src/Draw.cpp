

#include "Draw.h"


Draw::Draw()
{
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);
}


Draw::~Draw()
{
}

void Draw::drawText(char* textToDraw, int x, int y)
{
    /*char displayBuf[32];
    sprintf(displayBuf, "%s", textToDraw);*/

    cvPutText(DisplayImage, textToDraw, cvPoint(x, y), &font, cvScalar(255, 0, 255, 255));
}


void Draw::ShowImagesSideBySide(char* title, vector<IplImage> images, char* text, int X_textOnWindow, int Y_textOnWindow)
{
    int Ysize = images[0].height;
    int Xsize = images[0].width; 

    int numOfImages = images.size();
    int row_num = numOfImages > 2 ? (numOfImages+1) / 2 : 1;
    int col_num = numOfImages > 1 ? 2 : 1;


    DisplayImage = cvCreateImage(cvSize(Xsize * col_num, Ysize * row_num), 8, 3);

    int xStride = 0, yStride = 0;
    for (int i = 0; i < numOfImages;)
    {
        if (&images[i] == 0)
        {
            printf("Invalid arguments");
            cvReleaseImage(&DisplayImage);
            return;
        }

        xStride = (i % 2) * Xsize;
        yStride = (i / 2) * Ysize;
        cvSetImageROI(DisplayImage, cvRect(xStride, yStride, Xsize, Ysize));
        cvResize(&images[i], DisplayImage);
        cvResetImageROI(DisplayImage);

        i = i + 1;
        
    }

    drawText(text, X_textOnWindow, Y_textOnWindow);
    cvNamedWindow(title, 1);
    cvShowImage(title, DisplayImage);

    cvReleaseImage(&DisplayImage);
}


void Draw::GetDepthHistogram(cv::Mat &src, cv::Mat &dst)
{
    float depthHistogram[65536];
    int numberOfPoints = 0;
    cv::Mat depthHist(src.rows, src.cols, CV_8UC3);
    memset(depthHistogram, 0, sizeof(depthHistogram));
    for (int y = 0; y < src.rows; ++y)
    {
        ushort* depthCell = (ushort*)src.ptr<uchar>(y);
        for (int x = 0; x < src.cols; ++x)
        {
            if (*depthCell != 0)
            {
                depthHistogram[*depthCell]++;
                numberOfPoints++;
            }
            depthCell++;
        }
    }

    for (int nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
    {
        depthHistogram[nIndex] += depthHistogram[nIndex - 1];
    }
    for (int nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
    {
        depthHistogram[nIndex] = (numberOfPoints - depthHistogram[nIndex]) / numberOfPoints;
    }
    for (int y = 0; y < src.rows; ++y)
    {
        ushort* depthCell = (ushort*)src.ptr<uchar>(y);
        uchar * showcell = (uchar *)depthHist.ptr<uchar>(y);
        for (int x = 0; x < src.cols; ++x)
        {
            char depthValue = (char)(depthHistogram[*depthCell] * 255);
            *showcell++ = 0;
            *showcell++ = depthValue;
            *showcell++ = depthValue;

            depthCell++;
        }
    }
    dst = depthHist;
}
