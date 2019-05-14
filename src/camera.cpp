/******************************************************
获取彩色和深度图像以及点云的类
******************************************************/

#include "camera.h"

using namespace std;
using namespace openni;
using namespace nite;
//using namespace pcl;

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

void Camera::CameraInit()
{
	// 初始化OpenNI环境
    OpenNI::initialize();

    // 声明并打开Device设备，我用的是Kinect。
    mdevAnyDevice_.open(ANY_DEVICE );

    // 创建深度数据流
    mDepthStream_.create( mdevAnyDevice_, SENSOR_DEPTH );

    // 创建彩色图像数据流
    mColorStream_.create( mdevAnyDevice_, SENSOR_COLOR );

    // 设置深度图像视频模式
    VideoMode mModeDepth;
    // 分辨率大小
    mModeDepth.setResolution( 640, 480 );
    // 每秒30帧
    mModeDepth.setFps( 30 );
    // 像素格式
    mModeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );

    mDepthStream_.setVideoMode( mModeDepth);

    // 同样的设置彩色图像视频模式
    VideoMode mModeColor;
    mModeColor.setResolution( 640, 480 );
    mModeColor.setFps( 30 );
    mModeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );

    mColorStream_.setVideoMode( mModeColor);

	mDepthStream_.setMirroringEnabled(false);
	mColorStream_.setMirroringEnabled(false);

    // 图像模式注册
    if( mdevAnyDevice_.isImageRegistrationModeSupported( IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        mdevAnyDevice_.setImageRegistrationMode( IMAGE_REGISTRATION_OFF );
    }
        
    // 打开深度和图像数据流
    mDepthStream_.start();
    mColorStream_.start();

 	// 为了得到骨骼数据，先初始化NiTE
//    NiTE::initialize();
    
//    mUserTracker_.create( &mdevAnyDevice_ );
//    mUserTracker_.setSkeletonSmoothingFactor( 0.1f );
}

//默认构造函数
Camera::Camera()
{
    bSave_Pointcloud_ = false;
    bSave_ColorPointcloud_ = false;
    bSave_CloudFile_ = false;
    bSave_Image_ = false;
    bSave_Map_ = false;
    bSave_MapFile_ = false;
	m_W = 640;
	m_H = 480;
    //初始化
	CameraInit();

    LK_Mat << fxl,  0.0,  cxl,  0.0,
            0.0,  fyl,  cyl,  0.0,
            0.0,  0.0,  1.0,  0.0,
            0.0,  0.0,  0.0,  1.0;

    RK_Mat << fxr,  0.0,  cxr,  0.0,
            0.0,  fyr,  cyr,  0.0,
            0.0,  0.0,  1.0,  0.0,
            0.0,  0.0,  0.0,  1.0;

    R2L_Mat << dsf[0][0], dsf[0][1], dsf[0][2], dsf[0][3],
            dsf[1][0], dsf[1][1], dsf[1][2], dsf[1][3],
            dsf[2][0], dsf[2][1], dsf[2][2], dsf[2][3],
            0.0,    0.0,    0.0,    1.0;

    All_Mat = RK_Mat * R2L_Mat * LK_Mat.inverse();

    d2c[0] = All_Mat(0, 0);
    d2c[1] = All_Mat(0, 1);
    d2c[2] = All_Mat(0, 2);
    d2c[3] = All_Mat(0, 3);
    d2c[4] = All_Mat(1, 0);
    d2c[5] = All_Mat(1, 1);
    d2c[6] = All_Mat(1, 2);
    d2c[7] = All_Mat(1, 3);
    d2c[8] = All_Mat(2, 0);
    d2c[9] = All_Mat(2, 1);
    d2c[10] = All_Mat(2, 2);
    d2c[11] = All_Mat(2, 3);
    d2c[12] = All_Mat(3, 0);
    d2c[13] = All_Mat(3, 1);
    d2c[14] = All_Mat(3, 2);
    d2c[15] = All_Mat(3, 3);

//	std::cout << "LK_Mat" << std::endl << LK_Mat << std::endl;
//	std::cout << "RK_Mat" << std::endl << RK_Mat << std::endl;
//	std::cout << "R2L_Mat" << std::endl << R2L_Mat << std::endl;
//	std::cout << "All_Mat" << std::endl << All_Mat << std::endl;

}
Camera::~Camera()
{
	cout << "Close Camera" << endl;
	mDepthFrame_.release();
	mColorFrame_.release();
//	mUserFrame_.release();
    // 先销毁User跟踪器
//    mUserTracker_.destroy();
    // 销毁彩色数据流和深度数据流
    mColorStream_.destroy();
    mDepthStream_.destroy();

    // 关闭Kinect设备
    mdevAnyDevice_.close();
    // 关闭NITE和OpenNI环境
//    NiTE::shutdown();
    OpenNI::shutdown(); 
    cout << "Finish Process!" << endl;
}
bool  Camera::toggleRegister(bool setRegister)
{
    if (mdevAnyDevice_.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        if (setRegister){
            mdevAnyDevice_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        }
        else{
            mdevAnyDevice_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
        }

        return true;
    }
    else
    {
        return false;
    }
}

cv::Mat Camera::ShowColor()
{
	int changedStreamDummy;
	openni::VideoStream* pStream = &mColorStream_;
	openni::Status rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
//		cv::namedWindow( "Color",  CV_WINDOW_AUTOSIZE );
	// 读取彩色图像数据帧信息流
	mColorStream_.readFrame( &mColorFrame_);

	// 将彩色数据流转换为OpenCV格式，记得格式是：CV_8UC3（含R\G\B）
	const cv::Mat mImageRGB( mColorFrame_.getHeight(), mColorFrame_.getWidth(),
							 CV_8UC3, (void*)mColorFrame_.getData() );

	//cv::imshow( "Color", mImageRGB );

	// RGB ==> BGR
	cv::cvtColor( mImageRGB, mImageRGB, CV_RGB2BGR );

//		cv::waitKey( 1 );
	return mImageRGB;
}
void Camera::MappingDepth2Color(cv::Mat &src, cv::Mat &dst)
{
	double  z;
	uint16_t u, v, d;
	uint16_t u_rgb = 0, v_rgb = 0;
	cv::Mat newdepth(dst.rows, dst.cols, CV_16UC1, cv::Scalar(0));
	for (v = 0; v < src.rows; v++)
	{
		for (u = 0; u < src.cols; u++)
		{
			d = src.at<uint16_t>(v, u);
			z = (double)d;
            u_rgb = (uint16_t)(d2c[0] * (double)u + d2c[1] * (double)v + d2c[2] + d2c[3] / z);
            v_rgb = (uint16_t)(d2c[4] * (double)u + d2c[5] * (double)v + d2c[6] + d2c[7] / z);
			if (u_rgb < 0 || u_rgb >= newdepth.cols || v_rgb < 0 || v_rgb >= newdepth.rows) continue;
			uint16_t *val = (uint16_t *)newdepth.ptr<uchar>(v_rgb)+u_rgb;
			*val = d;
		}
	}

	dst = newdepth;
}
void Camera::checkLines(cv::Mat &CaliDepth, std::vector<cv::Vec4i>& lines, cv::Mat &show)
{
	cv::Mat mImage;
	CaliDepth.convertTo( mImage, CV_8U, 255.0 / 1.0 );
	//cv::imshow("bbbbbbbbbbbbbbbbbbbb", show);
cout<<"before: "<<lines.size()<<endl;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];

		std::vector<cv::Point> vPts;
		GetLinePts(l[0], l[1], l[2], l[3], vPts);

//line(show, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(55, 100, 195), 5, cv::LINE_AA);
        double average(0.0);
        int count(0);
		for(int j=0; j<vPts.size(); j++)
		{
			//circle(show, cv::Point(vPts[j].x, vPts[j].y), 5, cv::Scalar(0, 255, 0), -1);
			double d1 = CaliDepth.at<uint16_t>(vPts[j].y, vPts[j].x)*0.001f;
            if(d1<0.001)
                continue;
            else
            {
                count++;
                average+=d1;
            }
			//cout<<d1<<" ";
		}
		if(count)
        	average /= count;
		//cout<<average<<" "<<abs(average-th)<<endl;

        if(abs(average-th)>0.05)
        {
            lines.erase(lines.begin()+i);
            i--;
        }
    }
    cout<<"after: "<<lines.size()<<endl;
//    waitKey(0);
}
//bool bflag(false);
void Camera::d2cRegister(const cv::Mat &color, cv::Mat &CaliDepth)
{
    cv::Mat     caliDepthHistogram(m_H, m_W, CV_8UC3), registerMat;

    mDepthStream_.readFrame( &mDepthFrame_);
    cv::Mat mImageDepth( mDepthFrame_.getHeight(), mDepthFrame_.getWidth(), CV_16UC1, (void*)mDepthFrame_.getData());

    MappingDepth2Color(mImageDepth, CaliDepth);
    mDraw.GetDepthHistogram(CaliDepth, caliDepthHistogram);
    cv::addWeighted(caliDepthHistogram, 0.5, color, 0.5, 0.5, registerMat);
    cv::imshow("registerImage", registerMat);
}

std::vector<cv::Point3d> Camera::getPositions(const cv::Mat &src, const cv::Mat &CaliDepth, const std::vector<cv::Point>& crossPoints)
{
	CoordinateConverter cc;

    int n = crossPoints.size();
    std::vector<cv::Point3d> vProjecttiveCloud(n, cv::Point3d(0, 0, 0));
    std::vector<cv::Point3d> vRealworldCloud(n, cv::Point3d(0, 0, 0));
    std::vector<cv::Point3d> vRealFinal(n, cv::Point3d(0, 0, 0));
    for( size_t i=0; i<crossPoints.size(); i++)
    {
        int cross_x = crossPoints[i].x;
        int cross_y = crossPoints[i].y;

        circle(src, cv::Point(cross_x, cross_y), 5, cv::Scalar(0, 255, 0), -1);
        //waitKey(0);
        cv::imshow("eeeeeeeeeeeeeee", src);
        
        
        vProjecttiveCloud[i].x = (float)cross_x;
        vProjecttiveCloud[i].y = (float)cross_y;
		//vProjecttiveCloud[i].z = CaliDepth.at<uint16_t>(cross_y, cross_x)*0.001f;
        //cout<<vProjecttiveCloud[i].x<<" "<<vProjecttiveCloud[i].y<<" "<<vProjecttiveCloud[i].z<<endl;
        double z(0.0);
        int num(0);
        for (int v = cross_y-2; v <= cross_y+2; ++v)
        {
            for (int u = cross_x-2; u < cross_x+2; ++u)
            {
                double p1 = CaliDepth.at<uint16_t>(v, u)*0.001f;
                if(abs(p1-0.55)<=0.04){
                    num++;
                    z+=p1;
                }
            }
        }
        if(num)
            z /= num;
        cout<<z<<" "<<(th+0.08)<<endl;
        if(z>0)
        {
            vProjecttiveCloud[i].z = z+0.03;
        }
        else
            vProjecttiveCloud[i].z = th+0.08;
		if(vProjecttiveCloud[i].z>0)
		{
			//cc.convertDepthToWorld(mDepthStream_, vProjecttiveCloud[i].x, vProjecttiveCloud[i].y, vProjecttiveCloud[i].z, &vRealworldCloud[i].x, &vRealworldCloud[i].y, &vRealworldCloud[i].z);
            vRealworldCloud[i].x = (vProjecttiveCloud[i].x * vProjecttiveCloud[i].z - vProjecttiveCloud[i].z * cxr) / fxr;
            vRealworldCloud[i].y = (vProjecttiveCloud[i].y * vProjecttiveCloud[i].z - vProjecttiveCloud[i].z * cyr) / fyr;
            vRealworldCloud[i].z = vProjecttiveCloud[i].z;

            vRealFinal[i].x = Transform[0][0]*vRealworldCloud[i].x + Transform[0][1]*vRealworldCloud[i].y + Transform[0][2]*vRealworldCloud[i].z + Position[0] ;
			vRealFinal[i].y = Transform[1][0]*vRealworldCloud[i].x + Transform[1][1]*vRealworldCloud[i].y + Transform[1][2]*vRealworldCloud[i].z + Position[1] ;
			vRealFinal[i].z = Transform[2][0]*vRealworldCloud[i].x + Transform[2][1]*vRealworldCloud[i].y + Transform[2][2]*vRealworldCloud[i].z + Position[2] ;
			vRealFinal[i].z = -vRealFinal[i].z;//

            cout<<vRealworldCloud[i].x<<" "<<vRealworldCloud[i].y<<" "<<vRealworldCloud[i].z<<endl;
		}

	}
    return vRealworldCloud;
}
bool Camera::GetLinePts(int x1, int y1, int x2, int y2, std::vector<cv::Point>& vPts)
{
	//参数 c 为颜色值
	//增加第一个端点
	cv::Point ptStart(x1, y1);
	cv::Point ptEnd(x2, y2);
	vPts.push_back(ptStart);

	int dx = abs(x2 - x1), dy = abs(y2 - y1), yy = 0;
	if (dx < dy)
	{
		yy = 1;
		swap_int(&x1, &y1);
		swap_int(&x2, &y2);
		swap_int(&dx, &dy);
	}
	int ix = (x2 - x1) > 0 ? 1 : -1, iy = (y2 - y1) > 0 ? 1 : -1, cx = x1, cy = y1, n2dy = dy * 2, n2dydx = (dy - dx) * 2, d = dy * 2 - dx;
	if (yy) { // 如果直线与 x 轴的夹角大于 45 度
		while (cx != x2)
		{
			if (d < 0)
			{
				d += n2dy;
			}
			else
			{
				cy += iy;
				d += n2dydx;
			}
			vPts.push_back(cv::Point(cy, cx));
			cx += ix;
		}
	}
	else
	{
		// 如果直线与 x 轴的夹角小于 45 度
		while (cx != x2)
		{
			if (d < 0)
			{
				d += n2dy;
			}
			else
			{
				cy += iy;
				d += n2dydx;
			}
			vPts.push_back(cv::Point(cx, cy));
			cx += ix;
		}
	}

	//需要包含首尾端点，进行处理
	if (vPts.size() >= 2 && vPts[0] == vPts[1])
	{
		vPts.erase(vPts.begin());
	}

	if (vPts.size() && vPts[vPts.size() - 1] != ptEnd)
	{
		vPts.push_back(ptEnd);
	}

	return true;
}
int Camera::getNextFrame(cv::Mat& color, cv::Mat& ir, cv::Mat& depth)
{
	int changedStreamDummy;
	openni::VideoStream* pStream = &mColorStream_;
	openni::Status rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);


	rc = mColorStream_.readFrame(&mColorFrame_);
	if (rc != openni::STATUS_OK && mColorFrame_.isValid())
	{
		printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
		return -1;
	}
	double resizeFactor = std::min((m_W / (double)mColorFrame_.getWidth()), (m_H / (double)mColorFrame_.getHeight()));
	unsigned int texture_x = (unsigned int)(m_W - (resizeFactor * mColorFrame_.getWidth())) / 2;
	unsigned int texture_y = (unsigned int)(m_H - (resizeFactor * mColorFrame_.getHeight())) / 2;
	for (unsigned int y = 0; y < (m_H - 2 * texture_y); ++y)
	{
		uint8_t* data = (uint8_t*)color.ptr<uchar>(y);
		for (unsigned int x = 0; x < (m_W - 2 * texture_x); ++x)
		{
			OniRGB888Pixel* streamPixel = (OniRGB888Pixel*)((char*)mColorFrame_.getData() + ((int)(y / resizeFactor) * mColorFrame_.getStrideInBytes())) + (int)(x / resizeFactor);
			memcpy(data, streamPixel, 3);
			data = data + 3;
		}
	}

	openni::VideoStream* pDepthStream = &mDepthStream_;
	rc = openni::OpenNI::waitForAnyStream(&pDepthStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);

	rc = mDepthStream_.readFrame(&mDepthFrame_);
	if (rc != openni::STATUS_OK || !mDepthFrame_.isValid())
	{
		printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
		return -1;
	}

	uint16_t *pPixel;
	for (int y = 0; y < m_H; y++)
	{
		pPixel = ((uint16_t*)((char*)mDepthFrame_.getData() + ((int)(y)* mDepthFrame_.getStrideInBytes())));
		uint16_t* data = (uint16_t*)depth.ptr<uchar>(y);
		for (int x = 0; x < m_W; x++)
		{
			*data++ = (*pPixel);
			pPixel++;

			/*if (y == m_H / 2 && x == m_W / 2)
            {
                std::cout << " depth = " << *pPixel << std::endl;
            }*/
		}
	}
	return 0;
}
cv::Mat Camera::ShowDepth()
{
	CoordinateConverter cc;
	//		cv::namedWindow( "Depth",  CV_WINDOW_AUTOSIZE );
	// 读取深度图像数据帧信息流
	mDepthStream_.readFrame( &mDepthFrame_);

	int MaxDepth = mDepthStream_.getMaxPixelValue();

	// 将深度数据转换成OpenCV格式
	cv::Mat mImageDepth( mDepthFrame_.getHeight(), mDepthFrame_.getWidth(), CV_16UC1, (void*)mDepthFrame_.getData());

	cv::Mat mImage;
	// 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
	mImageDepth.convertTo( mImage, CV_8U, 255.0 / 1.0 );
	//cv::imshow( "Depth", mImageDepth );
	return mImageDepth;
}

/*bool Camera::Run(bool bcolor, bool bdepth, bool bMap)
{
	// 读取彩色图像数据帧信息流
	mColorStream_.readFrame( &mColorFrame_);

	cv::Mat cImageBGR;
	// 将彩色数据流转换为OpenCV格式，记得格式是：CV_8UC3（含R\G\B）
	const cv::Mat mImageRGB( mColorFrame_.getHeight(), mColorFrame_.getWidth(),
	    CV_8UC3, (void*)mColorFrame_.getData() );

	// RGB ==> BGR
	cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
	if(bcolor)
		cv::imshow( "User Color", mImageRGB );
		
	nite::Status rc = mUserTracker_.readFrame( &mUserFrame_ );
	if (rc != nite::STATUS_OK)
    {
        cout << "GetUser failed" << endl;
        return false;
    }
    
	// 读取深度图像数据帧信息流
	mDepthStream_.readFrame( &mDepthFrame_);
	
	int MaxDepth = mDepthStream_.getMaxPixelValue();
	
	// 将深度数据转换成OpenCV格式
    cv::Mat mImage( mDepthFrame_.getHeight(), mDepthFrame_.getWidth(), CV_16UC1, (void*)mDepthFrame_.getData());
    
	cv::Mat mImageDepth;
	// 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
    mImage.convertTo( mImageDepth, CV_8U, 255.0 / MaxDepth );
    
	//get data  
    DepthPixel *pDepth = (DepthPixel*)mDepthFrame_.getData();  
    
	CoordinateConverter cc;
	
	//create a projective map
	const int size = 200; //resolution
	cv::Mat Map( size, size, CV_8UC1, cv::Scalar(255) );
			
	ofstream outFile;
	if(bSave_CloudFile_)
	{
		outFile.open(TXT_PATH1);
	}
	//point cloud   
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
	int index=0, depth_count=0;
	for(int row=0; row<mDepthFrame_.getHeight(); row++)  
	{
		for(int column=0; column<mDepthFrame_.getWidth(); column++) 
		{
			cv::Point3f p1,p2;//p1 means point in pixel cordinate;p2 means point in camera cordinate
		   	p1.x = (float)column;
		 	p1.y = (float)row;
		 	p1.z = pDepth[index]*0.001f; // mm -> m
		
			cc.convertDepthToWorld(mDepthStream_, p1.x, p1.y, p1.z, &p2.x, &p2.y, &p2.z);
			if(bSave_Pointcloud_)
			{
				pcl::PointXYZ p;
		        p.x = p2.x;
		        p.y = p2.y;
		        p.z = p2.z;
            
            	if(p.y > -0.7 && p.z < 4 && p.z > 0)
		        	point_cloud->points.push_back( p );
			}
			if(bSave_ColorPointcloud_)
			{
				pcl::PointXYZRGB p;
		        p.x = p2.x;
		        p.y = p2.y;
		        p.z = p2.z;
                p.b = mImageRGB.data[ row*mImageRGB.step+column*mImageRGB.channels()+2 ];
                p.g = mImageRGB.data[ row*mImageRGB.step+column*mImageRGB.channels()+1 ];
                p.r = mImageRGB.data[ row*mImageRGB.step+column*mImageRGB.channels()+0 ];
                if(p.y > -0.7 && p.z < 4 && p.z > 0)
		        	color_cloud->points.push_back( p );
			}
			if(p2.y > -0.7 && p2.z < 4 && p2.z > 0)
            {
            	int n = (int)((p2.x+2)*size/4);
				int m = (int)(p2.z*size/4);
				Map.at<uchar>(m, n) = 0;
				if(bSave_CloudFile_)
				{
					outFile << p2.x << endl;
					outFile << p2.y << endl;
					outFile << p2.z << endl;
					depth_count++;
				}
			}
			
			index++;
		}
	}
	if(bSave_CloudFile_)
	{
		bSave_CloudFile_ = false;
		outFile.close();
		cout<<"Save "<< depth_count << " Points" << endl;
	}
	if(bSave_Image_)
	{
		bSave_Image_ = false;
		imwrite("color.jpg",mImageRGB);
		imwrite("depth.jpg",mImageDepth);
		imwrite("map.jpg",Map);
		cout<<"Save Picture OK!"<< endl;
	}
	if(bSave_Pointcloud_)
	{
		bSave_Pointcloud_ = false;
		point_cloud->is_dense = false;
		cout<<"There are "<<point_cloud->size()<<"points in point_Cloud."<<endl;
		pcl::io::savePCDFileBinary("xcy.pcd", *point_cloud );
	}
	
	if(bSave_ColorPointcloud_)
	{
		bSave_ColorPointcloud_ = false;
		color_cloud->is_dense = false;
		cout<<"There are "<<color_cloud->size()<<"points in color_Cloud."<<endl;
		pcl::io::savePCDFileBinary("xcy_color.pcd", *color_cloud );
	}
	if(bSave_MapFile_)
	{
		bSave_MapFile_ = false;
		outFile.open(TXT_PATH2);
		uchar* p; 
		int depth_count;
		for(int i=0; i<Map.rows; i++)
		{
			p = Map.ptr<uchar>(i);
			for(int j=0; j<Map.cols; j++)
			{
				if(p[j] == 0)
				{
					outFile << j << endl;
					outFile << i << endl;
					depth_count++;
				}
			}
		
		}
		outFile.close();
		cout<<"Save "<< depth_count << " Points" << endl;
	}
	
	if(bMap)			
		cv::imshow( "Map", Map );
		
	if(bdepth)			
		cv::imshow( "User Depth", mImageDepth );
	if(bdepth || bcolor)
	{
		char input = cv::waitKey(1);   
		if( input == 'q' )        
		    return false;  
		if( input == 'p' )  
		{  
		    bSave_Pointcloud_ = true;
		}
		if( input == 'g' )  
		{  
		    bSave_ColorPointcloud_ = true;
		}
		if( input == 'd' )  
		{  
		    bSave_CloudFile_ = true;
		}
		if( input == 'i' )  
		{  
		    bSave_Image_ = true;
		}
		if( input == 'm' )  
		{  
		    bSave_Map_ = true;
		}
		if( input == 'f' )  
		{  
		    bSave_MapFile_ = true;
		}
	}
	return true;
}*/
bool Camera::WaitForNew()
{
	int changedStreamDummy;
	VideoStream* pStream = &mDepthStream_;
    //等待有新的帧出现
    openni::Status rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
    if (rc != openni::STATUS_OK)
    {
        printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
        return false;
    }
	return true;
}




