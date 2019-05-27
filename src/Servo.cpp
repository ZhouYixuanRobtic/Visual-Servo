//
// Created by xzy on 19-4-1.
//
/******************************************************
获取彩色和深度图像以及点云的类
******************************************************/

#include "Servo.h"




//默认构造函数
Servo::Servo()
{
    //设置内参

    this->fx=615.3072;
    this->fy=616.1456;
    this->u0=333.4404;
    this->v0=236.2650;


}
Servo::~Servo()
{

}


std::vector<TagDetectInfo> Servo::GetTargetPoseMatrix(Mat UserImage, double TagSize)
{
    char TagFamilyName[20]="tag36h11"; //Tag family to use
    int ThreadsNumber=2;                //Use this many CPU threads
    double Decimate=2.0;                //Decimate input image by this factor
    double Blur=0.0;                    //Apply low-pass blur to input
    bool Refine_edges=true;             //Spend more time trying to align edges of tags
    bool Quiet=false;                   //Reduce output
    bool Debug=false;                    //Enable debugging output (slow)

    Eigen::Affine3d Trans_C2T;
    std::vector<TagDetectInfo> TagsDetected;
    TagDetectInfo TagDetected{};
    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;

    if (!strcmp(TagFamilyName, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(TagFamilyName, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(TagFamilyName, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(TagFamilyName, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(TagFamilyName, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(TagFamilyName, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(TagFamilyName, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(TagFamilyName, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }


    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = Decimate;
    td->quad_sigma = Blur;
    td->nthreads = ThreadsNumber;
    td->debug = Debug;
    td->refine_edges = Refine_edges;

    Mat frame, gray;

    frame=UserImage;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };

    zarray_t *detections = apriltag_detector_detect(td, &im);
    //cout << zarray_size(detections) << " tags detected" << endl;

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        line(frame, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[1][0], det->p[1][1]),
             Scalar(0, 0xff, 0), 2);
        line(frame, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0, 0, 0xff), 2);
        line(frame, Point(det->p[1][0], det->p[1][1]),
             Point(det->p[2][0], det->p[2][1]),
             Scalar(0xff, 0, 0), 2);
        line(frame, Point(det->p[2][0], det->p[2][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0xff, 0, 0), 2);

        std::stringstream ss;
        ss << det->id;
        String text = ss.str();
        int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        Size textsize = getTextSize(text, fontface, fontscale, 2,
                                    &baseline);
        putText(frame, text, Point(det->c[0]-textsize.width/2,
                                   det->c[1]+textsize.height/2),
                fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        //pose_estimation
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = TagSize;
        info.fx = fx;
        info.fy = fy;
        info.cx = u0;
        info.cy = v0;
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);
        Trans_C2T.matrix() << pose.R->data[0],  pose.R->data[1],  pose.R->data[2],  pose.t->data[0],
                pose.R->data[3],  pose.R->data[4],  pose.R->data[5],  pose.t->data[1],
                pose.R->data[6],  pose.R->data[7],  pose.R->data[8],  pose.t->data[2],
                0.0,  0.0,  0.0,  1.0;
        TagDetected.Trans_C2T=Trans_C2T;
        TagDetected.id=det->id;
        TagDetected.Center=Point2d(det->c[0],det->c[1]);
        TagDetected.PixelCoef=0.0;
        TagDetected.PixelCoef+=TagSize/norm(Point(det->p[0][0]-det->p[1][0],det->p[0][1]-det->p[1][1]));
        TagDetected.PixelCoef+=TagSize/norm(Point(det->p[0][0]-det->p[3][0],det->p[0][1]-det->p[3][1]));
        TagDetected.PixelCoef+=TagSize/norm(Point(det->p[1][0]-det->p[2][0],det->p[1][1]-det->p[2][1]));
        TagDetected.PixelCoef+=TagSize/norm(Point(det->p[2][0]-det->p[3][0],det->p[2][1]-det->p[3][1]));
        TagDetected.PixelCoef+=sqrt(2)*TagSize/norm(Point(det->p[0][0]-det->p[2][0],det->p[0][1]-det->p[2][1]));
        TagDetected.PixelCoef+=sqrt(2)*TagSize/norm(Point(det->p[1][0]-det->p[3][0],det->p[1][1]-det->p[3][1]));
        TagDetected.PixelCoef=TagDetected.PixelCoef/6;
        TagsDetected.push_back(TagDetected);
    }
    zarray_destroy(detections);

    imshow("Tag Detections", frame);
    waitKey(3);

    apriltag_detector_destroy(td);

    if (!strcmp(TagFamilyName, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(TagFamilyName, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(TagFamilyName, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(TagFamilyName, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(TagFamilyName, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(TagFamilyName, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(TagFamilyName, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(TagFamilyName, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }
    return TagsDetected;
}
Destination_t Servo::GetCameraDestination(Eigen::Affine3d Trans_C2T,Eigen::Affine3d Trans_E2C, Eigen::Affine3d ExpectTrans_C2T)
{
    Eigen::Affine3d EndMotion;
    EndMotion=Trans_E2C*Trans_C2T*ExpectTrans_C2T.inverse()*Trans_E2C.inverse();
    //动态插值

    //插值分割比例
    double lambda=0.8;
    //需要插值的最小值
    double Interpolate_tolerance=0.05;
    Eigen::Matrix3d R=EndMotion.rotation();
    Eigen::Vector3d t=EndMotion.translation();
    Sophus::SE3 DestinationSE3(R,t);
    //旋转插值
    Eigen::Quaterniond Td=Sophus::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0)).unit_quaternion().slerp(lambda,
            DestinationSE3.unit_quaternion());

    //平移插值
    for(int i=0;i<3;++i)
    {
     if(abs(t(i))>Interpolate_tolerance)
         t(i)=lambda*t(i);
    }
    Sophus::SE3 EndMotionDelta(Td,t);
    Eigen::Vector3d EndTranslation=EndMotionDelta.translation();
    Eigen::Vector3d EndRotation=EndMotionDelta.rotation_matrix().eulerAngles(0,1,2);//rpy
    //增量操作
    Destination_t EndDestinationDelta{};
    EndDestinationDelta.EE_Motion.matrix()=EndMotionDelta.matrix();
    //cout<<EndDestinationDelta.EE_Motion<<endl;
    EndDestinationDelta.error=sqrt(EndMotion(0,3)*EndMotion(0,3)+EndMotion(1,3)*EndMotion(1,3)+
                                              EndMotion(2,3)*EndMotion(2,3));
    return EndDestinationDelta;
}
void Servo::SetCameraParameter(double fx, double fy,  double u0,  double v0)
{
    this->fx=fx;
    this->fy=fy;
    this->u0=u0;
    this->v0=v0;
}




