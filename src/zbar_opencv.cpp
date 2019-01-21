#include "zbar_opencv/zbar_opencv.h"

namespace zbar_opencv {
    QRcodeREAD::QRcodeREAD(ros::NodeHandle nh_):
        nh_(nh_),it_(nh_),Judge(false),contour_(0){

        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
        scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
        scanner.set_config(ZBAR_QRCODE,ZBAR_CFG_X_DENSITY,1);
        scanner.set_config(ZBAR_QRCODE,ZBAR_CFG_Y_DENSITY,1);

        pub_QRdata=nh_.advertise<std_msgs::String>("QRdata",1);
        pub_QRpoint=nh_.advertise<geometry_msgs::Point>("QRpoint",1);
        pub_Fourpoint=nh_.advertise<geometry_msgs::Polygon>("Fourpoint",1);
        pub_Centralpoint=nh_.advertise<geometry_msgs::Point>("Centralpoint",1);
        pub_Judge=nh_.advertise<std_msgs::UInt8>("Judge",1);
        pub_Frame=it_.advertise("image_test",1);
        sub_Frame=nh_.subscribe("image_bin/compressed",1,&zbar_opencv::QRcodeREAD::Image_Callback,this);
        pub_Frame_1=it_.advertise("image_test_1",1);
        //cvNamedWindow("debug",CV_WINDOW_AUTOSIZE);
        //pub_QRdata,pub_QRpoint,pub_Fourpoint,pub_Centralpoint,pub_Judge;
    }
    void QRcodeREAD::Image_Callback(const sensor_msgs::CompressedImageConstPtr &Img) {
        //cout<<"test"<<endl;
         cv_bridge::CvImagePtr cv_ptr;
        try
        {
               cv_ptr=cv_bridge::toCvCopy(Img,sensor_msgs::image_encodings::MONO8);
               //cout<<"test"<<endl;
        }
                catch(cv_bridge::Exception& e)
        {
                ROS_ERROR("cv_bridge exception: %s",e.what());
                return;
        }
        //cv::Mat Image;
        //double thre = cv::threshold(cv_ptr->image, pic_0,50, 255, THRESH_OTSU);
        pic_0=SensorToMat(cv_ptr);
        if(!DetectQR(pic_0)) Judge = false;
        else  Judge = true;
        //Judge_topic.data = Judge;
        //pub_Judge.publish(Judge_topic);
        if(!Judge) {
            //cout<<"there is no qrcode"<<endl;
            Judge = false;
        }
        else{
        BuildQRROI(contour_,pic_0,cv_ptr);
        //QRdecoder(pic_1);
        }
        Judge_topic.data = Judge;
        pub_Judge.publish(Judge_topic);//when qrcode not in view or exceed the region, Judge set 0;

    }
    cv::Mat QRcodeREAD::SensorToMat(cv_bridge::CvImagePtr cv_ptr){
        cv::Mat Image;
        double thre = cv::threshold(cv_ptr->image, Image,60, 255, THRESH_OTSU);
        return  Image;

    }
    void QRcodeREAD::BuildQRROI(vector<vector<Point> > contour, cv::Mat pic,cv_bridge::CvImagePtr cv_Ptr){
        vector<Point> pt;
        for(int i=0;i<contour.size();i++){
            for(int j=0;j<contour[i].size();j++) pt.push_back(contour[i][j]);
        }
        cv::RotatedRect rectPoint_ = cv::minAreaRect(pt);
        cv::Rect rect0 = rectPoint_.boundingRect();
        if(0 <= rect0.x && 0 <= rect0.width && rect0.x + rect0.width <= pic.cols
                && 0 <= rect0.y && 0 <= rect0.height && rect0.y + rect0.height <= pic.rows){
            Judge = true;
            pic_1 = Mat(pic,rect0).clone();
            cv_Ptr->image = pic_1.clone();
            pub_Frame.publish(cv_Ptr->toImageMsg());
            QRdecoder(pic_1,rect0);
            //cv_Ptr->image = pic_1.clone();
            //pub_Frame_1.publish(cv_Ptr->toImageMsg());

        }
        else {
            Judge = false;
            cout<<"excueed the region"<<endl;
        }


    }
    bool QRcodeREAD::DetectQR(cv::Mat inImage){
        cv::Mat Img = inImage.clone();
        //cout<<"test1"<<endl;
        contour_ = RecogniseContour(Img);
        //there are 3 squeres involing multiple hierarchy
        if(contour_.size()!=3) return false;
        else return true;

        //cv::RotatedRect Rect_0 = RecogniseRect(Img);
    }
    void QRcodeREAD::QRdecoder(cv::Mat& Img,cv::Rect rect){
        //cv::Mat Img = inImage.clone();
        uint8_t *raw = (uint8_t *)Img.data;
        int x_norm=rect.x,y_norm=rect.y;
        zbar::Image image(Img.cols, Img.rows, "Y800", raw,Img.cols * Img.rows);
        int n = scanner.scan(image);
        if (image.symbol_begin() == image.symbol_end()){
            cout << "查询条码失败，请检查图片！" << endl;
        }
        else {
            cout<<"data: "<<image.symbol_begin()->get_data()<<endl;
            QRdata.data=image.symbol_begin()->get_data().c_str();//please make sure only one qrcode turns up.
            pub_QRdata.publish(QRdata);
            //cout<<image.symbol_begin()->get_location_x(0)<<endl;          
            x0.x=rect.x+image.symbol_begin()->get_location_x(0);
            x0.y=rect.y+image.symbol_begin()->get_location_y(0);
            x1.x=rect.x+image.symbol_begin()->get_location_x(1);
            x1.y=rect.y+image.symbol_begin()->get_location_y(1);
            x2.x=rect.x+image.symbol_begin()->get_location_x(2);
            x2.y=rect.y+image.symbol_begin()->get_location_y(2);
            x3.x=rect.x+image.symbol_begin()->get_location_x(3);
            x3.y=rect.y+image.symbol_begin()->get_location_y(3);
            Centralpoint.x=int((x0.x+x1.x+x2.x+x3.x)/4);
            Centralpoint.y=int((x0.y+x1.y+x2.y+x3.y)/4);
           //line(Img,Point(int(x0.x),int(x0.y)),Point(int(x1.x),int(x1.y)),Scalar(0,0,255),5,CV_AA);
            geometry_msgs::Polygon Fourpoint;
            Fourpoint.points.push_back(x0);
            Fourpoint.points.push_back(x1);
            Fourpoint.points.push_back(x2);
            Fourpoint.points.push_back(x3);
            //cout<<"test_polygon"<<endl;
            pub_Centralpoint.publish(Centralpoint);
            pub_Fourpoint.publish(Fourpoint);
            //cout<<"test1"<<endl;
            //Centralpoint.x = int((Fourpoint.points[0].x+Fourpoint.points[1].x+Fourpoint.points[2].x+Fourpoint.points[3].x)/4);
            //Centralpoint.y = int((Fourpoint.points[0].y+Fourpoint.points[1].y+Fourpoint.points[2].y+Fourpoint.points[3].y)/4);
            //pub_Centralpoint.publish(Centralpoint);
            StringToPoint(QRdata);
        }

    }
    void QRcodeREAD::StringToPoint(std_msgs::String data){
        int qr[2];
        char *pch;
        char datastr[20];
        strcpy(datastr,data.data.c_str());
        pch=strtok(datastr," ,.-");
        int i=0;
        while(pch!=NULL)
        {
          qr[i]=atoi(pch);
          i++;
          pch=strtok(NULL," ,.-");
        }
        QRpoint.x=qr[0];
        QRpoint.y=qr[1];
        pub_QRpoint.publish(QRpoint);
    }
    vector<vector<Point> > QRcodeREAD::RecogniseContour(cv::Mat Img){
        cv::Mat Image = Img.clone();
        vector<vector<Point> > contours_0,contours_1;
        vector<Vec4i> hierarchy;
        cv::findContours(Image,contours_0,hierarchy,CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
        int parentIdx=-1;
        int ic=0;
        for(int i=0;i<contours_0.size();i++)
        {
            if(hierarchy[i][2]!=-1){ //detect child contour
                if(ic==0) parentIdx=i;
                ic++;
                if(ic>=2){
                contours_1.push_back(contours_0[parentIdx]);
                //cv::drawContours(Image,contours_0,parentIdx,Scalar(255,255,255),8,8);
                ic=0;
                parentIdx=-1;
                }
            }
            else if(hierarchy[i][2]==-1){
                ic=0;
                parentIdx=-1;
            }
        }
        return contours_1;
    }
    QRcodeREAD::~QRcodeREAD(){

    }
}
