#include <zbar_opencv/zbar_opencv_nodelet.h>

namespace zbar_opencv {
        QRcodeREADNodelet::QRcodeREADNodelet(){}
        void QRcodeREADNodelet::onInit(){
            ros::NodeHandle nh = getNodeHandle();
            qrcodereader = new QRcodeREAD(nh);
        }
        QRcodeREADNodelet::~QRcodeREADNodelet(){
            if(qrcodereader) delete qrcodereader;
        }
}
PLUGINLIB_EXPORT_CLASS(zbar_opencv::QRcodeREADNodelet, nodelet::Nodelet)
