#ifndef _ZBAR_OPENCV_NODELET_H
#define _ZBAR_OPENCV_NODELET_H
#include <zbar_opencv/zbar_opencv.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace zbar_opencv {
class QRcodeREADNodelet:public nodelet::Nodelet{
public:
    QRcodeREADNodelet();
    virtual void onInit();
    ~QRcodeREADNodelet();
private:
    QRcodeREAD *qrcodereader;
};

}
#endif  //_ZBAR_OPENCV_NODELET_H
