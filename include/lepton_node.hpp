#ifndef LEPTON_NODE_HPP_
#define LEPTON_NODE_HPP_

#include <libuvc/libuvc.h>
#include <unistd.h>
#include "lepton_interface.hpp"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vector>

class LeptonNode
{
public:
    struct UsbId {
        int vid;
        int pid;
    };
    
    LeptonNode(ros::NodeHandle& private_node);
    ~LeptonNode( );
    void init();

    void pauseStream();
    void resumeStream();
    bool isRunning() { return m_isRunning; }
    void setGainLow(LeptonInterface* interface);
    void setVideoFormat(const LeptonInterface::VideoFormat &format, const int frameHeight, const int frameWidth );

protected:
    bool m_isRunning = false;
    uvc_context_t *m_ctx;
    uvc_device_t *m_dev;
    uvc_device_handle_t * m_devh;
    uvc_stream_ctrl_t m_ctrl;
    LeptonInterface * m_bi;
    std::vector<UsbId> m_ids;
    LeptonInterface::VideoFormat m_cur_format;
    std::string frame_id;
    std::string device_sn;
private:
    image_transport::ImageTransport m_it;
    image_transport::CameraPublisher m_pub;

    ros::NodeHandle private_node_;

    static void cb(uvc_frame_t *frame, void *ptr);
};

#endif //LEPTON_NODE_HPP_
