#include "lepton_node.hpp"
#include <ros/ros.h>
#include <libuvc/libuvc.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>

//#define PLANAR_BUFFER 1
//#define ACQ_RGB 1
#define ACQ_Y16 1

#define PT1_VID 0x1e4e
#define PT1_PID 0x0100
#define FLIR_VID 0x09cb

LeptonNode::LeptonNode(ros::NodeHandle &private_node)
    : m_ctx(NULL)
    , m_dev(NULL)
    , m_devh(NULL)
    , m_bi ( NULL )
    , m_it ( ros::NodeHandle("~" ) )
    , private_node_(private_node)
{
    m_pub = m_it.advertiseCamera("image_raw", 5);
    m_ids.push_back({ PT1_VID, PT1_PID });
    //m_ids.push_back({ FLIR_VID, 0x0000 }); // any flir camera
    init();
}

LeptonNode::~LeptonNode()
{
    if (m_bi != NULL)
    {
        delete m_bi;
    }

    if ( m_devh != NULL)
    {
        uvc_stop_streaming(m_devh);
        ROS_INFO("Done streaming.");

        /* Release our handle on the device */
        uvc_close(m_devh);
        ROS_INFO("Device closed");
    }

    if (m_dev != NULL)
    {
        /* Release the device descriptor */
        uvc_unref_device(m_dev);
    }

    if (m_ctx != NULL)
    {
        /* Close the UVC context. This closes and cleans up any existing device handles,
         * and it closes the libusb context if one was not provided. */
        uvc_exit(m_ctx);
        ROS_INFO("UVC exited");
    }
    m_isRunning = false;
}

void LeptonNode::init()
{
    uvc_error_t res;

    /* Initialize a UVC service context. Libuvc will set up its own libusb
     * context. Replace NULL with a libusb_context pointer to run libuvc
     * from an existing libusb context. */
    res = uvc_init(&m_ctx, NULL);

    if (res < 0) {
      uvc_perror(res, "uvc_init");
      ROS_ERROR("Could not init uvc.");
      return;
    }

    ROS_INFO("UVC initialized");

    std::string device_sn;
    if (private_node_.getParam("device_sn", device_sn))
    {
        ROS_INFO("Opening by SN");
        /* Locates the UVC device with specified SN */
        for (int i = 0; i < m_ids.size(); ++i) {
            res = uvc_find_device( m_ctx, &m_dev, m_ids[i].vid, m_ids[i].pid, device_sn.c_str());
            if (res >= 0)
                break;
        }
    } else {
        ROS_INFO("Opening first UVC device");
        /* Locates the first attached UVC device, stores in dev */
        for (int i = 0; i < m_ids.size(); ++i) {
            res = uvc_find_device( m_ctx, &m_dev, m_ids[i].vid, m_ids[i].pid, NULL);
            if (res >= 0)
                break;
        }
    }

    if (res < 0) {
        uvc_perror(res, "uvc_find_device"); /* no devices found */
        ROS_ERROR("No devices found.");
        return;
    }

    ROS_INFO("Device found");

    /* Try to open the device: requires exclusive access */
    res = uvc_open(m_dev, &m_devh);

    if (res < 0) {
        uvc_perror(res, "uvc_open"); /* unable to open device */

        ROS_ERROR("Could not open device.");
        m_isRunning = false;
        /* Release the device descriptor */
        uvc_unref_device(m_dev);
        m_dev = NULL;
        return;
    }

    ROS_INFO("Device opened");

    /* Print out a message containing all the information that libuvc
     * knows about the device */
    uvc_print_diag(m_devh, stderr);

    uvc_device_descriptor_t *desc;
    uvc_get_device_descriptor(m_dev, &desc);

    if (desc->idVendor == PT1_VID )
        m_bi = new LeptonInterface( m_ctx, m_dev, m_devh );

    uvc_free_device_descriptor( desc );

    if ( m_bi != NULL )
    {
        setVideoFormat( m_bi->getDefaultFormat(), m_bi->getDefaultHeight(), m_bi->getDefaultWidth() );
        setGainLow(m_bi);
    }
}

void LeptonNode::setGainLow(LeptonInterface* interface)
{
    interface->setGainLow();
    ROS_INFO("Set device gain to low");
}

void LeptonNode::setVideoFormat(const LeptonInterface::VideoFormat &format, const int frameHeight, const int frameWidth )
{
    uvc_error_t res;
    enum uvc_frame_format uvcFormat;

    uvc_stop_streaming(m_devh);

    m_cur_format = format;
    switch( format )
    {
    case LeptonInterface::VideoFormat::YUV420P:
        uvcFormat = UVC_FRAME_FORMAT_I420;
        break;
    case LeptonInterface::VideoFormat::RGB24:
        uvcFormat = UVC_FRAME_FORMAT_RGB;
        break;
    case LeptonInterface::VideoFormat::Y16:
        uvcFormat = UVC_FRAME_FORMAT_Y16;
        break;
    default:
        uvcFormat = UVC_FRAME_FORMAT_UNKNOWN;
        break;
    }

    res = uvc_get_stream_ctrl_format_size(
                m_devh, &m_ctrl, /* result stored in ctrl */
                uvcFormat, frameWidth, frameHeight, 0);

    /* Print out the result */
    uvc_print_stream_ctrl( &m_ctrl, stderr);

    if (res < 0) {
        m_isRunning = false;
        uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
        return;
    }

//    m_uvc_format = format;
//    switch(format.pixelFormat())
//    {
//    case QVideoFrame::Format_YUV420P:
//        m_format = QVideoSurfaceFormat(format.frameSize(), format.pixelFormat());
//        break;
//    case QVideoFrame::Format_RGB24:
//        m_format = QVideoSurfaceFormat(format.frameSize(), QVideoFrame::Format_RGB32);
//        break;
//    case QVideoFrame::Format_Y16:
//        m_format = QVideoSurfaceFormat(format.frameSize(), QVideoFrame::Format_RGB32);
//        break;
//    case QVideoFrame::Format_YV12:
//        m_format = QVideoSurfaceFormat(format.frameSize(), format.pixelFormat());
//        break;
//    default:
//        m_format = QVideoSurfaceFormat(format.frameSize(), QVideoFrame::Format_Invalid);
//        break;
//    }

    /* Start the video stream. The library will call user function cb:
     *   cb(frame, (void*) 12345)
     */
    res = uvc_start_streaming(m_devh, &m_ctrl, LeptonNode::cb, this, 0);

    if (res < 0) {
        uvc_perror(res, "start_streaming"); /* unable to start stream */
        uvc_close(m_devh);
        ROS_ERROR("Device closed");
        return;
    }
    ROS_INFO("Streaming...");
    m_isRunning = true;
}

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void LeptonNode::cb(uvc_frame_t *frame, void *ptr)
{
    ros::Time rec = ros::Time::now();
    LeptonNode *_this = static_cast<LeptonNode*>(ptr);
    static sensor_msgs::CameraInfoPtr cameraInfo;
    static std_msgs::Header header;
    static bool notInitialized = true;
    if ( notInitialized )
    {
        notInitialized = false;
        header.seq = 0;
        header.frame_id = "/thermal";
        cameraInfo = sensor_msgs::CameraInfoPtr ( new sensor_msgs::CameraInfo );
        cameraInfo->height = frame->height;
        cameraInfo->width = frame->width;
        cameraInfo->distortion_model = "plumb_bob";
        cameraInfo->binning_x = 1;
        cameraInfo->binning_y = 1;
        cameraInfo->D.resize(5,0);
        // K
        for ( int i = 0; i < cameraInfo->K.size(); ++i) cameraInfo->K[i] = 0;
        cameraInfo->K[0] = frame->width; // fx
        cameraInfo->K[2] = frame->width/2; // cx
        cameraInfo->K[4] = frame->width; // fy
        cameraInfo->K[5] = frame->height/2; // cy
        cameraInfo->K[8] = 1;
        // R=Id
        for ( int i = 0; i < cameraInfo->R.size(); ++i) cameraInfo->R[i] = 0;
        cameraInfo->R[0] = 1;
        cameraInfo->R[4] = 1;
        cameraInfo->R[8] = 1;
        // P=K
        for ( int i = 0; i < cameraInfo->P.size(); ++i) cameraInfo->P[i] = 0;
        cameraInfo->P[0] = frame->width; // fx
        cameraInfo->P[2] = frame->width/2; // cx
        cameraInfo->P[5] = frame->width; // fy
        cameraInfo->P[7] = frame->height/2; // cy
        cameraInfo->P[10] = 1;
    }
    ++header.seq;
    header.stamp = rec;
    cameraInfo->header = header;

    cv_bridge::CvImage msg;
    if ( _this->m_cur_format == LeptonInterface::VideoFormat::Y16 )
    {
        cv::Mat cv_img ( frame->height, frame->width, CV_16UC1, frame->data, frame->step );
        msg = cv_bridge::CvImage( header, sensor_msgs::image_encodings::MONO16, cv_img);
        _this->m_pub.publish( msg.toImageMsg(), cameraInfo );
    }
    if ( _this->m_cur_format == LeptonInterface::VideoFormat::YUV420P )
    {
        uvc_frame_t *outFrame = uvc_allocate_frame( frame->width * frame->height * 3 );
        uvc_error_t res = uvc_i4202bgr( frame, outFrame );
        if (res >= 0)
        {
            cv::Mat cv_img ( outFrame->height, outFrame->width, CV_8UC3, outFrame->data, outFrame->step );
            msg = cv_bridge::CvImage( header, sensor_msgs::image_encodings::RGB8, cv_img );
        }
        _this->m_pub.publish( msg.toImageMsg(), cameraInfo );
        uvc_free_frame ( outFrame );
    }
}

void LeptonNode::pauseStream()
{
    uvc_stop_streaming(m_devh);
    ROS_INFO("Stream paused.");
}

void LeptonNode::resumeStream()
{
    uvc_error_t res = uvc_start_streaming(m_devh, &m_ctrl, LeptonNode::cb, this, 0);
    if (res < 0)
    {
        uvc_perror(res, "start_streaming"); /* unable to start stream */
        uvc_close(m_devh);
        ROS_ERROR("Device closed");
        return;
    }
}
