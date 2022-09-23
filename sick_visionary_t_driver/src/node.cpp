#include "node.h"

void DriverNode::publish_frame(const Driver_3DCS::Data &data);

void DriverNode::thr_publish_frame() {
    g_mtx_data.lock();
    publish_frame(*g_data);
    g_mtx_data.unlock();
}

void DriverNode::on_frame(const boost::shared_ptr<Driver_3DCS::Data> &data) {
    //update data in queue and
    //detach publishing data from network thread

    if(g_publish_all || g_mtx_data.try_lock()) {
        if(g_publish_all)
            g_mtx_data.lock();
        g_data = data;
        g_mtx_data.unlock();

        boost::thread thr(thr_publish_frame());
    }
    else
        ROS_WARN("skipping frame");
}

void DriverNode::publish_frame(const Driver_3DCS::Data &data) {
    bool published_anything = false;

    sensor_msgs::ImagePtr msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = g_frame_id;

    if(g_pub_camera_info.getNumSubscribers()>0) {
        published_anything = true;

        sensor_msgs::CameraInfo ci;
        ci.header = header;

        ci.height = data.getCameraParameters().height;
        ci.width  = data.getCameraParameters().width;

        ci.D.clear();
        ci.D.resize(5,0);
        ci.D[0] = data.getCameraParameters().k1;
        ci.D[1] = data.getCameraParameters().k2;

        for(int i=0; i<9; i++) ci.K[i]=0;
        ci.K[0] = data.getCameraParameters().fx;
        ci.K[4] = data.getCameraParameters().fy;
        ci.K[2] = data.getCameraParameters().cx;
        ci.K[5] = data.getCameraParameters().cy;

        for(int i=0; i<12; i++)
            ci.P[i] = 0;//data.getCameraParameters().cam2worldMatrix[i];
        //TODO:....
        ci.P[0] = data.getCameraParameters().fx;
        ci.P[5] = data.getCameraParameters().fy;
        ci.P[10] = 1;
        ci.P[2] = data.getCameraParameters().cx;
        ci.P[6] = data.getCameraParameters().cy;

        g_pub_camera_info.publish(ci);
    }

    if(g_pub_depth.getNumSubscribers()>0) {
        published_anything = true;

        msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, data.get_distance()).toImageMsg();
        msg->header = header;
        g_pub_depth.publish(msg);
    }
    if(g_pub_confidence.getNumSubscribers()>0) {
        published_anything = true;

        msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, data.get_confidence()).toImageMsg();
        msg->header = header;
        g_pub_confidence.publish(msg);
    }
    if(g_pub_intensity.getNumSubscribers()>0) {
        published_anything = true;

        msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, data.get_intensity()).toImageMsg();
        msg->header = header;
        g_pub_intensity.publish(msg);
    }
    if(g_pub_points.getNumSubscribers()>0) {
        published_anything = true;

        typedef sensor_msgs::PointCloud2 PointCloud;

        // Allocate new point cloud message
        PointCloud::Ptr cloud_msg (new PointCloud);
        cloud_msg->header = header; // Use depth image time stamp
        cloud_msg->height = data.get_distance().rows;
        cloud_msg->width  = data.get_distance().cols;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        cloud_msg->fields.resize (5);
        cloud_msg->fields[0].name = "x"; cloud_msg->fields[1].name = "y"; cloud_msg->fields[2].name = "z";
        cloud_msg->fields[3].name = "confidence"; cloud_msg->fields[4].name = "intensity";
        int offset = 0;
        // All offsets are *4, as all field data types are float32
        for (size_t d = 0; d < cloud_msg->fields.size (); ++d, offset += 4)
        {
            cloud_msg->fields[d].offset = offset;
            cloud_msg->fields[d].datatype = (d<3) ? int(sensor_msgs::PointField::FLOAT32) : int(sensor_msgs::PointField::UINT16);
            cloud_msg->fields[d].count  = 1;
        }
        cloud_msg->point_step = offset;
        cloud_msg->row_step   = cloud_msg->point_step * cloud_msg->width;
        cloud_msg->data.resize (cloud_msg->height * cloud_msg->width * cloud_msg->point_step);

        const float f2rc = data.getCameraParameters().f2rc / 1000.f; // since f2rc is given in [mm], but the pcl message wants [m]

        uint16_t *pDepth, *pConf, *pInt;
        int cp=0;
        for(int i = 0; i < data.get_distance().rows; ++i)
        {
            pDepth = (uint16_t*)(data.get_distance()  .data + (data.get_distance()  .step*i) );
            pConf  = (uint16_t*)(data.get_confidence().data + (data.get_confidence().step*i) );
            pInt   = (uint16_t*)(data.get_intensity() .data + (data.get_intensity() .step*i) );

            for (int j = 0; j < data.get_distance().cols; ++j, ++cp)
            {
                if(pDepth[j]==0 || pDepth[j]==NARE_DISTANCE_VALUE) {
                    const float bad_point = std::numeric_limits<float>::quiet_NaN();
                    memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[0].offset], &bad_point, sizeof (float));
                    memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[1].offset], &bad_point, sizeof (float));
                    memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[2].offset], &bad_point, sizeof (float));
                }
                else {
                    float xp = (data.getCameraParameters().cx - j) / data.getCameraParameters().fx;
                    float yp = (data.getCameraParameters().cy - i) / data.getCameraParameters().fy;

                    // Correction of the lens distortion
                    float r2 = (xp * xp + yp * yp);
                    float r4 = r2 * r2;
                    float k = 1 + data.getCameraParameters().k1 * r2 + data.getCameraParameters().k2 * r4;
                    xp = xp * k;
                    yp = yp * k;

                    // Calculation of the distances
                    float s0 = std::sqrt(xp*xp + yp*yp + 1.f) * 1000.f;

                    // Calculation of the Cartesian coordinates
                    const float ox = xp * pDepth[j] / s0;
                    const float oy = yp * pDepth[j] / s0;
                    const float oz = pDepth[j] / s0 - f2rc;

                    memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[0].offset], &ox, sizeof (float));
                    memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[1].offset], &oy, sizeof (float));
                    memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[2].offset], &oz, sizeof (float));
                }

                memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[3].offset], &pConf[j], sizeof (uint16_t));
                memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[4].offset], &pInt [j], sizeof (uint16_t));
            }
        }

        g_pub_points.publish(cloud_msg);
    }

    if(!published_anything) {
        if(g_control) g_control->stopStream();
    }
}

void DriverNode::_on_new_subscriber() {
    ROS_DEBUG("Got new subscriber");

    if(!g_control) return;

    bool r=g_control->startStream(); // tell the device to start streaming data
    ROS_ASSERT(r);
}

void DriverNode::on_new_subscriber_ros(const ros::SingleSubscriberPublisher& pub) {
    _on_new_subscriber();
}

void DriverNode::on_new_subscriber_it(const image_transport::SingleSubscriberPublisher& pub) {
    _on_new_subscriber();
}