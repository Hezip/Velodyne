/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);

    flag_=1;
    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
    

    output_he_=
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points_he", 10,false);

      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      VelodyneConfigConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::VelodyneConfigConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  void Convert::callback(velodyne_pointcloud::VelodyneConfigConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    using namespace velodyne_rawdata;
    
    //if (output_.getNumSubscribers() == 0)         // no one listening?
    //  return;                                     // avoid much work
    //if (output_he_.getNumSubscribers() == 0)
    //  return;

    // allocate a point cloud with same time and frame ID as raw data
    VPointCloud::Ptr
      outMsg(new VPointCloud()), outMsg_he(new VPointCloud()), outMsg_diff(new VPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    outMsg_he->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg_he->header.frame_id = scanMsg->header.frame_id;
    outMsg_he->height = 1;    

    
    // If the message contains no packets, return an empty point cloud.
    if (scanMsg->packets.size() < 1) {
      output_.publish(outMsg);
      output_he_.publish(outMsg_he);
      return;
    }

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i){
      data_->unpack(scanMsg->packets[i], *outMsg);
      data_->unpack(scanMsg->packets[i], *outMsg_he);
    }

    ROS_ERROR("runtime");


//      *outMsg_he+=*outMsg;
//    for (size_t i=0; i < outMsg->points.size(); i+=2) {
//        (*outMsg_he).points[i].x= (*outMsg).points[i].x;
//      outMsg_he->points[i].x=outMsg->points[i].x;
//      outMsg_he->points[i].y=outMsg->points[i].y;
//      outMsg_he->points[i].z=outMsg->points[i].z;
//      outMsg_he->points[i].ring=outMsg->points[i].ring;
//      outMsg_he->points[i].intensity=outMsg->points[i].intensity;     
//      ++ outMsg_he->width;
//    }



    // Only for VLP-16 in dual-return mode: process two messages at once 
    // to capture a full scanner revolution.
    SensorModel model = data_->getSensorModel();
    ReturnMode  mode  = data_->getReturnMode(scanMsg->packets[0]);
    if (0) {
      
      if (bufferedMsg_) {
        // Insert the buffered message into the current one.
        outMsg->insert(outMsg->begin(), bufferedMsg_->begin(), bufferedMsg_->end());
        outMsg->header = bufferedMsg_->header;
        bufferedMsg_.reset();

        outMsg_he->insert(outMsg_he->begin(), bufferedMsg_he_->begin(), bufferedMsg_he_->end());
        outMsg_he->header = bufferedMsg_he_->header;
        bufferedMsg_he_.reset();
      } else {
        bufferedMsg_ = outMsg;
        bufferedMsg_he_ = outMsg_he;
        return;
      }
    }
      

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height << " x " << outMsg->width << "points "
                     << "from " << model << " "
                     << "in " << mode << " return mode, "
                     << "time: " << outMsg->header.stamp);

/*
      VPointCloud::iterator itrStrong = outMsg_he->begin();
      for(VPointCloud::iterator itr = outMsg->begin(); itr != outMsg->end() && itrStrong != outMsg_he->end();){
        if( fabs(itr->x - itrStrong->x) > 0.1 ){
          // diff point
          outMsg_diff->push_back(*itrStrong);
        }
         itr++;
        itrStrong++;
      }
      outMsg_diff->header = outMsg->header;
*/
      output_.publish(outMsg);
      output_he_.publish(outMsg_he);


  }

} // namespace velodyne_pointcloud
