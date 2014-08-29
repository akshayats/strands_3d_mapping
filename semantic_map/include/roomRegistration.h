#ifndef __ROOM_REGISTRATION__H
#define __ROOM_REGISTRATION__H

#include "room.h"

#include <scan.h>
#include <fine_mapping.h>
#include <stitched_map.h>

#include <pcl_ros/transforms.h>

class RoomRegistration {
public:

    typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::iterator CloudIterator;

    RoomRegistration(){}
    ~RoomRegistration(){}

    template <class PointType>
    bool registerRoom(SemanticRoom<PointType>& aRoom)
    {

        // convert to fine mapping datatypes
        std::vector<image_geometry::PinholeCameraModel> cameraParameters = aRoom.getIntermediateCloudCameraParameters();
        std::vector<tf::StampedTransform> cloudTransforms = aRoom.getIntermediateCloudTransforms();
        std::vector<CloudPtr> clouds= aRoom.getIntermediateClouds();

        if ((cameraParameters.size() != cloudTransforms.size()) || (cameraParameters.size() != clouds.size()) || (clouds.size() != cloudTransforms.size()))
        {
            ROS_ERROR("Cannot register individual points clouds for room "<<aRoom.getRoomLogName()<<"  as the sizes of the camera parameters, transforms and individual point cloud vectors are not equal.");
            return false;
        }

        std::vector<scan* > allScans;

        for (size_t i=0; i<clouds.size();i++)
        {
            Eigen::Matrix4f eigen_transform;
            Eigen::Matrix3f eigen_rot;
            Eigen::Vector3f eigen_translation;
            Eigen::Matrix3f eigen_K;
            pcl_ros::transformAsMatrix (cloudTransforms[i], eigen_transform);
            eigen_rot = eigen_transform.topLeftCorner<3, 3>();
            eigen_translation = eigen_transform.block<3, 1>(0, 3);
            cv2eigen(cameraParameters[i].intrinsicMatrix(),eigen_K);
            allScans.push_back(new scan(*clouds[i], eigen_translation,eigen_rot,eigen_K));
        }

        // Fine mapping
        fine_mapping f(allScans);
        f.build_graph();
        f.optimize_graph();
        stitched_map map(allScans);
        map.visualize(true);
    }

private:
};


#endif
