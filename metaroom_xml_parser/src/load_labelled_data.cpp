#include "load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;

using namespace std;

int main(int argc, char** argv)
{
   string waypId = "WayPoint16"; // the only one for which there is labelled data
   bool visualize = true; // change this if you want
   //string dataPath = "/path/to/data/KTH_longterm_dataset_processed/";
   string dataPath = "/media/akshaya/18F69AB9F69A969A/room-data";

   if (argc == 3)
   {
      dataPath = argv[1];
      waypId = argv[2];
   } else {
      cout<<"Using default arguments"<<endl;
   }

   pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer (argc, argv, "Labelled data");
   p->addCoordinateSystem();
   p->setBackgroundColor(1,1,1);

//   pcl::visualization::PCLVisualizer *q = new pcl::visualization::PCLVisualizer (argc, argv, "Labelled data");
//   q->addCoordinateSystem();
//   q->setBackgroundColor(1,1,1);

   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(dataPath, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());

   //for (size_t i=0; i<matchingObservations.size();i++) HACK HACK
   for (size_t i=0; i<5;i++)
   {
        semantic_map_load_utilties::LabelledData<PointType> data = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointType>(matchingObservations[i], false);

        if (data.labelledObjects.size() == 0) continue; // no labelled objects

        // To transform to the map frame of reference:
        static tf::StampedTransform world_transform = data.transformToGlobal;
// Akshaya ---
        Eigen::Affine3d world_transform_mat;
        tf::transformTFToEigen(world_transform, world_transform_mat);
        cout << "Transform is:::::" << world_transform_mat.matrix();
//        cout << "Transform is:::::" << world_transform_mat;
// --- Akshaya
        pcl_ros::transformPointCloud(*data.completeCloud, *data.completeCloud,world_transform);

        for (auto object: data.labelledObjects)
        {
            // Transforms entire point clouds
            pcl_ros::transformPointCloud(*(object->m_points), *(object->m_points),world_transform);
            // Transform single points
        }


        if (visualize)
        {
            pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_handler (data.completeCloud, 100, 0, 0);
            //pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_handler (data.completeCloud, 100, 0, 0);
            p->addPointCloud (data.completeCloud,cloud_handler,"complete_cloud");
            //q->addPointCloud (data.completeCloud,cloud_handler,"complete_cloud");
        }

        cout<<"Now looking at "<<matchingObservations[i]<<"  acquisition time "<<data.sweepTime<<"   labelled objects  "<<data.labelledObjects.size()<<endl<<endl;
        for ( size_t j=0; j<data.labelledObjects.size(); j++ )
        {
            DynamicObject::Ptr object = data.labelledObjects[j];
            cout<<"Labelled object "<<j<<"  points "<<object->m_points->points.size()<<"  label  "<< object->m_label<<"  Bounding box " <<object->m_bboxHighest.getVector3fMap()[0] <<endl;
            if (visualize)
            {
                stringstream ss;ss<<"object"<<j;
                p->addPointCloud(object->m_points,ss.str());
//                q->addPointCloud(object->m_points,ss.str());
            }
        }

        if (visualize)
        {
            p->spin();
//            q->spin();
            p->removeAllPointClouds();
//            q->removeAllPointClouds();
        }

   }
}




//geometry_msgs::PointStamped pt;
//geometry_msgs::PointStamped pt_transformed;
//pt.header = myCloud->header;
//pt.point.x = myCloud->points[1].x;
//pt.point.y = myCloud->points[1].y;
//pt.point.z = myCloud->points[1].z;

//tf::TransformListener listener;
//listener.transformPoint("target_frame", pt, pt_transformed);

// open load_utilities.h transforms.h semantic_map/dynamic_object.cpp
