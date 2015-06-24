#include "load_utilities.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;

using namespace std;

int main(int argc, char** argv)
{
   string waypId        = "WayPoint16"; // the only one for which there is labelled data
   bool visualize       = false; // change this if you want
   //string dataPath    = "/path/to/data/KTH_longterm_dataset_processed/";
   string dataPath      = "/media/akshaya/18F69AB9F69A969A/room-data";
   ofstream fileHandle;

   // Open a Write Out File
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

   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(dataPath, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());

   //for (size_t i=0; i<5;i++)
   for (size_t i=0; i<matchingObservations.size();i++)
   {
        semantic_map_load_utilties::LabelledData<PointType> data = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointType>(matchingObservations[i], false);

        if (data.labelledObjects.size() == 0) continue; // no labelled objects

        // To transform to the map frame of reference:
        tf::StampedTransform world_transform = data.transformToGlobal;
        // In Affine3d format
        Eigen::Affine3d world_transform_mat;
        tf::transformTFToEigen(world_transform, world_transform_mat);
        //***cout << "Transform is:::::" << world_transform_mat.matrix();
        pcl_ros::transformPointCloud(*data.completeCloud, *data.completeCloud,world_transform);

        for (auto object: data.labelledObjects)
        {
            // Transforms entire point clouds
            pcl_ros::transformPointCloud(*(object->m_points), *(object->m_points),world_transform);
        }

        if (visualize)
        {
            pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_handler (data.completeCloud, 100, 0, 0);
            p->addPointCloud (data.completeCloud,cloud_handler,"complete_cloud");
        }
        //cout<<"Now looking at "<<matchingObservations[i]<<"  acquisition time "<<data.sweepTime<<"   labelled objects  "<<data.labelledObjects.size()<<endl<<endl;
        //cout<<"  acquisition time "<<data.sweepTime<<"       "<<"   global transform  "<<data.transformToGlobal <<endl;
        //cout<<"  acquisition time "<<data.sweepTime<<endl<<"Transform is:::::" << world_transform.getOrigin().x()<<endl;
        // Making an output file name
        string pathName   = matchingObservations[i];
        string outFileName;
        string outPathName   = "data-write-out/";
        outFileName.append(outPathName);
        boost::filesystem::create_directories(outPathName);
        bool foundFlag      = false;
        while(true){
            string currSubstr;
            int currLen, currPos;
            if (!foundFlag)
            {
                currPos        = pathName.find("/2");
            }
            else
            {
                currPos        = pathName.find("/");
            }
            if (currPos<0)
                break;
            currSubstr = pathName.substr(0, currPos);
            currLen    = pathName.length();
            pathName   = pathName.substr(currSubstr.length()+1, currLen-currSubstr.length());
            // Concatenate To File Name
            if (foundFlag)
            {
                outFileName.append(currSubstr);
                outFileName.append("_");
            }
            if (!foundFlag)
                foundFlag   = true;
        }
        outFileName   = outFileName.substr(0,outFileName.find_last_of("_"));
        outFileName.append(".txt");

        // File writing out
        fileHandle.open(outFileName);
        fileHandle <<"Time"<<endl<<data.sweepTime<<endl;
        // Handle objects
        for ( size_t j=0; j<data.labelledObjects.size(); j++ )
        {
            DynamicObject::Ptr object = data.labelledObjects[j];
            object->m_centroid[3] = 1;
            //***cout<<"Labelled object "<<j<<"  points "<<object->m_points->points.size()<<"  label  "<< object->m_label<<"  Centroid " <<object->m_centroid<<endl;
            Eigen::Vector4f txCentroid, txBboxHighest, txBboxLowest;
            txBboxHighest[0]   = object->m_bboxHighest.x;
            txBboxHighest[1]   = object->m_bboxHighest.y;
            txBboxHighest[2]   = object->m_bboxHighest.z;
            txBboxHighest[3]   = 1.0;
            txBboxLowest[0]    = object->m_bboxLowest.x;
            txBboxLowest[1]    = object->m_bboxLowest.y;
            txBboxLowest[2]    = object->m_bboxLowest.z;
            txBboxLowest[3]    = 1.0;
            // Transforming the point into new coordinate axes
            txCentroid         = world_transform_mat.cast<float>().matrix() * object->m_centroid;
            txBboxHighest      = world_transform_mat.cast<float>().matrix() * txBboxHighest;
            txBboxLowest       = world_transform_mat.cast<float>().matrix() * txBboxLowest;
            //***cout << "Transformed point is :::"<< txCentroid  << endl;
            //File write
            fileHandle <<"Object_Label"<<endl<<object->m_label<<endl<<"Centroid"<<endl<<object->m_centroid[0]<<','<<object->m_centroid[1]<<','<<object->m_centroid[2]<<','<<object->m_centroid[3]<<endl;
            fileHandle <<"Bbox_Highs"<< endl <<txBboxHighest[0] << ","<< txBboxHighest[1] << ","<< txBboxHighest[2] << ","<< txBboxHighest[3] << ","<<endl;
            fileHandle <<"Bbox_Lows"<< endl<< txBboxLowest[0] << ","<< txBboxLowest[1] << ","<< txBboxLowest[2] << ","<< txBboxLowest[3] << ","<<endl;
            if (visualize)
            {
                stringstream ss;ss<<"object"<<j;
                p->addPointCloud(object->m_points,ss.str());
            }
        }
        fileHandle.close();
        if (visualize)
        {
            p->spin();
            p->removeAllPointClouds();
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



//ifstream currFile(matchingObservations[i]);
//string segment;
//vector<string> seglist;

//while(getline(currFile, segment, '/'))
//{
//   seglist.push_back(segment);
//           cout << segment <<endl;
//}
