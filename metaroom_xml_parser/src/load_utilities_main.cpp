#include "load_utilities.h"

typedef pcl::PointXYZRGB PointType;

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

    std::string pathToSweepXml = "/media/HD-PCFU3/KTH_longterm_dataset/20140822/patrol_run_4/room_1/room.xml";
    std::string folderPath = "/media/HD-PCFU3/test_depth_dataset";

    boost::shared_ptr<pcl::PointCloud<PointType>> mergedCloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointType>(pathToSweepXml, true);

    {
        std::stringstream ss;
        ss << "test_merged_cloud";
        ss <<".pcd";
        pcl::io::savePCDFileBinary(ss.str(), *mergedCloud);
    }

    auto mergedClouds = semantic_map_load_utilties::loadMergedCloudFromMultipleSweeps<PointType>(folderPath,true);

    for (size_t i=0; i<mergedClouds.size();i++)
    {
        {
            std::stringstream ss;
            ss << "test_merged_cloud";
            ss<<i;
            ss <<".pcd";
            pcl::io::savePCDFileBinary(ss.str(), *mergedClouds[i]);
        }
    }

    mergedClouds = semantic_map_load_utilties::loadMergedCloudForTopologicalWaypoint<PointType>(folderPath,"WayPoint5",true);

    for (size_t i=0; i<mergedClouds.size();i++)
    {
        {
            std::stringstream ss;
            ss << "waypoint_cloud";
            ss<<i;
            ss <<".pcd";
            pcl::io::savePCDFileBinary(ss.str(), *mergedClouds[i]);
        }
    }

//    auto intClouds = semantic_map_load_utilties::loadIntermediateCloudsFromSingleSweep<PointType>(pathToSweepXml,true);

//    for (size_t i=0; i<intClouds.size();i++)
//    {
//        {
//            std::stringstream ss;
//            ss << "int_cloud";
//            ss<<i;
//            ss <<".pcd";
//            pcl::io::savePCDFileBinary(ss.str(), *intClouds[i]);
//        }
//    }

//    auto intCloudsComplete = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataForTopologicalWaypoint<PointType>(folderPath, "WayPoint5",true);

//    for (size_t i=0; i<intCloudsComplete.size();i++)
//    {
//        for (size_t j=0; j<intCloudsComplete[i].vIntermediateRoomClouds.size();j++)
//        {
//            {
//                std::stringstream ss;
//                ss << "int_cloud";
//                ss<<i;
//                ss<<"_";
//                ss<<j;
//                ss <<".pcd";
//                pcl::io::savePCDFileBinary(ss.str(), *intCloudsComplete[i].vIntermediateRoomClouds[j]);
//            }

//        }

//        for (cv::Mat image : intCloudsComplete[i].vIntermediateRGBImages)
//        {
//            imshow( "Display window", image );                   // Show our image inside it.
//            waitKey(0);                                          // Wait for a keystroke in the window
//        }

//        for (cv::Mat image : intCloudsComplete[i].vIntermediateDepthImages)
//        {
//            imshow( "Display window", image );                   // Show our image inside it.
//            waitKey(0);                                          // Wait for a keystroke in the window
//        }
//    }

//    std::string intPosXml = "/media/HD-PCFU3/test_dataset/20121205/patrol_run_34/room_0/room.xml";
//    auto intPosImages = semantic_map_load_utilties::loadIntermediatePositionImagesFromSingleSweep<PointType>(intPosXml,true);

//    for (SimpleXMLParser<PointType>::IntermediatePositionImages int_image : intPosImages)
//    {
//        cout<<"Depth transform "<<endl;
//        cout<<"Stamp sec "<<int_image.intermediateDepthTransform.stamp_.sec<<endl;
//        cout<<"Stamp nsec "<<int_image.intermediateDepthTransform.stamp_.nsec<<endl;
//        cout<<"Frame "<<int_image.intermediateDepthTransform.frame_id_<<endl;
//        cout<<"Child frame "<<int_image.intermediateDepthTransform.child_frame_id_<<endl;
//        cout<<"Rotation "<<int_image.intermediateDepthTransform.getRotation().getW()<<" "<<int_image.intermediateDepthTransform.getRotation().getX()<<" "<<int_image.intermediateDepthTransform.getRotation().getY()<<" "<<int_image.intermediateDepthTransform.getRotation().getZ()<<" "<<endl;
//        cout<<"Translation  "<<int_image.intermediateDepthTransform.getOrigin().getX()<<" "<<int_image.intermediateDepthTransform.getOrigin().getY()<<" "<<int_image.intermediateDepthTransform.getOrigin().getZ()<<" "<<endl;

//        cout<<endl<<"RGB transform "<<endl;
//        cout<<"Stamp sec "<<int_image.intermediateRGBTransform.stamp_.sec<<endl;
//        cout<<"Stamp nsec "<<int_image.intermediateRGBTransform.stamp_.nsec<<endl;
//        cout<<"Frame "<<int_image.intermediateRGBTransform.frame_id_<<endl;
//        cout<<"Child frame "<<int_image.intermediateRGBTransform.child_frame_id_<<endl;
//        cout<<"Rotation "<<int_image.intermediateRGBTransform.getRotation().getW()<<" "<<int_image.intermediateRGBTransform.getRotation().getX()<<" "<<int_image.intermediateRGBTransform.getRotation().getY()<<" "<<int_image.intermediateRGBTransform.getRotation().getZ()<<" "<<endl;
//        cout<<"Translation  "<<int_image.intermediateRGBTransform.getOrigin().getX()<<" "<<int_image.intermediateRGBTransform.getOrigin().getY()<<" "<<int_image.intermediateRGBTransform.getOrigin().getZ()<<" "<<endl;

//        cout<<endl<<"RGB camera params "<<endl;
//        cout<<"frame "<<int_image.intermediateRGBCamParams.tfFrame()<<endl;
//        cout<<"fx" <<int_image.intermediateRGBCamParams.fx()<<endl;
//        cout<<"fy" <<int_image.intermediateRGBCamParams.fy()<<endl;
//        cout<<"cx" <<int_image.intermediateRGBCamParams.cx()<<endl;
//        cout<<"cy" <<int_image.intermediateRGBCamParams.cy()<<endl;

//        cout<<endl<<"Depth camera params "<<endl;
//        cout<<"frame "<<int_image.intermediateDepthCamParams.tfFrame()<<endl;
//        cout<<"fx" <<int_image.intermediateDepthCamParams.fx()<<endl;
//        cout<<"fy" <<int_image.intermediateDepthCamParams.fy()<<endl;
//        cout<<"cx" <<int_image.intermediateDepthCamParams.cx()<<endl;
//        cout<<"cy" <<int_image.intermediateDepthCamParams.cy()<<endl;

////        for (cv::Mat image : int_image.vIntermediateRGBImages)
////        {
////            imshow( "Display window", image );                   // Show our image inside it.
////            waitKey(0);                                          // Wait for a keystroke in the window
////        }

////        for (cv::Mat image : int_image.vIntermediateDepthImages)
////        {
////            imshow( "Display window", image );                   // Show our image inside it.
////            waitKey(0);                                          // Wait for a keystroke in the window
////        }
//    }

    auto dynamicClusters = semantic_map_load_utilties::loadDynamicClustersFromMultipleSweeps<PointType>("/home/rares/Data/test_metaroom_data", true);

    for (size_t i=0; i<dynamicClusters.size(); i++)
    {
        for (size_t j=0; j<dynamicClusters[i].size(); j++)
        {
            {
                std::stringstream ss;
                ss << "clusters";
                ss<<i;
                ss<<"_";
                ss<<j;
                ss <<".pcd";
                pcl::io::savePCDFileBinary(ss.str(), *dynamicClusters[i][j]);
            }
        }
    }
}
