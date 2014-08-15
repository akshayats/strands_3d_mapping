#include "semantic_map_node.h"

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "Semantic_map_node");
    ros::NodeHandle n;

    SemanticRoomXMLParser<pcl::PointXYZRGB> parser;
    SemanticRoom<pcl::PointXYZRGB> aRoom = SemanticRoomXMLParser<pcl::PointXYZRGB>::loadRoomFromXML("/home/rares/.semanticMap/patrol_run_1/room_0/room.xml",true);

    std::vector<SemanticRoom<pcl::PointXYZRGB>::CloudPtr> clouds = aRoom.getIntermediateClouds();

    std::vector<image_geometry::PinholeCameraModel> camInfos = aRoom.getIntermediateCloudCameraParameters();


    for (size_t i=0; i<clouds.size(); ++i)
    {
        std::cout<<"Cloud size "<<clouds[i]->points.size()<<std::endl;
        std::cout<<camInfos[i].fx()<<"  "<<camInfos[i].fy()<<"  "<<camInfos[i].cx()<<"  "<<camInfos[i].cy()<<std::endl;
    }


    ros::NodeHandle aRosNode("~");

    SemanticMapNode<pcl::PointXYZRGB> aSemanticMapNode(aRosNode);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
