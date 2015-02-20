#include <vector>
#include <QDir>

#include "semantic_map/room_xml_parser.h"
#include "load_utilities.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;

int main(int argc, char** argv)
{
   string path_from;
   string path_to;

   if (argc == 3)
   {
      path_from = argv[1];
      path_from += "/"; // just in case
      path_to = argv[2];
      path_to += "/"; // just in case
   } else {
      cout<<"Please provide the path from where to load data and the path where to save the data"<<endl;
      return -1;
   }


   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmls<PointType>(path_from);
   ROS_INFO_STREAM("Observation matches "<<matchingObservations.size());

   for (size_t i=0; i<matchingObservations.size();i++)
   {
      ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);

      SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(matchingObservations[i],false); // load sweep
      CloudPtr completeRoomCloud = aRoom.getCompleteRoomCloud();

      // TODO - segment sweep -> do some stuff ----------------------------------
      CloudPtr labelledCloud(new Cloud());


      // -----------------------------------------------------------------------
      // Once the labelling is done, set the result and save
      aRoom.setCompleteRoomCloud(labelledCloud);
      SemanticRoomXMLParser<PointType> saveParser(path_to);
      saveParser.saveRoomAsXML(aRoom);
   }
}
