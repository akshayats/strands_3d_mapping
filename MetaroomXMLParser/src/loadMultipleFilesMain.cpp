#include "simpleXMLparser.h"

#include <semanticMapSummaryParser.h>

typedef pcl::PointXYZRGB PointType;

typedef typename SemanticMapSummaryParser<PointType>::EntityStruct Entities;

using namespace std;

int main(int argc, char** argv)
{
    SemanticMapSummaryParser<PointType> summary_parser("/home/rares/data/test_parser/index.xml");
    summary_parser.createSummaryXML("/home/rares/data/test_parser/");

    SimpleXMLParser<PointType> simple_parser;
    SimpleXMLParser<PointType>::RoomData roomData;

    std::vector<Entities> allSweeps = summary_parser.getRooms();

    for (size_t i=0; i<allSweeps.size(); i++)
    {
        cout<<"Parsing "<<allSweeps[i].roomXmlFile<<endl;

        roomData = simple_parser.loadRoomFromXML(allSweeps[i].roomXmlFile);
        cout<<"Complete cloud size "<<roomData.completeRoomCloud->points.size()<<endl;
        for (size_t i=0; i<roomData.vIntermediateRoomClouds.size(); i++)
        {
            cout<<"Intermediate cloud size "<<roomData.vIntermediateRoomClouds[i]->points.size()<<endl;
            cout<<"Fx: "<<roomData.vIntermediateRoomCloudCamParams[i].fx()<<" Fy: "<<roomData.vIntermediateRoomCloudCamParams[i].fy()<<endl;
        }
    }
}
