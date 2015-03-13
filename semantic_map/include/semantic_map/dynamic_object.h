#ifndef __DYNAMIC_OBJECT__H
#define __DYNAMIC_OBJECT__H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/correspondence.h>
#include <pcl/features/vfh.h>

class ObjectMatches;


class DynamicObject  {
public:

    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    typedef pcl::search::KdTree<PointType> Tree;
    typedef Tree::Ptr TreePtr;

    typedef pcl::SHOT352 ShapeDescriptorType;
    typedef pcl::PointCloud<ShapeDescriptorType> ShapeDescriptorCloud;
    typedef ShapeDescriptorCloud::Ptr ShapeDescriptorCloudPtr;

    typedef pcl::SHOT1344 DescriptorType;
    typedef pcl::PointCloud<DescriptorType> DescriptorCloud;
    typedef DescriptorCloud::Ptr DescriptorCloudPtr;

    typedef pcl::Normal NormalType;
    typedef pcl::PointCloud<NormalType> NormalCloud;
    typedef typename NormalCloud::Ptr NormalCloudPtr;


    typedef boost::shared_ptr<DynamicObject> Ptr;

   DynamicObject();
   ~DynamicObject();

   void setTime(boost::posix_time::ptime);
   void setCloud(CloudPtr);

   void updateAllData();
   void computeCentroid();
   void computeBbox();
   void computeTree();
   void computeUniformSamplingKeypoints(double);
   void computeNormals(double);
   void computeFeatures(double); // uses m_keypoints and m_features
   void computeShapeFeatures(double); // uses m_keypoints and m_features
   void computeVFHFeatures();


   bool matchSingleObject(DynamicObject::Ptr other);
//   bool matchObjectCluster(boost::shared_ptr<ObjectMatches> others, double max_dist_from_centroid=.5); // match a few of the objects from the object cluster - the ones within 1 m of the cluster centroid

   bool matchDynamicObjectHelper(DynamicObject::Ptr other, std::vector<pcl::Correspondences>&, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >&);

   bool refineMatchBruteForce(DynamicObject::Ptr, double& fitnessScore, Eigen::Matrix4f& transform, CloudPtr bestMatch, double maxFitnessScore,
                              std::vector<Eigen::Matrix4f> allBruteForceTransforms,
                              bool downsample = true, bool twoWay = true, bool eitherWay = false);

   void saveCloud(std::string path);

   bool operator==(const DynamicObject& rhs); // equality operator -> deep comparison of all fields
   bool operator!=(const DynamicObject& rhs); // equality operator -> deep comparison of all fields

   static void computeFitness(CloudPtr src, CloudPtr tgt, double& fitnessScore, int& noPoints, double max_range);



private:

public:
   CloudPtr                         m_points;
   CloudPtr                         m_keypoints;
   DescriptorCloudPtr               m_descriptors;
   ShapeDescriptorCloudPtr          m_shapeDescriptors;
   NormalCloudPtr                   m_normals;
   TreePtr                          m_tree;
   std::string                      m_label;
   PointType                        m_bboxHighest, m_bboxLowest;
   Eigen::Vector4f                  m_centroid;
   boost::posix_time::ptime         m_time;
   std::vector<float>               m_vfh;



};

#endif // __DYNAMIC_OBJECT__H
