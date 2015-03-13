#include "semantic_map/dynamic_object.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <flann/flann.h>


using namespace std;

DynamicObject::DynamicObject() : m_points(new Cloud()), m_keypoints(new Cloud()), m_shapeDescriptors(new ShapeDescriptorCloud()),
   m_descriptors(new DescriptorCloud), m_normals(new NormalCloud), m_tree(new Tree), m_label("unknown")
{
}

DynamicObject::~DynamicObject()
{

}

void DynamicObject::setTime(boost::posix_time::ptime time)
{
   m_time = time;
}

void DynamicObject::setCloud(CloudPtr cloud)
{
   m_points = cloud->makeShared();
   computeCentroid();
   computeBbox();
}

void DynamicObject::updateAllData()
{
   computeCentroid();
   computeBbox();
   computeTree();
   computeUniformSamplingKeypoints(0.05);
   computeNormals(0.1);
   computeFeatures(0.08);

}


void DynamicObject::computeCentroid()
{
   pcl::compute3DCentroid(*m_points, m_centroid);
}

void DynamicObject::computeBbox()
{
   double max_z = 100000.0;
   m_bboxHighest.z = -max_z;m_bboxHighest.y = -max_z;m_bboxHighest.x = -max_z;
   m_bboxLowest.z = max_z;m_bboxLowest.y = max_z;m_bboxLowest.x = max_z;

   for (size_t i=0; i<m_points->points.size(); i++)
   {
      if (m_points->points[i].z > m_bboxHighest.z)
      {
         m_bboxHighest.z = m_points->points[i].z;
      }

      if (m_points->points[i].z < m_bboxLowest.z)
      {
         m_bboxLowest.z = m_points->points[i].z;
      }

      if (m_points->points[i].y > m_bboxHighest.y)
      {
         m_bboxHighest.y = m_points->points[i].y;
      }

      if (m_points->points[i].y < m_bboxLowest.y)
      {
         m_bboxLowest.y = m_points->points[i].y;
      }

      if (m_points->points[i].x > m_bboxHighest.x)
      {
         m_bboxHighest.x = m_points->points[i].x;
      }

      if (m_points->points[i].x < m_bboxLowest.x)
      {
         m_bboxLowest.x = m_points->points[i].x;
      }
   }
}

void DynamicObject::computeTree()
{
   m_tree->setInputCloud(m_points);
}

void DynamicObject::computeNormals(double radius_search)
{
   pcl::NormalEstimationOMP<PointType, NormalType> normalEstimation;
   normalEstimation.setNumberOfThreads(11);
   //   normalEstimation.set
   normalEstimation.setInputCloud(m_keypoints);
   normalEstimation.setRadiusSearch(radius_search);
   Tree::Ptr kdtree(new Tree);
   normalEstimation.setSearchMethod(kdtree);
   normalEstimation.compute(*m_normals);
}

void DynamicObject::computeFeatures(double search_radius)
{
   pcl::SHOTColorEstimationOMP<PointType, NormalType, DescriptorType> shot;
   shot.setNumberOfThreads(11);
   Tree::Ptr tree (new Tree());
   shot.setRadiusSearch(search_radius);
   shot.setInputCloud(m_keypoints);
   shot.setInputNormals(m_normals);
   //   shot.setSearchSurface(m_points);
   shot.compute(*m_descriptors);

   cout<<"Computing descriptors for object with points: "<<m_points->points.size()<<"  keypoints: "<<m_keypoints->points.size()<<endl;
}

void DynamicObject::computeShapeFeatures(double search_radius)
{
   if (m_keypoints->points.size() == 0)
   {
      computeUniformSamplingKeypoints(0.05);
      computeNormals(0.1);
   }

   pcl::SHOTEstimationOMP<PointType, NormalType, ShapeDescriptorType> shot;
   shot.setNumberOfThreads(11);
   Tree::Ptr tree (new Tree());
   shot.setRadiusSearch(search_radius);
   //   shot.setInputCloud(m_keypoints);
   shot.setInputCloud(m_points);
   shot.setInputNormals(m_normals);
   //   shot.setSearchSurface(m_points);
   shot.compute(*m_shapeDescriptors);

   cout<<"Computing shape descriptors for object with points: "<<m_points->points.size()<<"  keypoints: "<<m_keypoints->points.size()<<endl;
}

void DynamicObject::computeUniformSamplingKeypoints(double voxel_size)
{
   // Filter object.
   pcl::VoxelGrid<pcl::PointXYZRGB> filter;
   filter.setInputCloud(m_points);
   filter.setLeafSize(voxel_size, voxel_size, voxel_size);
   filter.filter(*m_keypoints);
}

bool DynamicObject::matchSingleObject(DynamicObject::Ptr other)
{

   static bool firstTime = true;
   static std::vector<Eigen::Matrix4f> allBruteForceTransforms;
   long noIterations = 36864;

   if (firstTime)
   {

      allBruteForceTransforms.reserve(noIterations);
      // initialize all transforms array
      double x_interval = 0.2;
      double y_interval = 0.2;
      double z_interval = 0.2;
      int step_x = 4;
      int step_y = 4;
      int step_z = 4;
      int step_theta = 36;

      double th_interval = 180.0;
      double inc_x = x_interval / (double)step_x;
      double inc_y = y_interval / (double)step_y;
      double inc_z = z_interval / (double)step_z;
      double inc_th = th_interval/ (double)step_theta;

      long iteration = 0;
      for (double it_x = -x_interval; it_x < x_interval; it_x+=inc_x)
      {
         for (double it_y = -y_interval; it_y < y_interval; it_y+=inc_y)
         {
            for (double it_z = -z_interval; it_z < z_interval; it_z+=inc_z)
            {
               for (double it_th = -th_interval; it_th < th_interval; it_th+=inc_th)
               {

                  Eigen::AngleAxisf init_rotation (it_th, Eigen::Vector3f::UnitZ ());
                  Eigen::Translation3f init_translation (it_x, it_y, it_z);
                  Eigen::Matrix4f transform = (init_rotation * init_translation ).matrix ();
                  allBruteForceTransforms.push_back(transform);

                  iteration++;
               }
            }
         }
      }

      cout<<"Initialized brute force transforms matrix with "<<iteration<<" elements. Matrix size "<<allBruteForceTransforms.size()<<endl;
      firstTime = false;
   }


   std::vector<pcl::Correspondences> clusteredCorrespondences;
   std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;

//   bool match = matchDynamicObjectHelper(other, clusteredCorrespondences, transformations);
   bool match = true;

   if (match)
   {
      // move both clusters to the origin
      CloudPtr transformed(new Cloud()), transformedOther(new Cloud());

      Eigen::Affine3f translation_ref(Eigen::Translation3f( - m_centroid[0], - m_centroid[1], /*- m_bboxHighest.z*/0.0)); // do not change the height -> implicit assumption about supporting structure
      Eigen::Affine3f translation_matched(Eigen::Translation3f(- other->m_centroid[0], - other->m_centroid[1], /*- other->m_bboxHighest.z */0.0)); // do not change the height -> implicit assumption about supporting structure
      Eigen::Matrix4f transform_ref = translation_ref.matrix();
      Eigen::Matrix4f transform_matched = translation_matched.matrix();

      pcl::transformPointCloud (*m_points, *transformed, transform_ref);
      pcl::transformPointCloud (*other->m_points, *transformedOther, transform_matched);

      // brute force checking

      DynamicObject::Ptr temp_me(new DynamicObject), temp_other (new DynamicObject);
      temp_me->setCloud(transformed);
      temp_other->setCloud(transformedOther);
//      char** argv;
//      int argc = 0;

//            static  pcl::visualization::PCLVisualizer * viewer = new pcl::visualization::PCLVisualizer (argc, argv, "Object viewer");
//            pcl::visualization::PointCloudColorHandlerCustom<PointType> me_handler (transformed, 0, 255, 0);
//            pcl::visualization::PointCloudColorHandlerCustom<PointType> other_handler (transformedOther, 0, 0, 255);
//            viewer->addPointCloud (transformed, me_handler, "me");
//            viewer->addPointCloud (transformedOther, other_handler, "other");
//            viewer->spin();
//            viewer->removeAllPointClouds();

      Eigen::Matrix4f bestTransform;
      CloudPtr bestMatch;
      double bestScore;

      return temp_me->refineMatchBruteForce(temp_other, bestScore, bestTransform, bestMatch, 0.005/*0.01*/,allBruteForceTransforms, true, true, false );

   } else {
      return false;
   }


}

bool DynamicObject::matchDynamicObjectHelper(DynamicObject::Ptr other, std::vector<pcl::Correspondences>& clusteredCorrespondences,
                                             std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& transformations)
{
   //   if ((m_descriptors->points.size() == 0) || (other->m_descriptors->points.size() == 0))
   if ((m_descriptors->points.size() < 10) || (other->m_descriptors->points.size() < 10))
   {
      //      ROS_ERROR_STREAM("Object descriptor vector has 0 elements.");
      PCL_ERROR("Object descriptor vector has 0 elements.");
      return false;
   }

   cout<<"Matching helper. No descriptors "<<m_descriptors->points.size()<<endl;

   //    pcl::KdTreeFLANN<DescriptorType, ::flann::L1<float>> matching;
   pcl::KdTreeFLANN<DescriptorType, ::flann::L2<float>> matching;
   matching.setInputCloud(m_descriptors);

   pcl::CorrespondencesPtr correspondences = pcl::CorrespondencesPtr(new pcl::Correspondences());

   // Check every descriptor computed for the scene.
   for (size_t i = 0; i < other->m_descriptors->size(); ++i)
   {
      std::vector<int> neighbors(2);
      std::vector<float> squaredDistances(2);
      // Ignore NaNs.
      if (pcl_isfinite(other->m_descriptors->at(i).descriptor[0]))
      {
         int neighborCount = matching.nearestKSearch(other->m_descriptors->at(i), 2, neighbors, squaredDistances);

         if (neighborCount == 2 && squaredDistances[0] / squaredDistances[1] < 0.8)
         {
            pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
            correspondences->push_back(correspondence);
         }
      }
   }
   std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;

   {

      pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;


      // Resolution of the consensus set used to cluster correspondences together,
      // in metric units. Default is 1.0.
      gc_clusterer.setGCSize (0.025);
      // Minimum cluster size. Default is 3 (as at least 3 correspondences
      // are needed to compute the 6 DoF pose).
      gc_clusterer.setGCThreshold (3);

      gc_clusterer.setInputCloud (m_keypoints);
      gc_clusterer.setSceneCloud (other->m_keypoints);
      gc_clusterer.setModelSceneCorrespondences (correspondences);
      gc_clusterer.recognize (transformations, clusteredCorrespondences);
   }

   std::cout << "Model instances found: " << transformations.size () << std::endl;

   return (clusteredCorrespondences.size() > 0);
}

void DynamicObject::computeFitness(CloudPtr one, CloudPtr other, double& fitnessScore, int& noPoints, double max_range)
{
   noPoints = 0;
   fitnessScore = 0;

   TreePtr tree(new Tree);
   tree->setInputCloud(other);


   std::vector<int> nn_indices (1);
   std::vector<float> nn_dists (1);

   // For each point in the source dataset
   for (size_t i = 0; i < one->points.size (); ++i)
   {
      Eigen::Vector4f p1 = Eigen::Vector4f (one->points[i].x,
                                            one->points[i].y,
                                            one->points[i].z, 0);
      // Find its nearest neighbor in the target
      tree->nearestKSearch (one->points[i], 1, nn_indices, nn_dists);

      // Deal with occlusions (incomplete targets)
      if (nn_dists[0] > max_range)
         continue;

      Eigen::Vector4f p2 = Eigen::Vector4f (other->points[nn_indices[0]].x,
            other->points[nn_indices[0]].y,
            other->points[nn_indices[0]].z, 0);
      // Calculate the fitness score
      fitnessScore += fabs ((p1-p2).squaredNorm ());
      noPoints++;
   }

   if (noPoints > 0)
      fitnessScore /= noPoints;
   else
      fitnessScore = std::numeric_limits<double>::max ();


}

bool DynamicObject::refineMatchBruteForce(DynamicObject::Ptr other, double& fitnessScore, Eigen::Matrix4f& bestTransform, CloudPtr bestMatch, double maxFitnessScore,
                                          std::vector<Eigen::Matrix4f> allBruteForceTransforms,
                                          bool downsample, bool twoWay, bool eitherWay)
{

   bestMatch = CloudPtr(new Cloud());
   Cloud::Ptr src (new Cloud);
   Cloud::Ptr tgt (new Cloud);
   pcl::VoxelGrid<PointType> grid;
   if (downsample)
   {
      grid.setLeafSize (0.05, 0.05, 0.05);
      grid.setInputCloud (m_points);
      grid.filter (*src);

      grid.setInputCloud (other->m_points);
      grid.filter (*tgt);
   } else {
      *src = *m_points;
      *tgt = *other->m_points;
   }

   double originalScore = 0;
   int originalPoints = 0;
   DynamicObject::computeFitness(src, tgt, originalScore, originalPoints, 1.0);
   cout<<"Original fitness score "<<originalScore<<endl;
   const clock_t begin_time = clock();

   CloudPtr transformedCloud(new Cloud());
   *transformedCloud = *src;

   struct iteration_result
   {
      double fitness;
      Eigen::Matrix4f transform;
      iteration_result(double f, Eigen::Matrix4f t)
      {
         fitness = f;
         transform = t;
      }
      iteration_result()
      {}
   };

   Eigen::Matrix4f best_transform;
   vector<iteration_result> bruteForceResult(allBruteForceTransforms.size(), iteration_result(0, best_transform));
   long iteration = 0;
   double best_fitness = std::numeric_limits<double>::max();

#pragma omp parallel for num_threads(11)
   for (long iteration=0; iteration<allBruteForceTransforms.size();iteration++)
   {

      CloudPtr tempCloud( new Cloud());
      Eigen::Matrix4f transform = allBruteForceTransforms[iteration];
      pcl::transformPointCloud (*src, *tempCloud, transform);

      double fitnessScore = 0;
      int noPoints = 0;
      DynamicObject::computeFitness(tempCloud, tgt, fitnessScore, noPoints, 1.0);


      bruteForceResult[iteration] = (iteration_result(fitnessScore,allBruteForceTransforms[iteration]));
   }


   auto best_iteration = std::min_element(bruteForceResult.begin(), bruteForceResult.end(),[](iteration_result const& a, iteration_result const& b){return a.fitness < b.fitness;});

   best_transform = best_iteration->transform;
   best_fitness = best_iteration->fitness;

   pcl::transformPointCloud (*src, *transformedCloud, best_transform);
   double otherwayfitness = 0.0;
   int otherwayPoints = 0;
   DynamicObject::computeFitness(tgt, transformedCloud, otherwayfitness, otherwayPoints, 1.0);

   // returning the transformed cloud according to the best match
   pcl::transformPointCloud (*src, *bestMatch, best_transform);

   cout<<"Start fitness: "<<originalScore<<endl;
   cout<<"Best fitness: "<<best_fitness<<endl;
   cout<<"Otherway fitness: "<<otherwayfitness<<endl;
   cout<<"Avg fitness: "<<(otherwayfitness + best_fitness) /2 <<endl;
   cout<<"Time: "<<float( clock () - begin_time ) /  CLOCKS_PER_SEC <<endl;
   cout<<"Iterations "<<iteration<<endl;
   cout<<"Matrix size "<<allBruteForceTransforms.size()<<"  "<<bruteForceResult.size()<<endl;

   if (eitherWay)
   {
      if ((otherwayfitness < maxFitnessScore) || (best_fitness < maxFitnessScore))
      {
         return true;
      } else {
         return false;
      }
   }

   if (twoWay)
   {

      if ((otherwayfitness + best_fitness) /2 < maxFitnessScore)
      {
         return true;
      } else {
         return false;
      }
   } else if ((best_fitness < maxFitnessScore) && (otherwayfitness < maxFitnessScore))
   {
      return true;
   } else {
      return false;
   }


}

void DynamicObject::saveCloud(std::string path)
{
   pcl::io::savePCDFileBinary(path, *m_points);
}


bool DynamicObject::operator==(const DynamicObject& rhs)
{
   if (m_label != rhs.m_label)
   {
      cout<<"Label not the same "<<endl;
      return false;
   }

   //   if (m_centroid[0] != rhs.m_centroid[0])
   //   {
   //      cout<<"Centroid not the same "<<endl;
   //      cout<<m_centroid<<"  "<<rhs.m_centroid<<endl;
   //      return false;
   //   }

   //   if (m_bboxHighest != rhs.m_bboxHighest)
   //      return false;

   //   if (m_bboxLowest != rhs.m_bboxLowest)
   //      return false;

   if (m_points->points.size() != rhs.m_points->points.size())
   {
      cout<<"No points not the same "<<endl;
      return false;
   }

   for (size_t i=0; i<m_points->points.size(); i++)
   {
      if ((m_points->points[i].x != rhs.m_points->points[i].x) ||
          (m_points->points[i].y != rhs.m_points->points[i].y) ||
          (m_points->points[i].z != rhs.m_points->points[i].z))
      {
         cout<<"Point not the same "<<endl;
         return false;
      }
   }

   return true;
}

bool DynamicObject::operator!=(const DynamicObject& rhs)
{
   return ! (*this==rhs);
}


//bool DynamicObject::matchObjectCluster(ObjectMatches::Ptr others,double max_dist_from_centroid)
//{
//   auto matches = others->getObjectsWithinDistance(max_dist_from_centroid);
//   for (DynamicObject::Ptr object : matches)
//   {
//      bool match = matchSingleObject(object);
//      if (match)
//      {
//         return true;
//      }
//   }

//   return false;
//}

void DynamicObject::computeVFHFeatures()
{
   pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
   vfh.setInputCloud (m_points);
   vfh.setInputNormals (m_normals);
   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
   vfh.setSearchMethod (tree);

   pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
   // Compute the features
   vfh.compute (*vfhs);
   cout<<"Computed VFH descriptor "<<endl;
   m_vfh.resize(308);

   for (size_t i = 0; i < 308; ++i)
   {
      m_vfh[i] = vfhs->points[0].histogram[i];
   }

}
