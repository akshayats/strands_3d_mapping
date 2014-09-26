#ifndef __PLANE_DETECTOR__H
#define __PLANE_DETECTOR__H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <string>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <pcl/io/pcd_io.h>

#include <float.h>


template <class PointType>
class PlaneDetector {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    PlaneDetector(CloudPtr cloud)
    {
        m_Cloud = cloud;
    }

    ~PlaneDetector()
    {

    }



    void setInputCloud(CloudPtr new_cloud)
    {
            m_Cloud = new_cloud;
    }

    std::vector<pcl::ModelCoefficients::Ptr> getBoundingPlanes()
    {
        return m_BoundingPlanes;
    }

    std::vector<bool> getBoundingPlanesDirections()
    {
        return m_BoundingPlanesDirections;
    }


    CloudPtr detectGroundPlane()
    {
        pcl::PassThrough<PointType> lower_pass;
        CloudPtr lower_cloud (new Cloud);
        CloudPtr cloud_plane (new Cloud ());
        CloudPtr temp_cloud (new Cloud);
        temp_cloud = m_Cloud;

        lower_pass.setInputCloud (temp_cloud);
        lower_pass.setFilterFieldName ("z");
        lower_pass.setFilterLimits (-.2, .2);
        lower_pass.filter (*lower_cloud);

        pcl::SACSegmentation<PointType> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);
        seg.setAxis(Eigen::Vector3f(0,0,1));
        seg.setEpsAngle (0.1);

        seg.setInputCloud (lower_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            //      break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud (lower_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        for (size_t i=0; i<coefficients->values.size();i++)
        {
            std::cout<<"Coefficients "<<coefficients->values[i]<<std::endl;
        }

        m_BoundingPlanes.push_back(coefficients);
        m_BoundingPlanesDirections.push_back(true);

        double tolerance = 0.05; // enough for the ceiling plane
        CloudPtr floor_cloud = PlaneDetector::extractPlane(m_Cloud, coefficients, tolerance);


        return floor_cloud;

    }

    CloudPtr detectCeilingPlane()
    {
        pcl::PassThrough<PointType> upper_pass;
        CloudPtr upper_cloud (new Cloud);
        CloudPtr cloud_plane (new Cloud());
        CloudPtr temp_cloud (new Cloud);

        temp_cloud = m_Cloud;
        upper_pass.setInputCloud (temp_cloud);
        upper_pass.setFilterFieldName ("z");
        upper_pass.setFilterLimits (2.5, 3.5);
        upper_pass.filter (*upper_cloud);

        pcl::SACSegmentation<PointType> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);
        seg.setAxis(Eigen::Vector3f(0,0,1));
        seg.setEpsAngle (0.1);

        seg.setInputCloud (upper_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            //      break;
        }

        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud (upper_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        for (size_t i=0; i<coefficients->values.size();i++)
        {
            std::cout<<"Coefficients "<<coefficients->values[i]<<std::endl;
        }

        m_BoundingPlanes.push_back(coefficients);
        m_BoundingPlanesDirections.push_back(false);

        double tolerance = 0.1; // enough for the ceiling plane
        CloudPtr ceiling_cloud = PlaneDetector::extractPlane(m_Cloud, coefficients, tolerance);

//        {
//            static int i=0;
//            pcl::PCDWriter writer;
//            std::stringstream ss;
//            ss << "without_top_plane";
//            ss << i;
//            ss << ".pcd";
//            writer.write<PointType> (ss.str (), *m_Cloud, false); //*
//            i++;
//        }

        return ceiling_cloud;
    }

    std::pair<CloudPtr, CloudPtr> detectWalls()
    {
        std::pair<CloudPtr, CloudPtr> toRet;
        CloudPtr walls_cloud (new Cloud);
        CloudPtr cloud_plane (new Cloud ());
        CloudPtr cloud_f (new Cloud);

        walls_cloud = m_Cloud->makeShared();

        pcl::SACSegmentation<PointType> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.15);
        seg.setAxis(Eigen::Vector3f(0,0,1));
        seg.setEpsAngle (0.1);

        int nr_points = walls_cloud->points.size();
        int plane_index = 0;
        pcl::PCDWriter writer;

        std::vector<CloudPtr> allPlanes;
        std::vector<Eigen::Vector4f> allCentroids;
        std::vector<pcl::ModelCoefficients::Ptr> allCoefficients;
        std::vector<pcl::PointIndices::Ptr> allInliers;

        while (walls_cloud->points.size () > 0.3 * nr_points)
        {


            seg.setInputCloud (walls_cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            pcl::ExtractIndices<PointType> extract;
            extract.setInputCloud (walls_cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);

            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;


//            {
//                std::stringstream ss;
//                ss << "original_plane"<<plane_index<<".pcd";
//                writer.write<PointType> (ss.str (), *cloud_plane, false); //*
//                plane_index++;
//            }

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_plane, centroid);

            CloudPtr new_cloud  = cloud_plane->makeShared();
            pcl::ModelCoefficients::Ptr new_coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr new_inliers(new pcl::PointIndices);
            *new_inliers = *inliers;
            *new_coefficients= *coefficients;

            allPlanes.push_back(new_cloud);
            allCoefficients.push_back(new_coefficients);
            allCentroids.push_back(centroid);
            allInliers.push_back(new_inliers);

            extract.setNegative (true);
            extract.filter (*cloud_f);
            *walls_cloud = *cloud_f;
        }

        std::vector<int> before(allPlanes.size(),0);
        std::vector<int> after(allPlanes.size(),0);

        for (size_t i=0; i<allPlanes.size(); i++)
        {
            for (size_t j=0; j<allPlanes.size(); j++)
            {
                if (j == i) continue;
                // compare these planes
                double dotproduct = allCoefficients[i]->values[0] * allCentroids[j][0] +
                        allCoefficients[i]->values[1] * allCentroids[j][1] +
                        allCoefficients[i]->values[2] * allCentroids[j][2] +
                        allCoefficients[i]->values[3];

                // check the dot product
                // added another check -> compare point cloud size, only compare to equally sized planes
                double ratio = 0.1;
                if (allPlanes[j]->points.size() >ratio*allPlanes[i]->points.size())
                {
                    if (dotproduct >= 0.0)
                    {
                        before[i]++;
                    } else {
                        after[i]++;
                    }
                }
            }
        }

        walls_cloud->points.clear();
        plane_index = 0;
        CloudPtr originalCloud(new Cloud());
        *originalCloud = *m_Cloud;
        for (size_t i=0; i<allPlanes.size(); i++)
        {
            pcl::ExtractIndices<PointType> extract;
            if ((before[i] == 0) || (after[i] == 0))
            {
                // boundary plane -> save it

                double tolerance = 0.15;
                CloudPtr new_cloud = PlaneDetector::extractPlane(m_Cloud,allCoefficients[i], tolerance);
                // extract a much finer plane from the cloud found
                {
//                    pcl::SACSegmentation<PointType> fine_seg;
//                    pcl::PointIndices::Ptr fine_inliers (new pcl::PointIndices);
//                    pcl::ModelCoefficients::Ptr fine_coefficients (new pcl::ModelCoefficients);

//                    fine_seg.setOptimizeCoefficients (true);
//                    fine_seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
//                    fine_seg.setMethodType (pcl::SAC_RANSAC);
//                    fine_seg.setMaxIterations (100);
//                    fine_seg.setDistanceThreshold (0.05);
//                    fine_seg.setAxis(Eigen::Vector3f(0,0,1));
//                    fine_seg.setEpsAngle (0.05);

//                    fine_seg.setInputCloud (new_cloud);
//                    fine_seg.segment (*fine_inliers, *fine_coefficients);
//                    if (fine_inliers->indices.size () == 0)
//                    {
//                        std::cout << "Could not extract a finer wall cloud. Keeping original one" << std::endl;
//                    } else {
//                        pcl::ExtractIndices<PointType> fine_extract;
//                        fine_extract.setInputCloud (new_cloud);
//                        fine_extract.setIndices (fine_inliers);
//                        fine_extract.setNegative (false);
//                        // Get the points associated with the planar surface
//                        CloudPtr fine_cloud(new Cloud());
//                        fine_extract.filter (*fine_cloud);
//                        *new_cloud = *fine_cloud;
//                        *allCoefficients[i] = *fine_coefficients;
//                    }


                    m_BoundingPlanes.push_back(allCoefficients[i]);
                    if (before[i] == 0)
                    {
                        m_BoundingPlanesDirections.push_back(false);
                    } else {
                        m_BoundingPlanesDirections.push_back(true);
                    }


                    // iterate through the original point cloud and extract the points belonging to this plane

    //                {
    //                    std::stringstream ss;
    //                    ss << "plane"<<plane_index<<".pcd";
    //                    writer.write<PointType> (ss.str (), *new_cloud, false); //*
    //                    plane_index++;
    //                }
                    *walls_cloud += *new_cloud;

//                    if (m_BoundingPlanes.size() == 2)
//                    {
//                        // enough planes
//                        break;
//                    }

                }







            }
            std::cout<<"before :"<<before[i] << ";;; after:  "  << after[i] <<";;; inliers: "<<allInliers[i]->indices.size()<<std::endl;
        }

        double tolerance = 0.05;

        if(m_BoundingPlanes.size()>3) // more than 4 walls
        {
            for (size_t i=0; i<m_BoundingPlanes.size(); i++)
            {
                long int inliers = PlaneDetector::countInliers(originalCloud, m_BoundingPlanes[i],m_BoundingPlanesDirections[i]);
                std::cout<<"Bounding plane "<<i<<" inliers "<<inliers<<std::endl;
            }
        }

        CloudPtr ceiling_outliers = PlaneDetector::removeOutliers(m_Cloud,m_BoundingPlanes,tolerance,m_BoundingPlanesDirections);
        std::cout << "PointCloud representing the boundary outliers has : " << ceiling_outliers->points.size () << " data points." << std::endl;

    //    pcl::PCDWriter writer;
//        std::stringstream ss;
//        ss << "ceiling_outliers"<<".pcd";
//        writer.write<PointType> (ss.str (), *ceiling_outliers, false); //*

//        {
//            std::stringstream ss;
//            ss << "remaining_cloud"<<".pcd";
//            writer.write<PointType> (ss.str (), *m_Cloud, false); //*
//        }
        toRet.first = m_Cloud;
        toRet.second = walls_cloud;
        return toRet;

    }

    static void computeBoundingBox(CloudPtr input, double& min_x, double& max_x,double& min_y,double& max_y,double& min_z,double& max_z)
    {
        min_x = DBL_MAX; max_x = -DBL_MAX;
        min_y = DBL_MAX; max_y = -DBL_MAX;
        min_z = DBL_MAX; max_z = -DBL_MAX;

        for (size_t i=0; i<input->points.size(); i++)
        {
            if (input->points[i].x > max_x)
            {
                max_x = input->points[i].x;
            }
            if (input->points[i].y > max_y)
            {
                max_y = input->points[i].y;
            }
            if (input->points[i].z > max_z)
            {
                max_z = input->points[i].z;
            }
            if (input->points[i].x < min_x)
            {
                min_x = input->points[i].x;
            }
            if (input->points[i].y < min_y)
            {
                min_y = input->points[i].y;
            }
            if (input->points[i].z < min_z)
            {
                min_z = input->points[i].z;
            }
        }
    }

    static void computeHistogram(CloudPtr input, const double& min_x, const double& max_x,const double& min_y,const double& max_y,\
                                   const double& min_z,const double& max_z, std::vector<int>& hist_x, std::vector<int>& hist_y, std::vector<int>& hist_z, int num_bins)
    {
        hist_x.resize(num_bins +1);
        hist_y.resize(num_bins +1);
        hist_z.resize(num_bins +1);

        hist_x = std::vector<int>(num_bins, 0);
        hist_y = std::vector<int>(num_bins, 0);
        hist_z = std::vector<int>(num_bins, 0);

        float num_binsf = (float)num_bins;

        for (size_t i=0; i<input->points.size(); i++)
        {
            // x
            if ((input->points[i].x >= min_x) && (input->points[i].x <= max_x))
            {
                // compute bin
                double percentage = (input->points[i].x - min_x)/(max_x - min_x);
                int bin_index = (int)(percentage*num_binsf);
                hist_x[bin_index]++;
            }
            // y
            if ((input->points[i].y >= min_y) && (input->points[i].y <= max_y))
            {
                // compute bin
                double percentage = (input->points[i].y - min_y)/(max_y - min_y);
                int bin_index = (int)(percentage*num_binsf);
                hist_y[bin_index]++;
            }
            // z
            if ((input->points[i].z >= min_z) && (input->points[i].z <= max_z))
            {
                // compute bin
                double percentage = (input->points[i].z - min_z)/(max_z - min_z);
                int bin_index = (int)(percentage*num_binsf);
                hist_z[bin_index]++;
            }
        }
    }

    static CloudPtr extractPlane(CloudPtr input, pcl::ModelCoefficients::Ptr coefficients, const double& tolerance)
    {
        // filter the points out of the original cloud
        pcl::PointIndices::Ptr indices (new pcl::PointIndices);
        // iterate through the original point cloud and extract the points belonging to this plane
        for (size_t j=0; j<input->points.size(); j++)
        {
            double dotproduct = coefficients->values[0] * input->points[j].x +
                    coefficients->values[1] * input->points[j].y +
                    coefficients->values[2] * input->points[j].z +
                    coefficients->values[3];

            if (fabs(dotproduct) < tolerance)
            {
                indices->indices.push_back(j);
            }

        }
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud (input);
        extract.setIndices (indices);
        extract.setNegative (false);

        CloudPtr filtered_cloud(new Cloud);
        extract.filter (*filtered_cloud);


        extract.setNegative (true);
        CloudPtr temp_cloud(new Cloud);
        extract.filter (*temp_cloud);
        *input = *temp_cloud;

        return filtered_cloud;
    }

    static int countInliers(CloudPtr input, pcl::ModelCoefficients::Ptr coefficients, bool direction)
    {
        int inliers =0;

        for (size_t j=0; j<input->points.size(); j++)
        {
                double dotproduct = coefficients->values[0] * input->points[j].x +
                        coefficients->values[1] * input->points[j].y +
                        coefficients->values[2] * input->points[j].z +
                        coefficients->values[3];

                if (((dotproduct < 0.0) && !direction) || ((dotproduct > 0.0) && direction))
                {
                    inliers++;
                }
        }

        return inliers;

    }

    static CloudPtr removeOutliers(CloudPtr input, std::vector<pcl::ModelCoefficients::Ptr> coefficients, const double& tolerance, std::vector<bool> direction)
    {
        // convention if the dot product is < 0 the point lies behind the plane, i.e. corresponding boolean value should be false for the point to be removed

        pcl::PointIndices::Ptr indices (new pcl::PointIndices);
        // iterate through the original point cloud and extract the points belonging to this plane
        for (size_t j=0; j<input->points.size(); j++)
        {
            for (size_t k=0; k<coefficients.size(); k++)
            {
                double dotproduct = coefficients[k]->values[0] * input->points[j].x +
                        coefficients[k]->values[1] * input->points[j].y +
                        coefficients[k]->values[2] * input->points[j].z +
                        coefficients[k]->values[3];

                if (((dotproduct < -tolerance) && direction[k]) || ((dotproduct > tolerance) && !direction[k]))
                {
                    indices->indices.push_back(j);
                    break;
                }
    //            cout<<dotproduct<<endl;
            }
        }
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud (input);
        extract.setIndices (indices);
        extract.setNegative (false);

        CloudPtr filtered_cloud(new Cloud);
        extract.filter (*filtered_cloud);


        extract.setNegative (true);
        CloudPtr temp_cloud(new Cloud);
        extract.filter (*temp_cloud);
        *input = *temp_cloud;

        return filtered_cloud;

    }

    static CloudPtr downsampleCloud(CloudPtr input)
    {
        std::cout << "PointCloud before filtering has: " << input->points.size () << " data points." << std::endl; //*

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<PointType> vg;
        CloudPtr cloud_filtered (new Cloud);
        vg.setInputCloud (input);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
        std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*    \

        return cloud_filtered;
    }

private:
    CloudPtr                 m_Cloud;
    std::vector<pcl::ModelCoefficients::Ptr>            m_BoundingPlanes;
    std::vector<bool>                                   m_BoundingPlanesDirections;

};


#endif //__PLANE_DETECTOR__H
