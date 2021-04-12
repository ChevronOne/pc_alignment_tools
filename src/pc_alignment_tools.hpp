

//  Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
//
//  This file is part of the 'pc_alignment_tools' utilities.
//
//  Distributed under the terms of the GNU General Public License
//  as published by the Free Software Foundation; You should have
//  received a copy of the GNU General Public License.
//  If not, see <http://www.gnu.org/licenses/>.
//
//
//  This software is distributed in the hope that it will be useful, but WITHOUT
//  WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
//  WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND
//  NON-INFRINGEMENT. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR ANYONE
//  DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY,
//  WHETHER IN CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. See the GNU
//  General Public License for more details.

/*
 * Copyright Abbas Mohammed Murrey 2020-21
 *
 * Permission to use, copy, modify, distribute and sell this software
 * for any purpose is hereby granted without fee, provided that the
 * above copyright notice appear in all copies and that both the copyright
 * notice and this permission notice appear in supporting documentation.
 * I make no representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 */







#ifndef PC_ALIGNMENT_TOOLS_HPP
#define PC_ALIGNMENT_TOOLS_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/topic.h"
#include "ros/duration.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
//#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "eigen_conversions/eigen_msg.h"
#include "sensor_msgs/PointCloud2.h"

#include "std_msgs/Header.h"
#include <map>

#include <ros/console.h>
#include "ros/package.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/io/obj_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/console/time.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/features/normal_3d.h>

#include <sstream>
#include <fstream>
#include <regex>
#include <thread>
#include <chrono>
#include <set>
#include <ctime>
#include <type_traits>

#include "tf2_eigen/tf2_eigen.h"
#include <boost/progress.hpp>
#include <boost/shared_ptr.hpp>

using namespace std::chrono_literals;

namespace pc_alignment_tools {


typedef pcl::visualization::PCLVisualizer pclVis;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproximatePolicy;

typedef pcl::PointXYZ P_type;
typedef pcl::Normal N_type;
typedef pcl::PointNormal PN_type;

typedef pcl::GeneralizedIterativeClosestPoint<P_type, P_type> gicp_aligner;
typedef pcl::IterativeClosestPoint<P_type, P_type> point_to_point_aligner;
typedef pcl::IterativeClosestPointNonLinear<P_type, P_type> nl_point_to_point_aligner;
typedef pcl::IterativeClosestPointWithNormals<PN_type, PN_type> point_to_plane_aligner;

typedef pcl::registration::TransformationEstimationPointToPlane<PN_type, PN_type, double> corres_filter_point_to_plane_aligner;
typedef pcl::registration::TransformationEstimationSVD<P_type, P_type, double> corres_filter_point_to_point_aligner;




template<class Type>
Eigen::Quaternion<Type> quat_rosToEigen(const tf2::Quaternion& q){

	return Eigen::Quaternion<Type>(q[3], q[0], q[1], q[2]);

}



template<class Type>
Eigen::Quaternion<Type> quat_rosToEigen(const geometry_msgs::Quaternion& q){

	return Eigen::Quaternion<Type>(q.w, q.x, q.y, q.z);

}



template<class Type>
Eigen::Matrix<Type, 4, 4> tf_to_EigenMat(const geometry_msgs::Transform& tf_trans){

	Eigen::Quaternion<Type> quat = quat_rosToEigen<Type>(tf_trans.rotation);

    auto tMat = Eigen::Transform<Type, 3, Eigen::Affine>(Eigen::Translation<Type,3>
                                                    (tf_trans.translation.x,
                                                     tf_trans.translation.y,
                                                     tf_trans.translation.z)).matrix();

    auto rMat = Eigen::Transform<Type, 3, Eigen::Affine>(quat.matrix());

    return (tMat*rMat).matrix();

}



template<class Type>
geometry_msgs::Transform EigenMat_to_msg(const Eigen::Matrix<Type, 4, 4>& mat){

    tf2::Matrix3x3  rotation_mat(mat(0, 0), mat(0, 1), mat(0, 2), 
                                 mat(1, 0), mat(1, 1), mat(1, 2),
                                 mat(2, 0), mat(2, 1), mat(2, 2));

    tf2::Vector3 translation_mat(mat(0, 3), mat(1, 3), mat(2, 3));

    return tf2::toMsg(tf2::Transform(rotation_mat, translation_mat));

}



template<class Type>
geometry_msgs::TransformStamped EigenMat_to_tf(const std::string& target_frame, 
                                               const std::string& source_frame, 
                                               const Eigen::Matrix<Type, 4, 4>& mat,
                                               ros::Time t_stamp = ros::Time::now()){

    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = t_stamp;
    transform_stamped.header.frame_id = target_frame;
    transform_stamped.child_frame_id = source_frame;
    transform_stamped.transform = EigenMat_to_msg(mat);

    return transform_stamped;

}




template<class PointT>
void exclud_points(pcl::PointCloud<PointT> &from, 
                   const pcl::PointCloud<PointT> &wrt, 
                   const double DIST){

    ROS_INFO_STREAM("excluding far points from <" << from.header.frame_id << "> w.r.t <" << wrt.header.frame_id << ">");

    typename pcl::PointCloud<PointT>::Ptr cloud_target(new pcl::PointCloud<PointT>(wrt));
	typename pcl::PointCloud<PointT>::Ptr cloud_source(new pcl::PointCloud<PointT>(from));
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(cloud_target);

    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    boost::progress_display _progress( cloud_source->points.size() );

    for (size_t i = 0; i < cloud_source->points.size(); ++i){

        tree.nearestKSearch(cloud_source->points[i], 1, nn_indices, nn_dists);

        if(nn_dists[0]<=DIST){

            cloud_filtered->points.push_back(cloud_source->points[i]);

        }

        ++_progress;

    }

    if(cloud_filtered->width != cloud_filtered->points.size() || cloud_filtered->height != 1){

        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;

    }

    from.points = cloud_filtered->points;
    from.width = cloud_filtered->width;
    from.height = cloud_filtered->height;

}



template <class PointT> double
f_Score(typename pcl::PointCloud<PointT>::Ptr target_cloud, 
        typename pcl::PointCloud<PointT>::Ptr source_cloud, 
        double max_dist = std::numeric_limits<double>::max())
{

    double f_score{0.0};

    std::vector<int> _index(1);
    std::vector<float> _dist (1);

    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(target_cloud);

    uint32_t N{0};
    for (size_t i = 0; i < source_cloud->points.size (); ++i){

        tree.nearestKSearch (source_cloud->points[i], 1, _index, _dist);

        if (_dist[0] <= max_dist){

            f_score += _dist[0];
            ++N;

        }

    }

    if (N>0)
        return (f_score/N);
    else
        return (std::numeric_limits<double>::max());
    
}



std::string d_t_id(){

    std::chrono::system_clock::time_point t_epoch = std::chrono::system_clock::now();

    std::time_t curr_t = std::chrono::system_clock::to_time_t(t_epoch);

    std::stringstream str_s;
    str_s << std::put_time(std::localtime(&curr_t), "%Y-%m-%d_%X");

    return str_s.str();

}



template<class Type> 
void print4x4Mat (const Eigen::Matrix<Type, 4, 4>& mat){

    std::cout << "\tRotation matrix :\n" << std::fixed
            << "\t\t    | "<< std::setw(10) << mat(0, 0) << " " << std::setw(10) << mat(0, 1) << " " << std::setw(10) <<  mat(0, 2) << " | \n"
            << "\t\tR = | "<< std::setw(10) << mat(1, 0) << " " << std::setw(10) << mat(1, 1) << " " << std::setw(10) <<  mat(1, 2) << " | \n"
            << "\t\t    | "<< std::setw(10) << mat(2, 0) << " " << std::setw(10) << mat(2, 1) << " " << std::setw(10) <<  mat(2, 2) << " | \n"
            << "\tTranslation vector :\n"
            << "\t\tt = < "<< std::setw(10) << mat(0, 3) << ", " << std::setw(10) << mat(1, 3) << ", " << std::setw(10) <<  mat(2, 3) << " >\n" << std::flush;

}



using namespace std;
template<class PType, class synchronization_policy>
struct basic_aligner{

    typedef pcl::visualization::PointCloudColorHandlerCustom<PType> p_color;

    basic_aligner(const std::string& target_topic,
            const std::string& source_topic, 
            double max_err, double max_dist, 
            uint32_t n_iterations): 
                source_topic{source_topic}, 
                target_topic{target_topic}, 
                estimated_initTransform_err{max_err}, 
                max_correspondences_dist{max_dist}, num_iterations{n_iterations} {


        viewer_1.setWindowName("pc alignment demo - full clouds");
        viewer_2.setWindowName("pc alignment demo - intersection fields");

        viewer_1.setSize(1280, 1024);
        viewer_2.setSize(1280, 1024);

        viewer_1.createViewPort(0.0, 0.0, 0.5, 1.0, v_port_1);
        viewer_1.createViewPort(0.5, 0.0, 1.0, 1.0, v_port_2);

        viewer_2.createViewPort(0.0, 0.0, 0.5, 1.0, v_port_1);
        viewer_2.createViewPort(0.5, 0.0, 1.0, 1.0, v_port_2);

        viewer_1.setBackgroundColor(0.0, 0.0, 0.0, v_port_1);
        viewer_1.setBackgroundColor(0.0, 0.0, 0.0, v_port_2);
        viewer_2.setBackgroundColor(0.0, 0.0, 0.0, v_port_1);
        viewer_2.setBackgroundColor(0.0, 0.0, 0.0, v_port_2);

        viewer_1.addText("White: Target cloud\nGreen: Transformed source cloud with initial transformation", 
                         10, 15, 16,   1.0, 1.0, 1.0, "icp_info_1", v_port_1);
        viewer_1.addText("White: Target cloud\nRed: Aligned source cloud",
                         10, 15, 16,   1.0, 1.0, 1.0, "icp_info_2", v_port_2);
                         
        
        viewer_2.addText("White: Target cloud\nGreen: Transformed source cloud with initial transformation",
                         10, 15, 16,   1.0, 1.0, 1.0, "icp_info_1", v_port_1);
        viewer_2.addText("White: Target cloud\nRed: Aligned source cloud",
                         10, 15, 16,   1.0, 1.0, 1.0, "icp_info_2", v_port_2);
        

        msg_txt.str("");
        msg_txt << 0;
        std::string iterations_count = "Iterations = " + msg_txt.str();
        viewer_1.addText(iterations_count,
                         10, 60, 16,
                         1.0, 1.0, 1.0, 
                         "iterations_count", v_port_2);
        
        viewer_2.addText(iterations_count, 
                         10, 60, 16,
                         1.0, 1.0, 1.0, 
                         "iterations_count", v_port_2);


        viewer_1.registerKeyboardCallback(&basic_aligner<PType, synchronization_policy>::k_events, 
                                          *this, (void *)NULL);
        viewer_2.registerKeyboardCallback(&basic_aligner<PType, synchronization_policy>::k_events,
                                          *this, (void *)NULL);

        viewer_1.addCoordinateSystem(1.5, "coordinate_v1", v_port_1);
        viewer_1.addCoordinateSystem(1.5, "coordinate_v1", v_port_2);
        viewer_2.addCoordinateSystem(1.5, "coordinate_v1", v_port_1);
        viewer_2.addCoordinateSystem(1.5, "coordinate_v1", v_port_2);

        viewer_1.setCameraPosition(0.0, 0.0, 100.0, 1.0, 0.0, 0.0, 0);
        viewer_2.setCameraPosition(0.0, 0.0, 100.0, 1.0, 0.0, 0.0, 0);


        pc_pub_target = boost::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>
                        (node_handle, target_topic, 1);
        
        pc_pub_source = boost::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>
                        (node_handle, source_topic, 1);
        
        msgs_synchronizer = boost::make_shared<message_filters::Synchronizer<synchronization_policy>>
                                           (synchronization_policy(1), *pc_pub_target, *pc_pub_source);
        
        msgs_synchronizer->registerCallback(boost::bind(&basic_aligner<PType, synchronization_policy>::callback, this, _1, _2));
        ROS_INFO_STREAM("Waiting for two synchronized messages...");

    }





    virtual ~basic_aligner() = default;

    void k_events(const pcl::visualization::KeyboardEvent &event,
                void *nothing)
    {

        if (event.getKeySym() == "space" && event.keyDown()){

            align();

        }
        else if (event.getKeySym() == "n" && event.keyDown()){

            next_batch = true;

        }
        else if (event.getKeySym() == "s" && event.keyDown()){

            std::regex regex("\\/");

            std::vector<std::string> target_topics_tree(
					std::sregex_token_iterator(target_topic.begin(), 
                    target_topic.end(), regex, -1),
					std::sregex_token_iterator()
					);
            
            std::vector<std::string> source_topics_tree(
					std::sregex_token_iterator(source_topic.begin(), 
                    source_topic.end(), regex, -1),
					std::sregex_token_iterator()
					);
            
            const std::string f_out{package_path+"/transforms/"
                              +source_topics_tree.back()
                              +"-to-"
                              +target_topics_tree.back()
                              +"_"+d_t_id()};
            
            std::ofstream f_stream( f_out, std::ios::trunc);

            if(!f_stream.is_open()){

                ROS_ERROR_STREAM("file could not be opened/created <" << f_out << ">!" );
                return;

            }

            geometry_msgs::Transform _transform{EigenMat_to_msg(alignment_transform)};

            f_stream<< "target frame: " << target_frame << '\n'
                    << "source frame: " << source_frame << '\n'
                    << "transformation:\n"
                    << "\trotaion:\n"
                    << std::fixed << std::setprecision(std::numeric_limits<double>::digits10)
                    << "\t\tx: " << _transform.rotation.x <<'\n'
                    << "\t\ty: " << _transform.rotation.y <<'\n'
                    << "\t\tz: " << _transform.rotation.z <<'\n'
                    << "\t\tw: " << _transform.rotation.w <<'\n'
                    << "\ttranslation:\n"
                    << "\t\tx: " << _transform.translation.x <<'\n'
                    << "\t\ty: " << _transform.translation.y <<'\n'
                    << "\t\tz: " << _transform.translation.z;
            
            f_stream.close();
            ROS_INFO_STREAM("transformation has been written into: " << f_out);

        }
        else if(event.getKeySym() == "d" && event.keyDown()){

            max_correspondences_dist-=(max_correspondences_dist/3.0);
            ROS_INFO_STREAM("max correspondences distance has been set to: " << max_correspondences_dist);

        }
        else if(event.getKeySym() == "i" && event.keyDown()){

            max_correspondences_dist+=(max_correspondences_dist/3.0);
            ROS_INFO_STREAM("max correspondences distance has been set to: " << max_correspondences_dist);

        }        

    }



    void callback(sensor_msgs::PointCloud2ConstPtr pc_msg_target, 
                  sensor_msgs::PointCloud2ConstPtr pc_msg_source)
    {

        ROS_INFO_STREAM("Two messages received!");
        target_cloud->clear();
        source_cloud->clear();
        transformed_source_cloud->clear();
        aligned_source_cloud->clear();
        interations_counter=0;
        convert_clouds<PType>(pc_msg_target, pc_msg_source);

        target_frame = target_cloud->header.frame_id;
        source_frame = source_cloud->header.frame_id;

        ROS_INFO_STREAM("target frame: " << target_cloud->header.frame_id);
        ROS_INFO_STREAM("source frame: " << source_cloud->header.frame_id);

        try{

            t_stamp = tf_buffer.lookupTransform(pc_msg_target->header.frame_id, 
                                                pc_msg_source->header.frame_id, 
                                                pc_msg_source->header.stamp);

        }catch (tf2::TransformException &e){

            ROS_WARN_STREAM(e.what());
            return; 

        }

        init_transform = alignment_transform = tf_to_EigenMat<double>(t_stamp.transform);
        pcl::transformPointCloud(*source_cloud, *transformed_source_cloud, init_transform);

        *aligned_source_cloud = * transformed_source_cloud;

        *target_cloud_inter = *target_cloud;
        // *source_cloud_inter = *source_cloud; 
        *transformed_source_cloud_inter = *transformed_source_cloud;

        exclud_points(*target_cloud_inter, 
                      *transformed_source_cloud, 
                      estimated_initTransform_err);
        

        exclud_points(*transformed_source_cloud_inter, 
                      *target_cloud, 
                      estimated_initTransform_err);
        
        // *transformed_source_cloud_inter = *source_cloud_inter;
        *aligned_source_cloud_inter = *transformed_source_cloud_inter;


        target_cloud_color = boost::make_shared<p_color>(target_cloud, 255, 255, 255);
        transformed_source_cloud_color = boost::make_shared<p_color>(transformed_source_cloud, 0, 255, 0);
        aligned_source_cloud_color = boost::make_shared<p_color>(aligned_source_cloud, 255, 0, 0);

        target_cloud_inter_color = boost::make_shared<p_color>(target_cloud_inter, 255, 255, 255);
        transformed_source_cloud_inter_color = boost::make_shared<p_color>(transformed_source_cloud_inter, 0.0, 255, 0);
        aligned_source_cloud_inter_color = boost::make_shared<p_color>(aligned_source_cloud_inter, 255, 0, 0);

        viewer_1.removeAllPointClouds(v_port_1);
        viewer_1.removeAllPointClouds(v_port_2);
        viewer_2.removeAllPointClouds(v_port_1);
        viewer_2.removeAllPointClouds(v_port_2);

        viewer_1.addPointCloud(target_cloud, 
                               *target_cloud_color, 
                               "target_cloud_v1", v_port_1);
        viewer_1.addPointCloud(transformed_source_cloud, 
                               *transformed_source_cloud_color, 
                               "transformed_source_cloud_v1", v_port_1);

        viewer_1.addPointCloud(target_cloud, 
                               *target_cloud_color, 
                               "target_cloud_v2", v_port_2);
        viewer_1.addPointCloud(aligned_source_cloud, 
                               *aligned_source_cloud_color, 
                               "aligned_source_cloud_v2", v_port_2);

        
        viewer_2.addPointCloud(target_cloud_inter, 
                               *target_cloud_inter_color, 
                               "target_cloud_inter_v1", v_port_1);
        viewer_2.addPointCloud(transformed_source_cloud_inter, 
                               *transformed_source_cloud_inter_color, 
                               "transformed_source_cloud_inter_v1", v_port_1);

        viewer_2.addPointCloud(target_cloud_inter, 
                               *target_cloud_inter_color, 
                               "target_cloud_inter_v2", v_port_2);
        viewer_2.addPointCloud(aligned_source_cloud_inter, 
                               *aligned_source_cloud_inter_color, 
                               "aligned_source_cloud_inter_v2", v_port_2);


        msg_txt.str("");
        msg_txt << interations_counter;

        std::string iterations_count = "Iterations = " + msg_txt.str();
        viewer_1.updateText(iterations_count, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_count");
        viewer_2.updateText(iterations_count, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_count");

        init_minimizer();

        viewer_1.spinOnce();
        viewer_2.spinOnce();
        next_batch = 0;

    }


    virtual void align(void) = 0;

    virtual void init_minimizer(void) = 0;

    template<class point_type>
    typename enable_if<is_same<point_type, PN_type>::value, void>::type 
    convert_clouds(sensor_msgs::PointCloud2ConstPtr pc_msg_target, 
                   sensor_msgs::PointCloud2ConstPtr pc_msg_source){
        
        pcl::PointCloud<P_type>::Ptr target_points(new pcl::PointCloud<P_type>);
        pcl::PointCloud<P_type>::Ptr source_points(new pcl::PointCloud<P_type>);

        pcl::fromROSMsg(*pc_msg_target, *target_points);
        pcl::fromROSMsg(*pc_msg_source, *source_points);

        pcl::NormalEstimation<P_type, N_type> n_estimator_t, n_estimator_s;

        n_estimator_t.setInputCloud(target_points);
        pcl::search::KdTree<P_type>::Ptr target_tree{new pcl::search::KdTree<P_type>()}, 
                                         source_tree{new pcl::search::KdTree<P_type>()};

        n_estimator_t.setSearchMethod(target_tree);
        pcl::PointCloud<N_type>::Ptr target_normal{new pcl::PointCloud<N_type>},
                                     source_normal{new pcl::PointCloud<N_type>};

        n_estimator_t.setRadiusSearch(0.3);
        n_estimator_t.compute(*target_normal);

        if(target_points->size() != target_normal->size())
            ROS_INFO_STREAM("Failed to compute normals for all target points");

        n_estimator_s.setInputCloud(source_points);
        n_estimator_s.setSearchMethod(source_tree);

        n_estimator_s.setRadiusSearch(0.3);
        n_estimator_s.compute(*source_normal);

        if(source_points->size() != source_normal->size())
            ROS_INFO_STREAM("Failed to compute normals for all source points");

        pcl::concatenateFields(*target_points, *target_normal, *target_cloud);
        pcl::concatenateFields(*source_points, *source_normal, *source_cloud);

    }


    template<class point_type>
    typename enable_if<!is_same<point_type, PN_type>::value, void>::type 
    convert_clouds(sensor_msgs::PointCloud2ConstPtr pc_msg_target, 
                   sensor_msgs::PointCloud2ConstPtr pc_msg_source){
        
        pcl::fromROSMsg(*pc_msg_target, *target_cloud);
        pcl::fromROSMsg(*pc_msg_source, *source_cloud);

    }


    bool is_terminated() const{

        return (viewer_1.wasStopped() || viewer_2.wasStopped());

    }

    bool update_inputs(){

        return next_batch;

    }

    void close_viewers(){
        viewer_1.close();
        viewer_2.close();
    }

    bool ok(){
        return node_handle.ok();
    }

    void update_viewers(){

        viewer_1.spinOnce();
        viewer_2.spinOnce();

    }



protected:
    pclVis viewer_1, viewer_2;
    Eigen::Matrix4d init_transform{Eigen::Matrix4d::Identity()}, 
                    alignment_transform{Eigen::Matrix4d::Identity()};

    geometry_msgs::TransformStamped t_stamp;

    class pcl::PointCloud<PType>::Ptr target_cloud{new pcl::PointCloud<PType>}, 
                            source_cloud{new pcl::PointCloud<PType>}, 
                            transformed_source_cloud{new pcl::PointCloud<PType>}, 
                            aligned_source_cloud{new pcl::PointCloud<PType>};

    class pcl::PointCloud<PType>::Ptr target_cloud_inter{new pcl::PointCloud<PType>}, 
                           source_cloud_inter{new pcl::PointCloud<PType>}, 
                           transformed_source_cloud_inter{new pcl::PointCloud<PType>}, 
                           aligned_source_cloud_inter{new pcl::PointCloud<PType>};

    tf2_ros::Buffer tf_buffer;

    tf2_ros::TransformListener tf_listener{tf_buffer};

    ros::NodeHandle node_handle;
    

    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_pub_target, pc_pub_source;

    std::string target_topic, target_frame, 
                source_topic, source_frame; 

    boost::shared_ptr<message_filters::Synchronizer<synchronization_policy>>  msgs_synchronizer;

    double estimated_initTransform_err{std::numeric_limits<double>::max()},
           max_correspondences_dist{std::numeric_limits<double>::max()};

    uint32_t num_iterations{1}, interations_counter{0};

    int v_port_1{0}, v_port_2{1};

    std::stringstream msg_txt;

    bool next_batch{1};

    boost::shared_ptr<p_color>
                    target_cloud_color, 
                    transformed_source_cloud_color,
                    aligned_source_cloud_color,

                    target_cloud_inter_color, 
                    transformed_source_cloud_inter_color,
                    aligned_source_cloud_inter_color;

    const std::string package_path{ros::package::getPath("pc_alignment_tools")};


}; //  struct aligner






template<class PType, class synchronization_policy, class minimizer>
struct correspondences_filtering_pipeline_aligner : basic_aligner<PType, synchronization_policy> 
{

    correspondences_filtering_pipeline_aligner(const std::string& target_topic,
            const std::string& source_topic, 
            double max_err, double max_dist, 
            uint32_t n_iterations) : basic_aligner<PType, synchronization_policy>
                    (target_topic, source_topic, 
                     max_err, max_dist, n_iterations){}


    virtual ~correspondences_filtering_pipeline_aligner()=default;

    virtual void init_minimizer() override{}

    virtual void align() override{

        for(uint32_t i{0}; i<this->num_iterations; ++i){

            pcl::CorrespondencesPtr correspondences{new pcl::Correspondences};

            pcl::registration::CorrespondenceEstimation<PType, PType> correspondences_estimator;

            correspondences_estimator.setInputTarget(this->target_cloud_inter);
            correspondences_estimator.setInputSource(this->aligned_source_cloud_inter);

            correspondences_estimator.determineCorrespondences(*correspondences, this->max_correspondences_dist);

            pcl::registration::CorrespondenceRejectorSampleConsensus<PType> ransac_rejector;

            ransac_rejector.setMaximumIterations(10000);

            ransac_rejector.setInputTarget(this->target_cloud_inter);
            ransac_rejector.setInputSource(this->aligned_source_cloud_inter);

            ransac_rejector.getRemainingCorrespondences(*correspondences, *correspondences);

            pcl::registration::CorrespondenceRejectorOneToOne one_to_one_fillter;

            one_to_one_fillter.getRemainingCorrespondences(*correspondences, *correspondences);



            /*
             * 
             * The following two filter types tend to have negative effects on alignment process when
             * the density of the point cloud is fairly low. And since the size to the point cloud doesn't
             * tell anything about its density, it's up to the user to uncomment them.
             *
             */

            // if(is_same<PType, PN_type>::value){
            //     pcl::registration::CorrespondenceRejectorSurfaceNormal normals_rejector;
            //     normals_rejector.initializeDataContainer<PN_type, PN_type>();
            //     normals_rejector.setInputTarget<PN_type>(this->target_cloud_inter);
            //     normals_rejector.setInputSource<PN_type>(this->aligned_source_cloud_inter);
            //     normals_rejector.setThreshold(45);
            //     normals_rejector.getRemainingCorrespondences(*correspondences, *correspondences);
            // }

            // pcl::registration::CorrespondenceRejectorMedianDistance median_distance_fillter;
            // median_distance_fillter.setInputTarget<PType>(target_cloud_inter);
            // median_distance_fillter.setInputSource<PType>(aligned_source_cloud_inter);
            // median_distance_fillter.getRemainingCorrespondences(*correspondences, *correspondences);


            

            Eigen::Matrix4d estimated_mat{Eigen::Matrix4d::Identity()};

            pc_minimizer.estimateRigidTransformation(*this->aligned_source_cloud_inter, 
                                                     *this->target_cloud_inter, 
                                                     *correspondences, estimated_mat);


            this->alignment_transform = estimated_mat * this->alignment_transform;

            pcl::transformPointCloud(*this->source_cloud, 
                                     *this->aligned_source_cloud, 
                                     this->alignment_transform);

            pcl::transformPointCloud(*this->aligned_source_cloud_inter, 
                                     *this->aligned_source_cloud_inter, 
                                     estimated_mat);

        }

        this->interations_counter+=this->num_iterations;

        std::cout << "\nDistance is " << std::fixed << f_Score<PType>(this->target_cloud_inter, 
                                                            this->aligned_source_cloud_inter, 
                                                            this->max_correspondences_dist) << std::endl;
        
        std::cout<< "Transformation:\n";
        print4x4Mat(this->alignment_transform);

        this->msg_txt.str("");
        this->msg_txt << this->interations_counter;

        std::string iterations_count = "Iterations = " + this->msg_txt.str();

        this->viewer_1.updateText(iterations_count, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_count");
        this->viewer_1.updatePointCloud(this->aligned_source_cloud, 
                                        *this->aligned_source_cloud_color, "aligned_source_cloud_v2");

        this->viewer_2.updateText(iterations_count, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_count");
        this->viewer_2.updatePointCloud(this->aligned_source_cloud_inter, 
                                        *this->aligned_source_cloud_inter_color, "aligned_source_cloud_inter_v2");
        
    }


private:
    minimizer pc_minimizer;



}; // struct correspondences_filtering_pipeline_aligner





template<class PType, class synchronization_policy, class minimizer>
struct pc_aligner : basic_aligner<PType, synchronization_policy> 
{

    pc_aligner(const std::string& target_topic,
            const std::string& source_topic, 
            double max_err, double max_dist, 
            uint32_t n_iterations) : basic_aligner<PType, synchronization_policy>
                    (target_topic, source_topic, 
                     max_err, max_dist, n_iterations){}


    virtual ~pc_aligner()=default;

    virtual void init_minimizer() override{

        pc_minimizer.setMaxCorrespondenceDistance(this->max_correspondences_dist);
        pc_minimizer.setMaximumIterations(this->num_iterations);
        pc_minimizer.setInputTarget(this->target_cloud_inter);
        pc_minimizer.setInputSource(this->aligned_source_cloud_inter);

    }


    virtual void align() override{

        pc_minimizer.setMaxCorrespondenceDistance(this->max_correspondences_dist);

        pc_minimizer.align(*this->aligned_source_cloud_inter);

        if (pc_minimizer.hasConverged()){

            this->interations_counter+=this->num_iterations;

            std::cout << "\nConverged, distance is " 
                      << std::fixed << pc_minimizer.getFitnessScore() 
                      << std::endl;

            this->alignment_transform = (pc_minimizer.getFinalTransformation().template cast<double>())*this->alignment_transform; 

            std::cout<< "Transformation:\n";

            print4x4Mat(this->alignment_transform);

            pcl::transformPointCloud(*this->source_cloud, 
                                     *this->aligned_source_cloud, 
                                     this->alignment_transform);

            this->msg_txt.str("");
            this->msg_txt << this->interations_counter;

            std::string iterations_count = "Iterations = " + this->msg_txt.str();

            this->viewer_1.updateText(iterations_count, 
                                      10, 60, 16, 
                                      1.0, 1.0, 1.0, "iterations_count");

            this->viewer_1.updatePointCloud(this->aligned_source_cloud, 
                                            *this->aligned_source_cloud_color, 
                                            "aligned_source_cloud_v2");

            this->viewer_2.updateText(iterations_count,
                                      10, 60, 16, 
                                      1.0, 1.0, 1.0, "iterations_count");

            this->viewer_2.updatePointCloud(this->aligned_source_cloud_inter,
                                            *this->aligned_source_cloud_inter_color, 
                                            "aligned_source_cloud_inter_v2");

        }else{

            ROS_ERROR_STREAM("\nMinimization failed to converge!.\n");

        }
        

    }

private:
    minimizer pc_minimizer;



}; // struct pc_aligner




} // namespace  pc_alignment_tools






#endif // PC_ALIGNMENT_TOOLS_HPP





