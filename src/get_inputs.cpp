

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



#include "ros/package.h"
#include "ros/ros.h"
#include "ros/topic.h"
#include <iostream>
#include <string>


void get_inputs(std::string& target_pub, 
                std::string& source_pub,
                uint32_t& num_iterations,
                double& estimated_err,
                double& max_correspondence_dist){


    std::vector<ros::master::TopicInfo> pubishers;
    std::vector<ros::master::TopicInfo> pc_pubishers;
    ros::master::getTopics(pubishers);


    for(const ros::master::TopicInfo& topic : pubishers){
        if(topic.datatype == "sensor_msgs/PointCloud2")
            pc_pubishers.emplace_back(topic);
    }



    if(pc_pubishers.size() == 0){
        ROS_WARN_STREAM("No \"sensor_msgs/PointCloud2\" publishers found!" );
        exit(EXIT_SUCCESS);
    }


    std::sort(pc_pubishers.begin(), pc_pubishers.end(), 
                [](const ros::master::TopicInfo& a, const ros::master::TopicInfo& b)
                { return a.name < b.name; });

    uint32_t i{0}, source_ind{0}, target_ind{1};
    ROS_INFO_STREAM("Available \"sensor_msgs/PointCloud2\" publishers:");

    for(i=0;i<pc_pubishers.size(); ++i){
        std::cout << i  << " >> " << pc_pubishers[i].name << "\n";
    }


    std::cout << "\n\n";
    std::cout << "source frame: ";
    std::cin >> source_ind;

    if(source_ind<0 || source_ind>=pc_pubishers.size()){
        ROS_ERROR_STREAM("invalid index: " << source_ind);
        exit(EXIT_FAILURE);
    }

    source_pub = pc_pubishers[source_ind].name;



    std::cout << "\n\n";
    std::cout << "target frame: ";
    std::cin >> target_ind;

    if(target_ind<0 || target_ind>=pc_pubishers.size() || target_ind==source_ind){
        ROS_ERROR_STREAM("invalid index: " << target_ind);
        exit(EXIT_FAILURE);
    }

    target_pub = pc_pubishers[target_ind].name;



    std::cout << "\n\n";
    std::cout << "estimated max error of initial transformation: ";
    std::cin >> estimated_err;
    if(estimated_err<=0 ){
        ROS_ERROR_STREAM("invalid estimated error value: " << estimated_err << ", should be bigger than 0.0!");
        exit(EXIT_FAILURE);
    }

    std::cout << "\n\n";
    std::cout << "max correspondence distance: ";
    std::cin >> max_correspondence_dist;

    if(max_correspondence_dist<=0){
        ROS_ERROR_STREAM("invalid max distance value: " << max_correspondence_dist << ", should be bigger than 0.0!");
        exit(EXIT_FAILURE);
    }



    std::cout << "\n\n";
    std::cout << "number of iterations: ";
    std::cin >> num_iterations;

    if(num_iterations<0){
        ROS_ERROR_STREAM("invalid iterations value: " << num_iterations << ", should be non-negative!");
        exit(EXIT_FAILURE);
    }




}







