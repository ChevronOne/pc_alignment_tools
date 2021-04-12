

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





#include "pc_alignment_tools.hpp"

extern void get_inputs(std::string&, std::string&, uint32_t&, double&, double&);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "corres_filter_point_to_plane_aligner");     
    while(!ros::master::check()){
        std::this_thread::sleep_for(2s);
        ROS_ERROR_STREAM("connection to master at ["
                         << ros::master::getHost()<<":"<<ros::master::getPort() 
                         << "] yet to be established..");
    }
    std::string target_pub{""}, source_pub{""};
    uint32_t num_iterations{1};
    double estimated_err{std::numeric_limits<double>::max()},
           max_correspondence_dist{std::numeric_limits<double>::max()};

    get_inputs(target_pub, source_pub, 
               num_iterations, estimated_err, 
               max_correspondence_dist);

    boost::shared_ptr<pc_alignment_tools::correspondences_filtering_pipeline_aligner<pc_alignment_tools::PN_type, 
                      pc_alignment_tools::ApproximatePolicy, 
                      pc_alignment_tools::corres_filter_point_to_plane_aligner>> 
            pc_aligner(new pc_alignment_tools::correspondences_filtering_pipeline_aligner<pc_alignment_tools::PN_type, 
                                pc_alignment_tools::ApproximatePolicy, 
                                pc_alignment_tools::corres_filter_point_to_plane_aligner>
                (target_pub, source_pub, 
                 estimated_err, 
                 max_correspondence_dist,
                 num_iterations));


    while(ros::ok() && !pc_aligner->is_terminated()){
        while(pc_aligner->update_inputs())
            ros::spinOnce();
        pc_aligner->update_viewers();
        std::this_thread::sleep_for(0.1s);
    }

    exit(EXIT_SUCCESS);
    
    
}




