
# PointCloud Alignment/Registration Tools with PCL & ROS
Calibration of LiDAR-Sensors - PointCloud Alignment/Registration tools with PCL &amp; ROS. 
<br/><br/>
The tools are based on different versions of the ICP approach for point cloud alignment.

Each tool expects the outputs of source and the target LiDAR-sensors to be published to two different topics, each with a [sensor_msgs::PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) message.

## Dependencies:
+ [ROS](https://www.ros.org):
    + [ROS sensors messages](http://wiki.ros.org/sensor_msgs)
    + [ROS Message Filters](http://wiki.ros.org/message_filters)
    + [ROS transform library tf2](http://wiki.ros.org/tf2)
+ [PCL](https://pointclouds.org/)
  

## Building the Package:
To build the package, in a terminal navigate to `src` folder in your [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace), assuming your catkin workspace is named `catkin_ws`, then run the following commands:  
+ `:~/catkin_ws/src$ git clone https://github.com/ChevronOne/pc_alignment_tools.git`
+ `:~/catkin_ws/src$ cd ..`
+ `:~/catkin_ws$ catkin_make` or `:~/catkin_ws$ catkin build` depending on which command you've used when you first build your **catkin workspace**

If the building process was successful, six executable tools will be generated, each applies a different method to algin two point clouds, and can be run with command `rosrun pc_alignment_tools <tool_name>` . But the publishers of point clouds of the source and target LiDAR-sensors should be already running.

## Available Tools:
The six available tools ara as follow:
1. `point_to_point_aligner`: A standard ICP alignment approach, that tries to minimize the overall distances between each point in the source point cloud and its corresponding point in the target cloud. For reference see "**Paul J. Besl and Neil D. McKay. “A Method for Registration of 3-D Shapes”. In: IEEE TRANSACTIONS ON PATTERN ANALYSIS AND MACHINE INTELLIGENCE 14.2 (Feb. 1992)**"
2. `nl_point_to_point_aligner`: Same as the previous one, but it uses a non-linear optimizer when minimizing the overall distances. 
3. `point_to_plane_aligner`: This method tries to minimize between each point in the source cloud and the estimated underlying surface of the target cloud. For reference see "**Yang Chen and Gérard G. Medioni. “Object modeling by registration of multiple range images”. In: 1991 IEEE International Conference on Robotics and Automation (Apr. 1991). doi: 10 . 1109 / ROBOT . 1991 .132043.**"
4. `gicp_aligner`: This applies the generalized ICP method, which is a probabilistic framework and a more generalized case of point-to-plane method, where it tries to minimize the overall distances between areas surrounding each point in source cloud and its corresponding point in the target cloud. For reference see "**Dirk Haehnel Aleksandr V. Segal and Sebastian Thrun. “Generalized-ICP”. In: in Proceedings of Robotics: Science and Systems (2009).**"
5. `corres_filter_point_to_point_aligner`: It applies the same method as the first one but it also applies some filtering processes on correspondence pairs before the alignment process, which can eliminate a lot of outliers. However, some of these filters may also have negative effects on the alignment process if the size and density of the point clouds are not decent for that filtering method.
6. `corres_filter_point_to_plane_aligner`: Same method as the second one but also with some correspondence-filtering processes before the alignment process. 


## Parameters:
Each tool takes five parameters to run, and they are as follow:
+ **The source and target cloud:** When running an alignment tool, it will list all current `sensor_msgs::PointCloud2` publishers, and the user is asked to choose the source and target clouds.
+ **Estimated max error of initial transformation:** This is very important to focus the alignment process only on the intersection area of field of views of the source and target LiDARs, therefore a constant should be chosen, where the estimated maximum error of the initial transformation should not be bigger than it. All points in the source and target cloud, that don't have a corresponding point within a distance of that constant in the other cloud, will be excluded before the alignment process.
+ **Max correspondence distance:** This usually should be a bit smaller than the max error.
+ **Number of iterations:** The number of iterations to be applied for each push-button on the keyboard. You can choose this to be 1 to observe the tendency of the alignment process whether it is going in right or wrong direction! 

## UI:
When running an alignment tool, it'll show two visualizers, each with two view ports. One visualizer will be showing the alignment process on the original point clouds, and the second one will show the process being applied on the intersection field only, and that based on the chosen estimated max error at the beginning. One view port will visualize the both point clouds without alignment for reference, the the second one will visualize the the point clouds after each alignment iteration.  
And the user can interact with each visualizer with following mouse and keyboard events:
+ `space`-Key: Each push-button will applies **n**-iterations, a parameter that should be given at the beginning.  
+ `d`-Key: Each push-button will decreases the **max correspondence distance** parameter that was given the beginning by a third "**1/3**".  
+ `i`-Key: Each push-button will increases the **max correspondence distance** parameter that was given the beginning by a third "**1/3**".  
+ `s`-Key: Each push-button will save the currently found final transformation on the disk, in a folder named **transforms** in the same package directory. 
+ `n`-Key: Each push-button will try oue a next batch and discard the current pair of point clouds. This can be used whenever the alignment process goes in a wrong direction and can not be fixed anymore.
+ `+`&`-`Keys: Will increase or decrease the size of points in each view port. The view port should be pointed bey mouse-pointer. 
+ `left-click` On Mouse: Hovering the mouse with `left-button` clicked, will rotate the camera about the both horizontal axis.
+ `ctl`+`left-click` On Mouse: Hovering the mouse with `left-button` clicked and `Ctrl` button is pushed down, will rotate the camera about the vertical axes.
+ `right-click` On Mouse: Hovering the mouse with `right-button` clicked, will move the camera along the vertical axes (Zoom-in & Zoom-out).
+ `middle-click` On Mouse: Hovering the mouse with `middle-button` clicked, will move the camera along the both horizontal axis.

<br/><br/>

**Here is a demo on an instance of [Zaytuna model vehicles](https://github.com/ChevronOne/zaytuna) rendered with its primitives:**  
<p align="center">
    <img width="800" align="center" src="alignment_demo" alt="Calligraphy" />
</p>

As can be seen the alignment process performs pretty well on modeled data, where a perfect matching actually exists. However, this is not always the case when the data are from real LiDAR-sensors, as they are usually very noisy and contain a lot of outliers. Therefore to reduce their effect on the alignment process, the parameters that must be given when running each tool, should be chosen carefully.

<br/><br/>
