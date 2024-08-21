#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_to_rosbag");

    // Check if the correct number of arguments are passed
    if (argc < 3)
    {
        ROS_ERROR("Usage: pcl_to_rosbag <directory_path> <output_bag_file>");
        return -1;
    }

    std::string directory = argv[1];
    std::string output_bag_file = argv[2];

    // Open the bag file
    rosbag::Bag bag;
    bag.open(output_bag_file, rosbag::bagmode::Write);

    // Iterate through the directory and process each PCD file
    boost::filesystem::directory_iterator end_iter;
    for (boost::filesystem::directory_iterator dir_itr(directory); dir_itr != end_iter; ++dir_itr)
    {
        if (boost::filesystem::is_regular_file(dir_itr->status()))
        {
            std::string file_path = dir_itr->path().string();

            // Load the PCD file
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1)
            {
                ROS_ERROR("Couldn't read file %s", file_path.c_str());
                continue;
            }

            // Convert to ROS message
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);

            // Extract timestamp from the filename (assuming filename is the timestamp in microseconds)
            std::string filename = dir_itr->path().stem().string();
            ros::Time timestamp;
            try
            {
                uint64_t timestamp_microseconds = std::stoull(filename);
                uint32_t seconds = static_cast<uint32_t>(timestamp_microseconds / 1e6);
                uint32_t nanoseconds = static_cast<uint32_t>((timestamp_microseconds % static_cast<uint64_t>(1e6)) * 1e3);
                timestamp = ros::Time(seconds, nanoseconds);
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("Invalid timestamp in filename %s, skipping this file.", filename.c_str());
                continue;
            }

            // Set the header
            cloud_msg.header.stamp = timestamp;
            cloud_msg.header.frame_id = "map";

            // Write to the bag file
            bag.write("/point_cloud", timestamp, cloud_msg);
        }
    }

    // Close the bag
    bag.close();

    ROS_INFO("Finished writing PCD files to ROS bag.");
    return 0;
}
