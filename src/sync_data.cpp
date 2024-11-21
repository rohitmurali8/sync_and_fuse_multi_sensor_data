#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>

/*
NOTE - Specify structure for the custom point cloud object (If specifying different custom PCD, please populate fields accordingly)
*/

// Define a custom point cloud structure
struct PointXYZCustom
{
    PCL_ADD_POINT4D;  // Macro to add x, y, z, and padding (for alignment)
    float velocity;
    float intensity;
    float signal_quality;
    float reflectivity;
    int time_offset_ns;
    uint point_flags_lsb;
    uint point_flags_msb;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Ensure proper alignment
};

// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZCustom,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, velocity, velocity)
                                  (float, intensity, intensity)
                                  (float, signal_quality, signal_quality)
                                  (float, reflectivity, reflectivity)
                                  (int, time_offset_ns, time_offset_ns)
                                  (uint, point_flags_lsb, point_flags_lsb)
                                  (uint, point_flags_msb, point_flags_msb))

using namespace std::chrono_literals;

/**
 * @brief The DataSynchronizer class is responsible for synchronizing and saving data
 *        from multiple ROS topics (camera images, LiDAR point clouds, and IMU data).
 *        It saves the data into structured directories, making it easy for further processing.
 */
class DataSynchronizer : public rclcpp::Node
{
    public:
        // Constructor to initialize node, subscribers, and synchronization policies
        DataSynchronizer() : Node("data_synchronizer")
        {
            // Specify the path to read the YAML file that contains configuration details
            config_file_path = "../config/sync_data.yaml";

            // Load the specified YAML file in a TRY/CATCH condition 
            try
            {
                config = YAML::LoadFile(config_file_path);
            }
            catch(const YAML::BadFile& e)
            {
                std::cerr << "Error loading the specified YAML file" << e.what() << '\n';
            }
            
            // Subscribe to the topics specified in the YAML file for Lidar, Cameras, and IMU
            cam1_sub_.subscribe(this, config["cam1_topic"].as<std::string>());
            cam2_sub_.subscribe(this, config["cam2_topic"].as<std::string>());
            lidar_sub_.subscribe(this, config["lidar_topic"].as<std::string>());
            imu_sub_.subscribe(this, config["imu_topic"].as<std::string>());        

            // Initialize the synchronization policy with a queue size and time slop (tolerance)
            sync_.reset(new Sync(MySyncPolicy(50), cam1_sub_, cam2_sub_, imu_sub_, lidar_sub_));
            sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.2));  // 200 ms tolerance

            // Initialize message counter for synchronized data
            count_ = 0;

            // Create the root folder directory to save data extracted from the published ROS2 topics
            folder_path = config["folder_path"].as<std::string>();
            if (!createDirectory(folder_path)) {
                std::cerr << "Failed to create directory: " << folder_path << std::endl;
            }        

            // Create directories for CAM1, CAM2, LiDAR, and IMU data
            cam1_path = folder_path + "cam1/";
            if (!createDirectory(cam1_path)) {
                std::cerr << "Failed to create directory: " << cam1_path << std::endl;
            }        

            cam2_path = folder_path + "cam2/";
            if (!createDirectory(cam2_path)) {
                std::cerr << "Failed to create directory: " << cam2_path << std::endl;
            }                

            lidar_path = folder_path + "lidar/";
            if (!createDirectory(lidar_path)) {
                std::cerr << "Failed to create directory: " << lidar_path << std::endl;
            }

            imu_path = folder_path + "imu/";
            if (!createDirectory(imu_path)) {
                std::cerr << "Failed to create directory: " << imu_path << std::endl;
            }
            
            // Register the callback function that will be called when the data is synchronized
            sync_->registerCallback(std::bind(&DataSynchronizer::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        }

    private:
        
        // Helper function to create specified directory
        bool createDirectory(const std::string& directory) {
            // Creates the directory and returns true if successful or directory already exists
            return mkdir(directory.c_str(), 0777) == 0 || errno == EEXIST;
        }

        /**
         * @brief Synchronized callback function responsible for extracting and saving data
         *        from the synchronized ROS2 topics (images, point cloud, and IMU data).
         *        The data is saved to appropriate files in the designated directories.
         */
        void synchronized_callback(const sensor_msgs::msg::Image::ConstSharedPtr &cam1_msg,
                                    const sensor_msgs::msg::Image::ConstSharedPtr &cam2_msg,
                                    const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
                                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_msg)
        {
            RCLCPP_INFO(this->get_logger(), "Synchronized message count %d", count_);

            // Initialize the custom point cloud object 
            pcl::PointCloud<PointXYZCustom>::Ptr cloud(new pcl::PointCloud<PointXYZCustom>);

            // Populate the initialized PointCloud object with data from the ROS2 lidar message 
            cloud->width = lidar_msg->width;
            cloud->height = lidar_msg->height;
            cloud->is_dense = lidar_msg->is_dense;
            cloud->points.resize(cloud->width * cloud->height);     

            // Iterate through the points in the ROS message and copy them to the Point Cloud object 
            for (size_t i = 0; i < cloud->points.size(); ++i)
            {   
                // Calculate the offset for each point based on the POINT STEP SIZE
                const size_t offset = i * lidar_msg->point_step;

                // Copy individual fields from the ROS point cloud stream to the initialized point cloud object
                std::memcpy(&cloud->points[i].x, &lidar_msg->data[offset + lidar_msg->fields[0].offset], sizeof(float));
                std::memcpy(&cloud->points[i].y, &lidar_msg->data[offset + lidar_msg->fields[1].offset], sizeof(float));
                std::memcpy(&cloud->points[i].z, &lidar_msg->data[offset + lidar_msg->fields[2].offset], sizeof(float));

                // Copy additional fields such as Velocity, Intensity, Reflectivity, etc.
                std::memcpy(&cloud->points[i].velocity, &lidar_msg->data[offset + lidar_msg->fields[3].offset], sizeof(float));
                std::memcpy(&cloud->points[i].intensity, &lidar_msg->data[offset + lidar_msg->fields[4].offset], sizeof(float));
                std::memcpy(&cloud->points[i].signal_quality, &lidar_msg->data[offset + lidar_msg->fields[5].offset], sizeof(float));
                std::memcpy(&cloud->points[i].reflectivity, &lidar_msg->data[offset + lidar_msg->fields[6].offset], sizeof(float)); 
                std::memcpy(&cloud->points[i].time_offset_ns, &lidar_msg->data[offset + lidar_msg->fields[7].offset], sizeof(int32_t));
                std::memcpy(&cloud->points[i].point_flags_lsb, &lidar_msg->data[offset + lidar_msg->fields[8].offset], sizeof(uint32_t));
                std::memcpy(&cloud->points[i].point_flags_msb, &lidar_msg->data[offset + lidar_msg->fields[9].offset], sizeof(uint32_t));
            }

            // Save the point cloud to a PLY file
            std::string filename = lidar_path + std::to_string(count_) + ".ply";  
            pcl::io::savePLYFileBinary(filename, *cloud);

            // Save the IMU data to a text file
            filename = imu_path + std::to_string(count_) + ".txt";
            imu_file_.open(filename, std::ios::out);
            if (!imu_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
                rclcpp::shutdown();
            }

            // Write IMU data to the text file
            imu_file_ << "Timestamp, Orientation(x,y,z,w), AngularVelocity(x,y,z), LinearAcceleration(x,y,z)" << std::endl;
            imu_file_ << imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9 << ", ";
            imu_file_ << imu_msg->orientation.x << ", " << imu_msg->orientation.y << ", "
                << imu_msg->orientation.z << ", " << imu_msg->orientation.w << ", ";
            imu_file_ << imu_msg->angular_velocity.x << ", " << imu_msg->angular_velocity.y << ", "
                << imu_msg->angular_velocity.z << ", ";
            imu_file_ << imu_msg->linear_acceleration.x << ", " << imu_msg->linear_acceleration.y << ", "
                << imu_msg->linear_acceleration.z << std::endl;

            imu_file_.close();

            // Convert the ROS2 image message from Camera 1 to an OpenCV Mat
            cv::Mat image = cv_bridge::toCvCopy(cam1_msg, cam1_msg->encoding)->image;

            // Save the image as a PNG file
            filename = cam1_path + std::to_string(count_) + ".png";
            if (!cv::imwrite(filename, image)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image as %s", filename.c_str());
            }

            // Convert the ROS2 image message from Camera 2 to an OpenCV Mat
            image = cv_bridge::toCvCopy(cam2_msg, cam2_msg->encoding)->image;

            // Save the image as a PNG file
            filename = cam2_path + std::to_string(count_) + ".png";
            if (!cv::imwrite(filename, image)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image as %s", filename.c_str());
            }        

            // Increment the synchronized message counter
            count_ += 1;
        }

        // Define the synchronization policy for the topics (images, IMU, and LiDAR)
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Imu, sensor_msgs::msg::PointCloud2> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;

        // Define the subscribers for each topic using the message filters namespace
        message_filters::Subscriber<sensor_msgs::msg::Image> cam1_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> cam2_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;

        // Synchronizer pointer to manage the synchronized topics
        std::shared_ptr<Sync> sync_;

        // File paths for saving the data
        std::string cam1_path;
        std::string cam2_path;
        std::string lidar_path;
        std::string imu_path;    
        std::string folder_path;
        std::string config_file_path;  // Path to configuration file

        // Initialize the message counter, IMU file output stream, and YAML node
        int count_;
        YAML::Node config;
        std::ofstream imu_file_;
};

// Main function that initializes ROS2, creates a DataSynchronizer node, and runs it
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataSynchronizer>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
