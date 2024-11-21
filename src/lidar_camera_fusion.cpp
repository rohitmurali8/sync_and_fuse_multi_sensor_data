#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <filesystem>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include "Eigen/Dense"

// Namespace for file system operations
namespace fs = std::filesystem;

/**
 * @brief Custom point cloud structure to store LiDAR data along with additional information
 * 
 * This structure is designed to hold information such as the velocity, intensity, 
 * signal quality, reflectivity, and flags for each LiDAR point. It extends the 
 * standard PCL (Point Cloud Library) point structure by adding these custom fields.
 */
struct PointXYZCustom
{
    PCL_ADD_POINT4D;  // Macro to add x, y, z, and padding (for alignment)
    float velocity;          ///< Velocity of the point (e.g., speed of the LiDAR sensor or point)
    float intensity;         ///< Intensity of the point (e.g., how strongly the sensor detected the point)
    float signal_quality;    ///< Signal quality of the point (used to assess the reliability of the data)
    float reflectivity;      ///< Reflectivity of the point (used to assess surface properties)
    int time_offset_ns;      ///< Time offset (nanoseconds) relative to a reference timestamp
    uint point_flags_lsb;    ///< Least significant bits of point flags (custom flags for point status)
    uint point_flags_msb;    ///< Most significant bits of point flags (custom flags for point status)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Ensure proper memory alignment for Eigen matrix usage
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

/**
 * @brief DepthMapProjection Node for ROS 2
 * 
 * This ROS 2 node processes LiDAR point cloud data and camera images to project
 * the LiDAR points onto the camera's image plane, creating a depth map representation
 * of the LiDAR data visualized as a color-mapped image. It reads LiDAR and camera
 * data from specified directories, applies extrinsic and intrinsic calibration,
 * and performs projection using linear algebra and image manipulation techniques.
 * The resulting images are saved to the specified output directory.
 */
class DepthMapProjection : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for DepthMapProjection Node
         * 
         * This constructor loads the configuration file, parses the required paths 
         * for LiDAR files, camera files, and output directory, and extracts necessary 
         * calibration matrices (extrinsics and intrinsics) from the YAML configuration.
         */
        DepthMapProjection() : Node("depth_map_projection")
        {
            config_file_path = "../config/params.yaml";  ///< Path to the configuration file

            // Attempt to load the YAML configuration file
            try {
                config = YAML::LoadFile(config_file_path);
            } 
            catch (const YAML::BadFile& e) {
                std::cerr << "Error loading file: " << e.what() << std::endl;
            }

            // Extract paths for LiDAR files, camera files, and output directory
            lidar_path = config["lidar_files_path"].as<std::string>();
            cam2_path = config["cam2_files_path"].as<std::string>();
            output_path = config["output_files_path"].as<std::string>();

            // Load extrinsics (LiDAR to camera transformation) from config
            for (size_t i = 0; i < 4; i++) {
                for (size_t j = 0; j < 4; j++) {
                    lidar_cam2_extrinsics(i, j) = config["lidar_cam2_extrinsics"][i][j].as<double>();
                }
            }

            // Load camera intrinsics (camera's internal calibration matrix) from config
            for (size_t i = 0; i < 4; i++) {
                for (size_t j = 0; j < 4; j++) {
                    cam2_intrinsics(i, j) = config["cam2_intrinsics"][i][j].as<double>();
                }
            }

            // Iterate through the LiDAR and camera directories to get file lists
            for (const auto& entry : fs::directory_iterator(lidar_path)) {
                if (entry.is_regular_file()) {
                    lidarList.push_back(entry.path().string()); // Store LiDAR file paths
                }
            }

            for (const auto& entry : fs::directory_iterator(cam2_path)) {
                if (entry.is_regular_file()) {
                    cam2List.push_back(entry.path().string()); // Store camera file paths
                }
            }
        }

        /**
         * @brief Utility function to create a directory if it doesn't exist
         * 
         * This function attempts to create the given directory and returns `true` 
         * if successful, or `false` if it already exists or there is an error.
         * 
         * @param filepath Path to the directory
         * @return true if directory creation is successful or already exists, false otherwise
         */
        static bool createDirectory(std::string filepath){
            return mkdir(filepath.c_str(), 0777) == 0 || errno == EEXIST;
        }

        /**
         * @brief Comparator to sort files by numerical filename order
         * 
         * This function compares two filenames based on their numeric part (assumes 
         * filenames contain numbers like "1005.ply"), ensuring they are sorted in 
         * increasing numerical order.
         * 
         * @param a First file path
         * @param b Second file path
         * @return true if `a` should come before `b` in numerical order, false otherwise
         */
        static bool numericFilenameComparator(const std::string& a, const std::string& b) {
            size_t posA = a.find_last_of('/');
            size_t posB = b.find_last_of('/');
            std::string fileA = a.substr(posA + 1);
            std::string fileB = b.substr(posB + 1);

            size_t dotPosA = fileA.find(".ply");
            size_t dotPosB = fileB.find(".ply");
            
            int numA = std::stoi(fileA.substr(0, dotPosA)); 
            int numB = std::stoi(fileB.substr(0, dotPosB)); 

            return numA < numB; // Compare numerically
        }        

        /**
         * @brief Project LiDAR points onto camera image plane and generate depth map
         * 
         * This function processes LiDAR data and camera images, projecting the LiDAR 
         * points onto the camera's image plane. It computes the depth map and visualizes 
         * the LiDAR points as colored pixels on the image. The result is saved as a color 
         * depth image in the output directory.
         */
        void project_lidar_points(){
            // Sort LiDAR and camera files in numerical order based on filenames
            std::sort(lidarList.begin(), lidarList.end(), numericFilenameComparator);
            std::sort(cam2List.begin(), cam2List.end(), numericFilenameComparator);     

            // Create output directory if it doesn't exist
            createDirectory(output_path);              

            // Iterate through the sorted LiDAR and camera files
            for (size_t i =0; i < lidarList.size(); i++){
                // Load the corresponding camera image
                cv::Mat image = cv::imread(cam2List[i], cv::IMREAD_COLOR);
                int height = image.size[0];
                int width = image.size[1];

                pcl::PointCloud<PointXYZCustom>::Ptr cloud(new pcl::PointCloud<PointXYZCustom>());

                // Load LiDAR point cloud from PLY file
                if (pcl::io::loadPLYFile<PointXYZCustom>(lidarList[i], *cloud) == -1) {
                    PCL_ERROR("Couldn't read the PLY file\n");
                }

                // Convert LiDAR points to a 4xN matrix for projection
                Eigen::MatrixXd lidar_points(4, cloud->points.size());
                for (size_t i = 0; i < cloud->points.size(); ++i) {
                    lidar_points(0, i) = cloud->points[i].x;  
                    lidar_points(1, i) = cloud->points[i].y;  
                    lidar_points(2, i) = cloud->points[i].z;
                    lidar_points(3, i) = 1.0;  // Homogeneous coordinates
                }

                // Perform the projection from LiDAR to camera coordinates
                Eigen::MatrixXd proj_points = cam2_intrinsics * (lidar_cam2_extrinsics * lidar_points);
                Eigen::VectorXd first_row = proj_points.row(0);
                Eigen::VectorXd second_row = proj_points.row(1);
                Eigen::VectorXd third_row = proj_points.row(2);

                // Normalize the points by dividing by the third row (depth)
                first_row = first_row.array() / third_row.array();
                second_row = second_row.array() / third_row.array();

                // Create masks to filter out points that fall outside the image bounds
                Eigen::Array<bool, Eigen::Dynamic, 1> u_out = (first_row.array() < 0) || (first_row.array() > width);
                Eigen::Array<bool, Eigen::Dynamic, 1> v_out = (second_row.array() < 0) || (second_row.array() > height);

                // Combine masks to create an overall outlier mask
                Eigen::Array<bool, Eigen::Dynamic, 1> outlier = u_out || v_out;

                // Filter out the outlier points and store the valid ones
                Eigen::VectorXd u_filtered, v_filtered, z_filtered;
                int num_inliers = (outlier == false).count();
                u_filtered.resize(num_inliers);
                v_filtered.resize(num_inliers);
                z_filtered.resize(num_inliers);

                // Store the valid (inlier) points
                int index = 0;
                for (int i = 0; i < first_row.size(); ++i) {
                    if (!outlier(i)) {
                        u_filtered(index) = first_row(i);
                        v_filtered(index) = second_row(i);
                        z_filtered(index) = third_row(i);
                        index++;
                    }
                }

                // Project LiDAR points onto the image, coloring based on depth
                for (int i = 0; i < u_filtered.size(); ++i) {
                    // Extract coordinates and depth
                    int x = static_cast<int>(u_filtered(i));
                    int y = static_cast<int>(v_filtered(i));

                    // Ensure points are within image bounds
                    if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                        // Color scale based on depth (nearer points are darker)
                        int color_intensity = static_cast<int>(255 * (z_filtered(i) / 100.0));
                        cv::Scalar color(0, color_intensity, 255 - color_intensity); // R-G-B format

                        // Draw a circle for each LiDAR point
                        cv::circle(image, cv::Point(x, y), 1, color, -1);
                    }
                }

                // Generate depth map and apply interpolation for missing data
                cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);  // Mask for interpolation
                cv::Mat d_img = cv::Mat::zeros(height, width, CV_8UC1); // Depth image

                // PART 1: Populate depth map with filtered LiDAR points
                for (size_t idx = 0; idx < u_filtered.size(); ++idx) {
                    int x = u_filtered[idx];
                    int y = v_filtered[idx];
                    int lidar_depth = z_filtered[idx];
                    d_img.at<uchar>(std::min(y, height - 1), std::min(x, width - 1)) = static_cast<uchar>(lidar_depth);
                }

                // PART 2: Threshold and interpolate missing depth values
                int win_dim = 15;  // Window size for interpolation
                int thresh = 1;    // Threshold for interpolation

                // Create a mask for areas that need interpolation
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        if (d_img.at<uchar>(i, j) == 0) {
                            int min_y = std::max(0, i - win_dim);
                            int max_y = std::min(height, i + win_dim);
                            int min_x = std::max(0, j - win_dim);
                            int max_x = std::min(width, j + win_dim);

                            // Count non-zero pixels in the window for interpolation
                            cv::Mat window = d_img(cv::Range(min_y, max_y), cv::Range(min_x, max_x));
                            int non_zero_cnt = cv::countNonZero(window);

                            if (non_zero_cnt >= thresh) {
                                mask.at<uchar>(i, j) = 255; // Mark pixel for interpolation
                            }
                        }
                    }
                }

                // Interpolate missing depth values
                cv::Mat inter_img;
                cv::inpaint(d_img, mask, inter_img, 1.0, cv::INPAINT_TELEA);

                // Apply a color map to the interpolated depth map
                cv::Mat color_mapped_img;
                cv::applyColorMap(inter_img, color_mapped_img, cv::COLORMAP_HSV);

                // Generate the output filename and save the image
                std::string filename = output_path + std::to_string(i) + ".png";
                if (cv::imwrite(filename, color_mapped_img)) {
                    RCLCPP_INFO(this->get_logger(), "Saved image as %s", filename.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to save image as %s", filename.c_str());
                }
            }
        }

    private:

        std::string config_file_path;       ///< Path to configuration file
        std::string lidar_path;             ///< Path to LiDAR files
        std::string cam2_path;               ///< Path to camera files
        std::string output_path;           ///< Path to output directory for generated images
        YAML::Node config;                 ///< Loaded YAML configuration
        std::vector<std::string> lidarList; ///< List of LiDAR file paths
        std::vector<std::string> cam2List;   ///< List of camera image paths
        Eigen::Matrix4d lidar_cam2_extrinsics; ///< LiDAR to camera extrinsic calibration matrix
        Eigen::Matrix4d cam2_intrinsics;      ///< Camera intrinsic calibration matrix
};

/**
 * @brief Main entry point for the DepthMapProjection ROS 2 node
 * 
 * Initializes the ROS 2 system, creates an instance of the DepthMapProjection node,
 * and starts the process of projecting LiDAR points onto the camera image plane.
 * The program then enters the ROS 2 event loop to handle communication.
 */
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);  // Initialize the ROS system
    auto node = std::make_shared<DepthMapProjection>();  // Create node instance
    node->project_lidar_points();  // Perform LiDAR point projection

    rclcpp::spin(node);  // Spin the node to handle ROS 2 events
    rclcpp::shutdown();  // Shut down ROS 2
    return 0;
}
