#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <string>
#include <vector>

namespace fs = std::filesystem;

long extractNumber(const std::string& filename) {
    std::string::size_type start = filename.find_last_of('_') + 1;
    std::string::size_type end = filename.find_last_of('.');
    std::string numberStr = filename.substr(start, end - start);
    return std::stol(numberStr);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <directory_path>" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];
    std::vector<std::string> pcdFiles;

    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        if (entry.path().extension() == ".pcd") {
            pcdFiles.push_back(entry.path().string());
        }
    }

    std::sort(pcdFiles.begin(), pcdFiles.end(), [](const std::string& a, const std::string& b) {
        return extractNumber(a) < extractNumber(b);
    });

    if (pcdFiles.empty()) {
        std::cerr << "No PCD files found in the directory." << std::endl;
        return -1;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Convex Hull Visualization"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);

    int colorIndex = 0;
    for (const auto& file : pcdFiles) {
        std::stringstream cloud_ss;
        cloud_ss << "cloud_" << colorIndex;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, *cloud) == -1) {
            PCL_ERROR("Couldn't read file %s \n", file.c_str());
            continue;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(10);
        sor.setStddevMulThresh(10.0);
        sor.filter(*cloud_filtered);

        pcl::PointCloud<pcl::PointXYZI>::Ptr hull(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ConvexHull<pcl::PointXYZI> chull;
        chull.setInputCloud(cloud_filtered);
        chull.setDimension(3);
        chull.setComputeAreaVolume(true); // Ensure valid convex hull
        pcl::PolygonMesh hullMesh;
        chull.reconstruct(hullMesh);

        if (hullMesh.polygons.empty()) {
            std::cerr << "Convex hull has no polygons for file: " << file << std::endl;
            continue;
        }

        std::stringstream ss;
        ss << "hull_mesh_" << colorIndex;
        viewer->addPolygonMesh(hullMesh, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, ss.str());
        viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered, cloud_ss.str() + "_intensity");
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_filtered, "intensity");
        viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered, intensity_distribution, cloud_ss.str() + "_color");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_ss.str());

        colorIndex++;
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
