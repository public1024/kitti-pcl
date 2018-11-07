#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>

#include "ros/ros.h"
using namespace pcl;
using namespace std;

namespace po = boost::program_options;

static ros::Publisher g_cloud_pub;
static std::vector<std::string> file_lists;

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

void readKittiPclBinData(std::string &in_file, std::string& out_file)
{
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();
//    g_cloud_pub.publish( points );

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.writeBinary< pcl::PointXYZI > (out_file, *points);
}


int main(int argc, char **argv)
{
//    ros::init(argc, argv, "ground_remove_test");
//    ros::NodeHandle n;
//    g_cloud_pub = n.advertise< pcl::PointCloud< pcl::PointXYZI > > ("point_chatter", 1);

    // std::string bin_path = "../velodyne/binary/";
    // std::string pcd_path = "../velodyne/pcd/";
    std::string bin_path = argv[1];
    std::string pcd_path = argv[2];
    std::cout<<"bin path:\t"<<bin_path<<std::endl;
    std::cout<<"pcd path:\t"<<pcd_path<<std::endl;
    read_filelists( bin_path, file_lists, "bin" );
    sort_filelists( file_lists, "bin" );
    for (int i = 0; i < file_lists.size(); ++i)
    {
        std::cout<<file_lists[i]<<std::endl;
        std::string bin_file = bin_path + file_lists[i];
        std::string tmp_str = file_lists[i].substr(0, file_lists[i].length() - 4) + ".pcd";
        std::string pcd_file = pcd_path + tmp_str;
        readKittiPclBinData( bin_file, pcd_file );
    }

    return 0;
}

// int main(int argc, char **argv){
	// ///The file to read from.
	// string infile;

	// ///The file to output to.
	// string outfile;

	// // Declare the supported options.
	// po::options_description desc("Program options");
	// desc.add_options()
		// //Options
		// ("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
		// ("outfile", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud & normals to")
		// ;
	// // Parse the command line
	// po::variables_map vm;
	// po::store(po::parse_command_line(argc, argv, desc), vm);

	// // Print help
	// if (vm.count("help"))
	// {
		// cout << desc << "\n";
		// return false;
	// }

	// // Process options.
	// po::notify(vm);

	// // load point cloud
	// fstream input(infile.c_str(), ios::in | ios::binary);
	// if(!input.good()){
		// cerr << "Could not read file: " << infile << endl;
		// exit(EXIT_FAILURE);
	// }
	// input.seekg(0, ios::beg);

	// pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);

	// int i;
	// for (i=0; input.good() && !input.eof(); i++) {
		// PointXYZI point;
		// input.read((char *) &point.x, 3*sizeof(float));
		// input.read((char *) &point.intensity, sizeof(float));
		// points->push_back(point);
	// }
	// input.close();

	// cout << "Read KTTI point cloud with " << i << " points, writing to " << outfile << endl;

    // pcl::PCDWriter writer;

    // // Save DoN features
    // writer.write<PointXYZI> (outfile, *points, false);
// }
