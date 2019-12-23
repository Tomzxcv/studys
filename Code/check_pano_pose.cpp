//
// Created by galyean on 19-6-26.
//

#include <fstream>
#include <iostream>
#include <dirent.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
int main(int argc,char*argv[]){

    PointCloudT::Ptr cloud_all(new PointCloudT);
    PointCloudT::Ptr cloud_new(new PointCloudT);
    std::string file_path = "/media/galyean/DATA/test1/cloud_dense.ply";
    pcl::io::loadPLYFile(file_path, *cloud_all);
    std::vector<PointT> points_axis;
    for(size_t i=0;i<50;++i){
        PointT p_x(0,0,255);
        p_x.x = i*0.01;
        PointT p_y(0,255,0);
        p_y.y = i*0.01;
        PointT p_z(255,0,0);
        p_z.z = i*0.01;
        points_axis.push_back(p_x);
        points_axis.push_back(p_y);
        points_axis.push_back(p_z);
    }
    cloud_new->points.reserve(2*cloud_all->points.size());
    std::ifstream f_read_pano("/media/galyean/DATA/test1/image_pose.txt");
    const char*format = "%lf, %lf, %lf, %lf, %lf, %lf, %lf";
    char buffer[1024];
    while(f_read_pano.getline(buffer,1024)){
        double value[7];
        sscanf(buffer,format,&value[0],&value[1],&value[2],&value[3],&value[4],&value[5],&value[6]);
        Eigen::Quaterniond q(value[3],value[0],value[1],value[2]);
        Eigen::Vector3d translation(value[4],value[5],value[6]);
        for(size_t i=0;i<points_axis.size();++i){
            Eigen::Vector3f point_local = points_axis[i].getArray3fMap();
            Eigen::Vector3f point_global = q.cast<float>().toRotationMatrix()*point_local+translation.cast<float>();
            PointT tmp = points_axis[i];
            tmp.x = point_global[0];
            tmp.y = point_global[1];
            tmp.z = point_global[2];
            cloud_new->points.push_back(tmp);
        }
    }

    f_read_pano.close();
    for(auto point:cloud_all->points){
        PointT tmp = point;
        Eigen::Vector3f point_global = point.getArray3fMap();
        Eigen::Vector3f asdasd = point_global;
        tmp.x = asdasd[0];
        tmp.y = asdasd[1];
        tmp.z = asdasd[2];
        cloud_new->points.push_back(tmp);
    }
    pcl::io::savePLYFileBinary("/media/galyean/DATA/test1//cloud_dense_newyaw_axis.ply", *cloud_new);
}
