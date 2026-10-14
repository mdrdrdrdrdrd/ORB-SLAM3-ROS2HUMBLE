#include "PointClouds.h"
#include <pcl/filters/voxel_grid.h>
#include<vector>

namespace ORB_SLAM3 {

PointCloudData::PointCloudData(){
    cloud = std::make_shared<PointCloud>();
}

PointCloudData::~PointCloudData()
{
}

PointClouds::PointClouds()
{

    mbUpdateCloudPoint = false;
}

PointClouds::~PointClouds()
{
}


void PointClouds::insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth, cv::Mat& mK, cv::Mat& mDistCoef)
{
    unique_lock<mutex> locker(mKeyFrameMtx);
    
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    int stride = 6;
    for (int i = 0; i < color.rows; i += stride)
    {
        for (int j = 0; j < color.cols; j+= stride)
        {
            cv::KeyPoint kp;
            kp.pt.x = j;
            kp.pt.y = i;
            mvKeys.push_back(kp);
        }
    }

    int N = mvKeys.size();

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);

    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
    mat=mat.reshape(1);

    int cnt = 0;
    PointCloud::Ptr cloud( new PointCloud() );
    float v_min = 1000000000.0;
    float v_max = 0.0;

    for ( int m=0; m<depth.rows; m+=stride )
    {
        for ( int n=0; n<depth.cols; n+=stride )
        {
            float d = depth.ptr<float>(m)[n];
            v_min = v_min > d ? d : v_min; 
            v_max = v_max > d ? v_max : isnan(d) ? v_max : d;

            if (isnan(d) || d < 0.0 || d>5.0)
            {
                cnt += 1;
                continue;
            }
            
            PointT p;
            p.x = ( mat.at<float>(cnt, 0) - kf->cx) * d / kf->fx;
            p.y = ( mat.at<float>(cnt, 1) - kf->cy) * d / kf->fy;
            p.z = d;

            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back(p);
            cnt += 1;
        }
    }

    //cout << "v_max=" << v_max << ", v_min=" << v_min << endl; 

    cloud->width = cloud->points.size();
    cloud->height = 1;

    PointCloudData pcd;
    pcd.kf = kf;
    pcd.cloud = cloud;
    pcd.T_wc = kf->GetPoseInverse().matrix();  // freeze the pose for this keyframe

    mPointCloudDatas.push_back(pcd);
    mbUpdateCloudPoint = true;

    //cout << "insert cloud" << endl;

}


void PointClouds::generatePointCloud(PointCloud::Ptr& GlobalCloud, Atlas* pAtlas,
                                     Eigen::Matrix4f& Trans_cam2ground,
                                     bool freeze_pose, float voxel_leaf)
{ 

    if (!mbUpdateCloudPoint)
        return; 

    if (!pAtlas)
        return;

    const vector<KeyFrame*> vpKFs = pAtlas->GetAllKeyFrames();
    // Compare by pointer identity only. After a map reset the old keyframes may
    // have been erased, so stored pointers must never be dereferenced before we
    // know they still belong to the current map.
    std::set<KeyFrame*> sKFptr(vpKFs.begin(), vpKFs.end());

    std::unique_lock<std::mutex> locker(mPointCloudMtx);   
    
    std::vector<PointCloudData> alive;
    alive.reserve(mPointCloudDatas.size());

    for (auto & pcd : mPointCloudDatas)
    {
        if (sKFptr.find(pcd.kf) == sKFptr.end())
            continue;  // stale keyframe from a previous map: drop it

        alive.push_back(pcd);

        // Camera->world. Frozen pose keeps already collected points stable;
        // the live pose re-applies BA / loop-closure corrections every time a
        // keyframe is inserted, which looks like the map drifting / re-adding
        // previous points at an offset.
        Eigen::Matrix4f T_wc;
        if (freeze_pose)
            T_wc = pcd.T_wc;
        else
            T_wc = pcd.kf->GetPoseInverse().matrix();

        PointCloud::Ptr cloud(new PointCloud);
        pcl::transformPointCloud(*pcd.cloud, *cloud, T_wc); 

        PointCloud::Ptr cloud1(new PointCloud);
        pcl::transformPointCloud(*cloud, *cloud1, Trans_cam2ground); 

        *GlobalCloud += *cloud1;
    }

    mPointCloudDatas.swap(alive);
    mbUpdateCloudPoint = false;

    // Optional voxel downsampling: merges overlapping copies so the map does
    // not thicken where the same surface is measured from several keyframes.
    if (voxel_leaf > 0.0f && !GlobalCloud->empty())
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(GlobalCloud);
        vg.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
        PointCloud::Ptr filtered(new PointCloud);
        vg.filter(*filtered);
        GlobalCloud.swap(filtered);
    }
}


}
