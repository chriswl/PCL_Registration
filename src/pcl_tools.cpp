#include "pcl_tools.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

//#include <tf/transform_datatypes.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/common.h>

//Projection
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>


//Pairwise
#include <boost/make_shared.hpp>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/gicp.h>

#include "reg_visualizer.h"

using namespace std;

struct timespec t1_, t2_;
double elapsed_time_;
volatile long long i_;

void start_timer(){
    clock_gettime(CLOCK_MONOTONIC,  &t1_);
}

void end_timer(string message = "Elapsed time:"){
    clock_gettime(CLOCK_MONOTONIC,  &t2_);
    elapsed_time_ = (t2_.tv_sec - t1_.tv_sec) + (double) (t2_.tv_nsec - t1_.tv_nsec) * 1e-9;
    std::cout << message << " " << elapsed_time_ << " seconds" << std::endl;
}


int file_exist (const char *filename)
{
    struct stat   buffer;
    return (stat (filename, &buffer) == 0);
}


std::string int2str(int num){
    stringstream strs;
    strs << num;
    return strs.str();
}


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

pcl_tools::pcl_tools(params *config)
{
    config_ = config;
    MAX_NUM_SCANS = 83;
    for(int i=0; i < MAX_NUM_SCANS; i++){
        transformations[i].completed = false;
        transformations[i].is_root = false;
        transformations[i].T.setIdentity();
        transformations[i].T_ndt.setIdentity();
        transformations[i].ok = false;
    }
}

Eigen::Matrix3f pcl_tools::quaternion_to_rotation(float x, float y, float z, float w){
    Eigen::Quaternionf quat(w, x, y, z);
    return quat.toRotationMatrix();
    //std::cout << rotation << std::endl;
    /*Eigen::Matrix4f Trans; // Your Transformation Matrix
    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Trans.block<3,3>(0,0) = rotation;
    Trans.rightCols<1>() = translation;
    std::cout << Trans << std::endl;*/
}

Eigen::Matrix4f pcl_tools::createTransformationMatrix(Eigen::Matrix3f rotation, Eigen::Vector3f translation){
    Eigen::Matrix4f Trans; // Transformation Matrix
    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Trans.block<3,3>(0,0) = rotation;
    Trans.block<3,1>(0,3) = translation;
    return Trans;
}

Eigen::Matrix4f pcl_tools::getInitialGuess(int input, int target){
    string tline;
    fstream fid;
    std::string initial_guess_filename = config_->get_initial_estimate_path();
    if (!file_exist(initial_guess_filename.c_str())) {
        std::cout << "Can not find initial guesses; exiting" << std::endl;
        throw std::invalid_argument( "no initial guess" );
    }

    fid.open(initial_guess_filename.c_str(), ios::in);
    getline(fid, tline);
    stringstream transform_pair;
    transform_pair << "scan_" << input << " to " << "scan_" << target;
    while(tline.find(transform_pair.str())==string::npos && !fid.eof()){
        getline(fid, tline);
    }
    if(fid.eof()){
        std::cout << "Transformation from scan_" << input << " to scan_" << target << " not found" << std::endl;
        fid.close();
        return Eigen::Matrix4f::Zero();
    }
    getline(fid, tline);
    float t1, t2, t3;
    float x, y, z, w;
    if(tline.find("- Translation: [")!=string::npos && !fid.eof()){
        sscanf(tline.c_str(), "- Translation: [%f, %f, %f]", &t1, &t2, &t3);
        getline(fid, tline);
    }
    else{
        cout << "Erronous file format" << endl;
        fid.close();
        return Eigen::Matrix4f::Zero();
    }
    if(tline.find("- Rotation: in Quaternion [")!=string::npos && !fid.eof()){
        sscanf(tline.c_str(), "- Rotation: in Quaternion [%f, %f, %f, %f]", &x, &y, &z, &w);
        getline(fid, tline);
    }
    else{
        cout << "Erronous file format" << endl;
        fid.close();
        return Eigen::Matrix4f::Zero();
    }
    Eigen::Vector3f translation_vector(t1, t2, t3);
    Eigen::Matrix3f rotation_matrix = quaternion_to_rotation(x, y, z, w);
    return createTransformationMatrix(rotation_matrix, translation_vector);
    //cout << "Translation: " << t1 << " " << t2 << " " << t3 << endl;
    //cout << "Rotation: " << x << " " << y << " " << z << " " << w << endl;
    //cout << endl;
    fid.close();
}

int pcl_tools::topMostParent(int id){
    if(transformations[id-1].ok == false){
        return -1;
    }
    if(transformations[id-1].is_root){
        return id;
    }
    transformation_relation current_transform;
    current_transform = transformations[id-1];
    while(current_transform.is_root==false){
        current_transform = transformations[current_transform.parent_id-1];
        if(current_transform.ok == false){
            return -1;
        }
    }
    return current_transform.parent_id;
}


int pcl_tools::getInitialGuesses(string filename){
    string tline;
    fstream fid;
    int input, target;
    if (!file_exist(filename.c_str())) {
        std::cout << "Can not find initial guesses in " << filename << "; exiting" << std::endl;
        throw std::invalid_argument("no initial guess");
    }

    fid.open(filename.c_str(), ios::in);
    getline(fid, tline);
    while(!fid.eof()){
        while(tline.find("scan_")==string::npos && !fid.eof()){
            getline(fid, tline);
        }
        if(fid.eof()){
            std::cout << "Reached the end of initial guess file" << std::endl;
            fid.close();
            return 1;
        }
        sscanf(tline.c_str(), "scan_%d to scan_%d", &input, &target);
        getline(fid, tline);
        float t1, t2, t3;
        float x, y, z, w;
        if(tline.find("- Translation: [")!=string::npos && !fid.eof()){
            sscanf(tline.c_str(), "- Translation: [%f, %f, %f]", &t1, &t2, &t3);
            getline(fid, tline);
        }
        else{
            cout << "Erronous file format" << endl;
            fid.close();
            return 1;
        }
        if(tline.find("- Rotation: in Quaternion [")!=string::npos && !fid.eof()){
            sscanf(tline.c_str(), "- Rotation: in Quaternion [%f, %f, %f, %f]", &x, &y, &z, &w);
            getline(fid, tline);
        }
        else{
            cout << "Erronous file format" << endl;
            fid.close();
            return 1;
        }
        if(input==target){
            transformations[input-1].completed = true;
            transformations[input-1].is_root = true;
        }
        else{
            transformations[input-1].is_root = false;
        }
        Eigen::Vector3f translation_vector(t1, t2, t3);
        Eigen::Matrix3f rotation_matrix = quaternion_to_rotation(x, y, z, w);
        transformations[input-1].parent_id = target;
        transformations[input-1].init_guess = createTransformationMatrix(rotation_matrix, translation_vector);
        transformations[input-1].ok = true;
    }
    fid.close();
    return 0;
}



Eigen::Matrix4f pcl_tools::getTransformation(int input, int target){
    string tline;
    fstream fid;
    fid.open((config_->get_tf_directory() + "icp_result.txt").c_str(), ios::in);
    getline(fid, tline);
    stringstream transform_pair;
    transform_pair << "scan_" << input << " to " << "scan_" << target;
    while(tline.find(transform_pair.str())==string::npos && !fid.eof()){
        getline(fid, tline);
    }
    if(fid.eof()){
        std::cout << "Transformation from scan_" << input << " to scan_" << target << " not found" << std::endl;
        fid.close();
        return Eigen::Matrix4f::Zero();
    }
    Eigen::Matrix4f transformation;

    for(int i=0; i<4; i++){
        getline(fid, tline);
        sscanf(tline.c_str(), "%f %f %f %f", &transformation(i,0), &transformation(i,1), &transformation(i,2), &transformation(i,3));
    }
    fid.close();
    return transformation;
}


int pcl_tools::getTransformations(string filename, Eigen::Matrix4f transformation_matrices[]){
    string tline;
    fstream fid;
    int input, target;
    fid.open(filename.c_str(), ios::in);
    getline(fid, tline);
    while(!fid.eof()){
        while(tline.find("scan_")==string::npos && !fid.eof()){
            getline(fid, tline);
        }
        if(fid.eof()){
            std::cout << "Reached the end of initial guess file" << std::endl;
            fid.close();
            return 1;
        }
        sscanf(tline.c_str(), "scan_%d to scan_%d", &input, &target);
        for(int i=0; i<4; i++){
            getline(fid, tline);
            sscanf(tline.c_str(), "%f %f %f %f", &transformation_matrices[input-1](i,0), &transformation_matrices[input-1](i,1), &transformation_matrices[input-1](i,2), &transformation_matrices[input-1](i,3));
        }
    }
    fid.close();
    return 0;
}

int pcl_tools::computeGlobalTransformations(){
    fstream overall_icp, overall_ndt;
    overall_icp.open((config_->get_tf_directory() + "overall_icp.txt").c_str(), ios::out);
    overall_ndt.open((config_->get_tf_directory() + "overall_ndt.txt").c_str(), ios::out);

    for(int i=0; i<MAX_NUM_SCANS; i++){
        if(transformations[i-1].is_root){
            transformations[i-1].T = transformations[i-1].init_guess;
            transformations[i-1].T_ndt = transformations[i-1].init_guess;
            overall_icp << "scan_" << i << " to scan_" << i << std::endl;
            overall_icp << transformations[i-1].T << std::endl << std::endl;
            overall_ndt << "scan_" << i << " to scan_" << i << std::endl;
            overall_ndt << transformations[i-1].T_ndt << std::endl << std::endl;
            continue;
        }
        if(transformations[transformations[i-1].parent_id-1].completed==false){
            std::cerr << "Parent transformation is not completed for scan_" << i << std::endl;
            continue;
        }
        transformations[i-1].T = transformations[transformations[i-1].parent_id-1].T * transformations_icp[i-1] * transformations[i-1].init_guess;
        transformations[i-1].T_ndt = transformations[transformations[i-1].parent_id-1].T_ndt * transformations_ndt[i-1] * transformations[i-1].init_guess;
        transformations[i-1].completed = true;

        int top_parent = topMostParent(i);
        if(top_parent>=1 && top_parent<=MAX_NUM_SCANS){
            overall_icp << "scan_" << i << " to scan_" << top_parent << std::endl;
            overall_icp << transformations[i-1].T << std::endl << std::endl;

            overall_ndt << "scan_" << i << " to scan_" << top_parent << std::endl;
            overall_ndt << transformations[i-1].T_ndt << std::endl << std::endl;
        }
    }
    overall_icp.close();
    overall_ndt.close();

    return 0;
}


pcl::PointCloud<PointT>::Ptr pcl_tools::transform_pcd(pcl::PointCloud<PointT>::Ptr source_cloud, Eigen::Matrix4f transform_matrix)
{
    // Executing the transformation
    pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_matrix);

    return transformed_cloud;
}



Eigen::Matrix4f pcl_tools::apply_icp(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_out, bool viewResult){
    start_timer();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    icp.setMaxCorrespondenceDistance (1.5);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (2000);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(0.01);

    pcl::PointCloud<PointT> Final;
    icp.align(Final);
    std::cout << "ICP has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    std::cout << transformation << std::endl;

    end_timer("ICP has converged in:");

    std::cout << "Iterations: " << icp.nr_iterations_ << std::endl;

    if(viewResult){
        RegVisualizer reg_viewer;
        reg_viewer.viewICPResult(cloud_in, cloud_out, Final.makeShared());
    }

    return transformation;
}

Eigen::Matrix4f pcl_tools::do_gicp(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt) {
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (config_->get_init_max_correspondence_distance());
    reg.setEuclideanFitnessEpsilon(0.01);
    // Set the point representation
    /* reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation)); */

    reg.setInputSource (cloud_src);
    reg.setInputTarget (cloud_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloud::Ptr reg_result = cloud_src;

    reg.setMaximumIterations (1000);
    for (int i = 0; i < 10; ++i) {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        /* points_with_normals_src = reg_result; */

        // Estimate
        reg.setInputSource (cloud_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ()){
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - config_->get_max_correspondence_distance_step());
        }

        prev = reg.getLastIncrementalTransformation ();
    }
    return Ti;
}


Eigen::Matrix4f pcl_tools::do_icp(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt) {
    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud(cloud_src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*cloud_src, *points_with_normals_src);

    norm_est.setInputCloud (cloud_tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*cloud_tgt, *points_with_normals_tgt);

    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (config_->get_init_max_correspondence_distance());
    reg.setEuclideanFitnessEpsilon(0.01);
    // Set the point representation
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (1000);
    for (int i = 0; i < 10; ++i) {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ()){
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - config_->get_max_correspondence_distance_step());
        }

        prev = reg.getLastIncrementalTransformation ();
    }
    return Ti;
}

int pcl_tools::apply_icp(string path_in, string path_out){
    //Read input cloud
    std::cout << "Reading input cloud" << std::endl;
    start_timer();
    pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (path_in, *cloud_in) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    end_timer("ICP: Input cloud loaded in:");

    //Read output cloud
    std::cout << "Reading output cloud" << std::endl;
    start_timer();
    pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (path_out, *cloud_out) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    end_timer("ICP: Output cloud loaded in:");


    start_timer();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<PointT> Final;
    icp.align(Final);
    std::cout << "ICP has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    end_timer("ICP has converged in:");
    return 0;
}

Eigen::Matrix4f pcl_tools::do_ndt(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f init_guess){

    //start_timer();
    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.00001);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (8.0);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (35);

    // Setting point cloud to be aligned.
    ndt.setInputSource (cloud_in);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (target_cloud);

    // Set initial alignment estimate found using robot odometry.
    /*Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();*/

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<PointT>::Ptr output_cloud (new pcl::PointCloud<PointT>);
    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << std::endl;

    return ndt.getFinalTransformation ();


    //std::cout << ndt.getFinalTransformation () << std::endl;
    /*//end_timer("NDT has converged in:");

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*cloud_in, *output_cloud, ndt.getFinalTransformation ());

    // Initializing point cloud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<PointT>
            target_color (target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<PointT> (target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<PointT>
            output_color (output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<PointT> (output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "output cloud");

    // Starting visualizer
    //viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }*/
}

pcl::PointCloud<PointT>::Ptr pcl_tools::loadPCD(string path){
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (cloud);
    }
    else{
        return cloud;
    }
}



pcl::PointCloud<PointT>::Ptr pcl_tools::getSlice(pcl::PointCloud<PointT>::Ptr cloud, float z1, float z2, string field_name){
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (field_name);
    pass.setFilterLimits (z1, z2);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_tools::getSliceRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float z1, float z2, string field_name){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (field_name);
    pass.setFilterLimits (z1, z2);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    return cloud_filtered;
}

void pcl_tools::savePCD(pcl::PointCloud<PointT>::Ptr cloud, string path){
    cloud->width = 1;
    cloud->height = cloud->points.size();
    pcl::io::savePCDFileASCII (path, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to pcd." << std::endl;
    std::cerr << "Cuboid size" << cloud->points.size() << std::endl;
}


void pcl_tools::getPCDStatistics(string dir, int low_ind, int up_ind){
    for(int i=low_ind; i<=up_ind; i++){
        string filename = dir;
        filename.append("/scan_");
        string num = int2str(i);
        filename.append(num.c_str());
        filename.append(".pcd");
        std::cout << "Loading " << filename << std::endl;
        pcl::PointCloud<PointT>::Ptr cloud = loadPCD(filename);
        PointT minPt, maxPt;
        pcl::getMinMax3D (*cloud, minPt, maxPt);
        std::cout << "Min: " << minPt.x << " " << minPt.y << " " << minPt.z << std::endl;
        std::cout << "Max: " << maxPt.x << " " << maxPt.y << " " << maxPt.z << std::endl;
    }
}


pcl::PointCloud<PointT>::Ptr pcl_tools::projectPCD(pcl::PointCloud<PointT>::Ptr cloud, float a, float b, float c, float d){
    pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>);

    // Create a set of planar coefficients
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = a;
    coefficients->values[1] = b;
    coefficients->values[2] = c;
    coefficients->values[3] = d;

    // Create the filtering object
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    return cloud_projected;
}

