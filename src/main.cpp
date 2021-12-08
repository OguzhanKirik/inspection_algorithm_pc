
//#include "workObjectDetection.h"



#include<string>
#include<iostream>
#include <vector>



#include <Eigen/Geometry>
#include <Eigen/Core>

#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>


#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h> // extract borders


#include <pcl/registration/icp.h>// ICP algorit


#include<pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h> // filter point cloud


// segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_line.h> // line segmentation

// Circle Detection in Point cloud
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/octree/octree_search.h> // octree search


#include "data_type.h"


// FOR THE SPARS SCAN
 std::vector<cv::Vec3f> findClosestPointsInSparsePC(pcl::PointCloud<pcl::PointXYZ>::Ptr sparsePointCloud,
                                                     const std::vector<cv::Vec3f>& selectedPoints );




// returns the final poses of drill targets and check edge Tolerances
std::vector<drillTarget> inspectDrillTargets(const std::vector<boundingBox>& BBs,
                                            std::vector<drillTarget>& drillTargets,
                                            const params& params); 

void matchCadTargetsWithDetectedHoles(std::vector<drillTarget>& drillTargets,std::vector<drillTarget>& detectedHoles,const float& maxCorrespondenceDistance);

// Given the detected Bounnding boxes and the corresponding range image, it segments the holes in the point cloud and returns the centers
std::vector<drillTarget>  getDetectedHolesWithPointCloud(const  std::vector<boundingBox>& BBs,const std::string& auditFolder,const float& scanDisToWorkpiece);

std::vector<float> detect3DCirclePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered); // return center of the holes in point cloud

pcl::PointCloud<pcl::PointXYZ>::Ptr createRangeImageBorderExtaction(const params& params,const cv::Affine3f& profilometerPose); // create another range iamge with lower resolution to extract borders
pcl::PointCloud<pcl::PointXYZ>::Ptr borderExtraction(const pcl::RangeImage& range_image); // returns border points on the range image
void checkEdgeTolerances(std::vector<drillTarget>& drillTargets,const params& params,const cv::Affine3f& profilometerPose); // Creates a octree and finds the k closest neighbor 
void checkCenterTolerances(std::vector<drillTarget>& drillTargets,const params& params,const cv::Affine3f& profilometerPose);

// It uses Icp to align the drill targets extracted from Cad model and the detected piloted&&fullsized holes
std::vector<drillTarget>  alignDrillTargets( std::vector<drillTarget>& drillTargets,const float& maxCorrespondenceDistance); 

cv::Affine3f calculateProfilometerPose(std::vector<drillTarget>& drillTargets,const params& params);

void transformDrillTarges(std::vector<drillTarget>& drillTargets,const Eigen::Matrix4f& transform_matrix); // transfrom all drilltargets

Eigen::Vector3f getPointLocationInWorldFrame(const cv::Point2i& image_point, const float& range,const range_image& range_image_info);
cv::Point2f getPixelCoordinates(const Eigen::Vector3f& point3D,const range_image& range_image_info );
range_image readRangeImageFile(const std::string& auditFolder);






pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(const std::string& cloudPath); // read pcd
pcl::PointCloud<pcl::PointXYZ>::Ptr readPLYFile(const std::string& cloudPath); // read ply
float atanCalc (float y, float x); // helper function for 3d-2d mapping
std::vector<drillTarget> readPosesFromFile (const std::string& pathTOAlignedHolesLoc); // read poses
cv::Matx33f rotation_from_euler(float roll, float pitch, float yaw); // 
Quaternion ToQuaternion(const float& yaw, const float& pitch, const float& roll); // yaw (Z), pitch (Y), roll (X)
EulerAngles ToEulerAngles(const Quaternion& q);
// ffilter the floor
void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
					const std::string& axis,
					const float (&limits)[2]);




int main(int argc, char **argv) {
        
        /*
        // audit_1
        std::vector<cv::Vec3f> circles;
        cv::Vec3f circle1{2188, 154, 18}; // true
        circles.emplace_back(circle1);
       cv::Vec3f circle2{2746, 2296,18}; // true but belong to another cluster
        circles.emplace_back(circle2);
        cv::Vec3f circle3{1904, 584, 18}; // true
        circles.emplace_back(circle3);
        cv::Vec3f circle4{1900, 1046, 18}; // true
        circles.emplace_back(circle4);
        cv::Vec3f circle5{974, 658,18}; // false
        circles.emplace_back(circle5);
        cv::Vec3f circle6{1500, 700,18}; // false
        circles.emplace_back(circle6);
        */
        /*
        // audit_2, holes on the both sides
        std::vector<cv::Vec3f> circles;
        cv::Vec3f circle1{746, 1066, 18}; // true
        circles.emplace_back(circle1);
       cv::Vec3f circle2{358, 1936, 18}; // true 
        circles.emplace_back(circle2);
        cv::Vec3f circle3{352, 560, 18}; // true
        circles.emplace_back(circle3);
        cv::Vec3f circle4{352, 1024, 18}; // true
        circles.emplace_back(circle4);
        cv::Vec3f circle5{736, 1976, 18}; // false
        circles.emplace_back(circle5);
        cv::Vec3f circle6{742, 1532, 18}; // false
        circles.emplace_back(circle6);
         cv::Vec3f circle7{350, 1490, 18}; // false
        circles.emplace_back(circle7);
       */
        
        /*
        // audit_3
        std::vector<cv::Vec3f> circles;
        cv::Vec3f circle1{1858, 1450,18}; // true
        circles.emplace_back(circle1);
       cv::Vec3f circle2{1856, 1922, 18}; // true
        circles.emplace_back(circle2);
        cv::Vec3f circle3{1856, 982, 18}; // true
        circles.emplace_back(circle3);
        cv::Vec3f circle4{1858, 518, 16}; // true
        circles.emplace_back(circle4);
        cv::Vec3f circle5{974, 658,18};
        circles.emplace_back(circle5);
        cv::Vec3f circle6{1500, 700,18};
        circles.emplace_back(circle6);
        */
        
        // audit_4, holes on the both sides
        std::vector<cv::Vec3f> circles;
        cv::Vec3f circle1{542, 1940, 18}; // true
        circles.emplace_back(circle1);
       cv::Vec3f circle2{272, 1114, 18}; // true 
        circles.emplace_back(circle2);
        cv::Vec3f circle3{272, 646, 18}; // true
        circles.emplace_back(circle3);
        cv::Vec3f circle4{1718, 868, 18}; // true
        circles.emplace_back(circle4);
        cv::Vec3f circle5{530,1018, 18}; // false
        circles.emplace_back(circle5);
        cv::Vec3f circle6{1426, 1880, 18}; // false
        circles.emplace_back(circle6);
         cv::Vec3f circle7{1454, 1474, 18}; // false
        circles.emplace_back(circle7);
        cv::Vec3f circle8{532, 554, 18}; // false
        circles.emplace_back(circle8);
        cv::Vec3f circle9{1600, 1120 ,18}; // false
        circles.emplace_back(circle9);
         cv::Vec3f circle10{268, 2038 ,18}; // false
        circles.emplace_back(circle10);
        cv::Vec3f circle11{268, 2038 ,18}; // false
        circles.emplace_back(circle11);
        cv::Vec3f circle12{640 ,1484 ,18}; // false
        circles.emplace_back(circle12);


        
        std::vector<boundingBox> BBs;
        for (int i = 0; i < circles.size(); i++){ // search points in boundingBox
            boundingBox bb;
            bb.createBB(circles.at(i));
            BBs.emplace_back(bb);
      }
        
        params params;


        std::string pathToHoles= params.auditFolder + "/holes_dense4.txt";
        std::vector<drillTarget> drillTargets = readPosesFromFile(pathToHoles); //That comes from dataBase

        // Implement finding the ids
        // check the flow
        // points less than 3
        std::vector<drillTarget> drillTargets_Inspected = inspectDrillTargets(BBs,drillTargets,params);
        
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr searchPoint_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < drillTargets.size(); i++){
            pcl::PointXYZ p{drillTargets.at(i).position[0],drillTargets.at(i).position[1],drillTargets.at(i).position[2]};
            searchPoint_ptr->points.emplace_back(p);
        }
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(searchPoint_ptr, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ> (searchPoint_ptr, color2, "searchPoint_ptr");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        while (!viewer->wasStopped ()) {
         viewer->spinOnce ();
        }
        
            
        

        

    return 0;
}

std::vector<drillTarget> inspectDrillTargets(const std::vector<boundingBox>& BBs,
                                            std::vector<drillTarget>& drillTargets,
                                            const params& params){
        
        cv::Affine3f profilometerPose = calculateProfilometerPose(drillTargets,params);

        // if there is no BBs found, then skip the alignment and check toletances
        if(!BBs.empty()){
            std::vector<drillTarget> detectedHoles = getDetectedHolesWithPointCloud(BBs,params.auditFolder,params.scanDisToWorkpiece); // extract hole centers and Labels; unpiloted,piloted,fullsized
            
            matchCadTargetsWithDetectedHoles(drillTargets,detectedHoles, params.maxCorrespondenceDistance); // find the correspondence between cad targets&detected targets discard misprediction
            
            alignDrillTargets(drillTargets,params.maxCorrespondenceDistance); // Apply transformation

        }else{
            for (size_t i = 0; i < drillTargets.size(); i++){
                drillTargets.at(i).position_aligned =drillTargets.at(i).position;
                drillTargets.at(i).fullSized=false;
                drillTargets.at(i).piloted=false;
                drillTargets.at(i).detectedHoleDiameter=0;
            }
        }
        
        checkCenterTolerances(drillTargets,params,profilometerPose); // check tolerance between centers
       
        checkEdgeTolerances(drillTargets,params,profilometerPose); // Check tolerace to edges

        return drillTargets;
}


cv::Affine3f calculateProfilometerPose(std::vector<drillTarget>& drillTargets,const params& params){


        for (size_t i = 0; i < drillTargets.size(); i++){
            EulerAngles ea = ToEulerAngles(drillTargets.at(i).quartenion);
            drillTargets.at(i).rotation_matrix = rotation_from_euler(ea.roll,ea.pitch,ea.yaw);
            cv::Affine3f homogeneousmatrix{drillTargets.at(i).rotation_matrix, drillTargets.at(i).position};
            drillTargets.at(i).homogeneousMat = homogeneousmatrix;
        }

        // TCP Pose
        drillTarget middlePoint = drillTargets.at((std::floor(drillTargets.size()/2)));
        cv::Vec3f offset{0,0,params.sensorDistanceOffset};
        cv::Vec3f translationInSensorFrame = middlePoint.rotation_matrix * offset;
        cv::Affine3f robotPose = middlePoint.homogeneousMat.translate(translationInSensorFrame);
        
        // Profilometer calibration
        cv::Affine3f profilometerCalib;
        profilometerCalib.rotation(rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]));
        cv::Vec3f profilometerTranslation{params.profilometerCalibTranslation};
       
        
        cv::Affine3f sensorPose = robotPose * profilometerCalib ; 
        cv::Affine3f sensorPoseFinal = sensorPose.translate(profilometerTranslation);
        return sensorPoseFinal;
    }

   



void checkCenterTolerances(std::vector<drillTarget>& drillTargets, const params& params,const cv::Affine3f& profilometerPose){
    // Find Adjacent drill targets  in two rows
     std::vector<int> ID_first_row;
     std::vector<int> ID_second_row;
     
    // The first drill target in the list is the starting point of the first_row
     cv::Vec3f position_inSensorFrame_first = profilometerPose * drillTargets.at(0).position_aligned;

     int count_first_row = 0; 
     int count_second_row = 1; 

     drillTargets.at(0).ID = count_first_row;
     ID_first_row.emplace_back(count_first_row);
     

   
     for (size_t i = 1; i < drillTargets.size(); i++){
         cv::Vec3f position_inSensorFrame = profilometerPose * drillTargets.at(i).position_aligned;
            if(position_inSensorFrame[0] <= (position_inSensorFrame_first[0] + params.shiftInHoleCenter) &&  
            position_inSensorFrame[0] >= (position_inSensorFrame_first[0] - params.shiftInHoleCenter)){
                 count_first_row +=2;
                ID_first_row.emplace_back(i); 
                drillTargets.at(i).ID = count_first_row;
            }else{
                ID_second_row.emplace_back(i);
                drillTargets.at(i).ID = count_second_row;
                count_second_row +=2;
            }
     }
    

     // number of holes nees to be bigger than 1 for checking center tolerances
     if(ID_first_row.size() > 1 ){
        drillTargets.at(ID_first_row.front()).distanceToDownerCenter=0;
        drillTargets.at(ID_first_row.back()).distanceToUpperCenter=0;
         for (size_t i = 1; i < ID_first_row.size(); i++){
            cv::Vec3f centerDifference;
            
            centerDifference = drillTargets.at(ID_first_row.at(i)).position_aligned - drillTargets.at(ID_first_row.at(i-1)).position_aligned;
            float distance = cv::norm(centerDifference,cv::NORM_L2);
            drillTargets.at(ID_first_row.at(i-1)).distanceToUpperCenter = distance;
            drillTargets.at(ID_first_row.at(i)).distanceToDownerCenter = distance;

            if(distance < params.centerToleranceMin || distance > params.centerToleranceMax){
                std::cout << distance  <<"false centerTolerance"  << std::endl;
                drillTargets.at(ID_first_row.at(i-1)).withinCenterToleranceUpper =false;
                drillTargets.at(ID_first_row.at(i)).withinCenterToleranceDowner =false;
             }else{
                 std::cout << distance <<" true centerTolerance" <<std::endl;
                drillTargets.at(ID_first_row.at(i-1)).withinCenterToleranceUpper =true;
                drillTargets.at(ID_first_row.at(i)).withinCenterToleranceDowner =true;
             }
        }
    }
         
// number of holes nees to be bigger than 1 for checking center tolerances
     if(ID_second_row.size() > 1 ){
        drillTargets.at(ID_second_row.front()).distanceToDownerCenter=0;
        drillTargets.at(ID_second_row.back()).distanceToUpperCenter=0;

         for (size_t i = 1; i < ID_second_row.size(); i++){
            cv::Vec3f centerDifference;
           
            centerDifference = drillTargets.at(ID_second_row.at(i)).position_aligned - drillTargets.at(ID_second_row.at(i-1)).position_aligned;
            float distance = cv::norm(centerDifference,cv::NORM_L2);
            drillTargets.at(ID_second_row.at(i-1)).distanceToUpperCenter = distance;
            drillTargets.at(ID_second_row.at(i)).distanceToDownerCenter = distance;

            if(distance < params.centerToleranceMin || distance > params.centerToleranceMax){
                std::cout << distance  <<"false centerTolerance"  << std::endl;
               drillTargets.at(ID_second_row.at(i-1)).withinCenterToleranceUpper =false;
                drillTargets.at(ID_second_row.at(i)).withinCenterToleranceDowner =false;
            }else{
                 std::cout <<" true centerTolerance" <<std::endl;
                 drillTargets.at(ID_second_row.at(i-1)).withinCenterToleranceUpper =true;
                drillTargets.at(ID_second_row.at(i)).withinCenterToleranceDowner =true;
             }
            }
        }
    }


std::vector<drillTarget>  getDetectedHolesWithPointCloud(const std::vector<boundingBox>& BBs, const std::string& auditFolder,const float& scanDisToWorkpiece){
     
     // call pixel coordinates within bounding box in range image to get their pos in 3D
    std::vector<drillTarget> detectedHoles;

    range_image range_image_info = readRangeImageFile(auditFolder);
    std::string imagePath = auditFolder + "/rangeImage.png";
    cv::Mat image = cv::imread(imagePath,cv::IMREAD_COLOR);

        for (int k = 0; k < BBs.size(); k++){
            
            std::vector<float> ranges;
            for (int u = BBs.at(k).topRight.y; u < BBs.at(k).bottomLeft.y; u++){  
                for (int v = BBs.at(k).bottomLeft.x; v <  BBs.at(k).topRight.x; v++){
                    cv::Point2i image_point{u,v}; 
                    float pixelValue = (float)(image.at<cv::Vec3b>(image_point.x,image_point.y)[0]);// height,width
                    float range = (pixelValue / 255) * range_image_info.max; // calculate range from pixel intensity
                    
                    if(range != 0 && range < scanDisToWorkpiece ){
                        ranges.emplace_back(range); 

                    }
                }
            }
            if(!ranges.empty()){
                // Calculate 3d location of hole center
                float average_range = (float)(std::accumulate(ranges.begin(),ranges.end(),0) / ranges.size()); // find average range in bb
                cv::Point2i image_point{BBs.at(k).center.y, BBs.at(k).center.x}; // center pixel coordinates, height, width
                //float pixelValue = (float)(image.at<cv::Vec3b>(image_point.x,image_point.y)[0]);// height,width
                Eigen::Vector3f point3D = getPointLocationInWorldFrame(image_point,average_range,range_image_info);
                cv::Vec3f position{point3D.x(),point3D.y(),point3D.z()}; 
                drillTarget detectedHole;
                detectedHole.position = position;  
                if(BBs.at(k).isFullSized()) 
                     detectedHole.fullSized = true;
                 else    
                      detectedHole.piloted = true;
            
                 detectedHoles.emplace_back(detectedHole);
            }
            
        }   
        return detectedHoles; 
}

std::vector<drillTarget>  alignDrillTargets(std::vector<drillTarget>& drillTargets,const float& maxCorrespondenceDistance){

	pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_cad (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations (new pcl::PointCloud<pcl::PointXYZ>);

    // create the point cloud with holes provided after dense alignment // find the correspding point and discard it
	for (size_t i = 0; i < drillTargets.size(); i++){
        if(drillTargets.at(i).fullSized == true ||  drillTargets.at(i).piloted == true){
            pilot_holes_locations_cad->points.emplace_back(drillTargets.at(i).position[0],drillTargets.at(i).position[1],drillTargets.at(i).position[2]);
            //if(drillTargets.at(i).misPrediction == false){
                pilot_holes_locations->points.emplace_back(drillTargets.at(i).position_aligned[0],drillTargets.at(i).position_aligned[1],drillTargets.at(i).position_aligned[2]);
            //}
	    }
    }


    // The Iterative Closest Point algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    int iterations = 1000;
    icp.setInputSource (pilot_holes_locations_cad);
    icp.setInputTarget (pilot_holes_locations);
    icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance); 
    icp.setMaximumIterations (iterations);
    icp.align (*pilot_holes_locations_cad);

    if (icp.hasConverged ()){
        Eigen::Matrix4f  transformation_matrix = icp.getFinalTransformation();
        transformDrillTarges(drillTargets,transformation_matrix);
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl; // 
  }
  else{
    std::cout << "alignment was not possible " << std::endl;
    for (size_t i = 0; i < drillTargets.size(); i++){
        if(drillTargets.at(i).fullSized ==false && drillTargets.at(i).piloted ==false )
            drillTargets.at(i).position_aligned = drillTargets.at(i).position;
    }
    
  }
  return drillTargets;
     
}

void transformDrillTarges(std::vector<drillTarget>& drillTargets,const Eigen::Matrix4f& transform_matrix){
            cv::Matx33f rot_a;
            cv::Vec3f tra_a;
            rot_a(0,0) =  transform_matrix(0,0);
            rot_a(0,1) =  transform_matrix(0,1);
            rot_a(0,2) =  transform_matrix(0,2);
            rot_a(1,0) =  transform_matrix(1,0);
            rot_a(1,1) =  transform_matrix(1,1);
            rot_a(1,2) =  transform_matrix(1,2);
            rot_a(2,0) =  transform_matrix(2,0);
            rot_a(2,1) =  transform_matrix(2,1);
            rot_a(2,2) =  transform_matrix(2,2);
            tra_a(0) =  transform_matrix(0,3);
            tra_a(1) =  transform_matrix(1,3);
            tra_a(2) =  transform_matrix(2,3);
            cv::Affine3f transform{rot_a,tra_a};
        for (size_t i = 0; i < drillTargets.size(); i++){
            if(drillTargets.at(i).fullSized == false && drillTargets.at(i).piloted == false){
                drillTargets.at(i).position_aligned = transform * drillTargets.at(i).position;
            }          
    }
    
}


pcl::PointCloud<pcl::PointXYZ>::Ptr borderExtraction(const pcl::RangeImage& range_image){
     
     pcl::RangeImageBorderExtractor border_extractor (&range_image);
     pcl::PointCloud<pcl::BorderDescription> border_descriptions;
     border_extractor.compute(border_descriptions);

     pcl::PointCloud<pcl::PointXYZ>::Ptr border_points_ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int y=0; y< (int)range_image.height; ++y){
        for (int x=0; x< (int)range_image.width; ++x) {
            if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER_LEFT] ||
            border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER_RIGHT ]) {
                pcl::PointXYZ p{range_image[y*range_image.width + x].x,range_image[y*range_image.width + x].y,range_image[y*range_image.width + x].z};
                border_points_ptr_cloud->points.emplace_back(p); 
            }
  
    }
  }
 
   return border_points_ptr_cloud;
    
}

void checkEdgeTolerances(std::vector<drillTarget>& drillTargets,const params& params,const cv::Affine3f& profilometerPose){


    pcl::PointCloud<pcl::PointXYZ>::Ptr border_points_ptr_cloud = createRangeImageBorderExtaction(params,profilometerPose); // Extract border points
    
    // Octree 
     float resolution = 128.0f;

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud (border_points_ptr_cloud);
        octree.addPointsFromInputCloud ();
        int K = 5;
    for (size_t i = 0; i < drillTargets.size(); i++){
       
        pcl::PointXYZ searchPoint{drillTargets.at(i).position_aligned[0],drillTargets.at(i).position_aligned[1],drillTargets.at(i).position_aligned[2]};
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        

        if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
             pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_neigbour_ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < K; i++){

                pcl::PointXYZ p{(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].x,(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].y,(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].z};
                nearest_neigbour_ptr_cloud->points.emplace_back(p);
            }
            // Segment a line 
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	        // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_LINE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations(10000);
            seg.setDistanceThreshold (5);
            //seg.setEpsAngle(0.01);
            seg.setInputCloud (nearest_neigbour_ptr_cloud);
            seg.segment (*inliers, *coefficients);






            Eigen::Vector4f position{coefficients->values[0],coefficients->values[1],coefficients->values[2],0};
            Eigen::Vector4f direction{ coefficients->values[3],coefficients->values[4],coefficients->values[5],0};
            Eigen::Vector4f point{drillTargets.at(i).position_aligned[0],drillTargets.at(i).position_aligned[1],drillTargets.at(i).position_aligned[2],0};


            double distance  = std::sqrt(pcl::sqrPointToLineDistance(point,position,direction));
            std::cout <<   drillTargets.at(i).position_aligned <<" distance to Edge : " << distance << std::endl;
            drillTargets.at(i).distanceToEdge = (float)(distance);

            if(drillTargets.at(i).distanceToEdge <= params.edgeToleranceMax && 
                drillTargets.at(i).distanceToEdge >= params.edgeToleranceMin ){
                    drillTargets.at(i).withinEdgeTolerance = true;
                    std::cout << "True edge tolerance" <<std::endl;
            }else{
                drillTargets.at(i).withinEdgeTolerance = false;
                std::cout <<  " false  edge tolerance" <<std::endl;
            }

        }
        
    }
}



 void matchCadTargetsWithDetectedHoles(std::vector<drillTarget>& drillTargets,std::vector<drillTarget>& detectedHoles, const float& maxCorrespondenceDistance ){


    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_detected (new pcl::PointCloud<pcl::PointXYZ>);
    
    // create the point cloud with holes provided after dense alignment // find the correspding point and discard it
	for (size_t i = 0; i < drillTargets.size(); i++){
        //if(drillTargets.at(i).fullSized == true || drillTargets.at(i).piloted == true)
            pilot_holes_locations_detected->points.emplace_back(drillTargets.at(i).position[0],drillTargets.at(i).position[1],drillTargets.at(i).position[2]);
	}
    // Octree 
    for (size_t i = 0; i < detectedHoles.size(); i++){

            pcl::PointXYZ searchPoint{detectedHoles.at(i).position[0],detectedHoles.at(i).position[1],detectedHoles.at(i).position[2]};
            float resolution = 128.0f;
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
            octree.setInputCloud (pilot_holes_locations_detected);
            octree.addPointsFromInputCloud ();
        
             int K = 1; // closest neighbour,

            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;
 
             if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                if( std::sqrt(pointNKNSquaredDistance[0]) < maxCorrespondenceDistance)  {
                    cv::Vec3f position_matched{(*pilot_holes_locations_detected)[ pointIdxNKNSearch[0] ].x ,
                     (*pilot_holes_locations_detected)[ pointIdxNKNSearch[0] ].y,
                    (*pilot_holes_locations_detected)[ pointIdxNKNSearch[0] ].z};
                    detectedHoles.at(i).position_aligned = position_matched;             
                }
             }else{
                 detectedHoles.at(i).misPrediction=true;
             }
       
    }
   
    // inherit the features of detectedhole; fullsize&pilotsize, center, diameter
    for (size_t i = 0; i < drillTargets.size(); i++){
       for (size_t j = 0; j < detectedHoles.size(); j++){
           if(drillTargets.at(i).position == detectedHoles.at(j).position_aligned){
               drillTargets.at(i).fullSized = detectedHoles.at(j).fullSized;
               drillTargets.at(i).piloted = detectedHoles.at(j).piloted;
               drillTargets.at(i).position_aligned = detectedHoles.at(j).position;
               drillTargets.at(i).detectedHoleDiameter = detectedHoles.at(j).detectedHoleDiameter;
           }
       }
       
    }
}   


/*
void matchCadTargetsWithDetectedHoles(std::vector<drillTarget>& drillTargets,std::vector<drillTarget>& detectedHoles, const float& maxCorrespondenceDistance ){


    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_detected (new pcl::PointCloud<pcl::PointXYZ>);
    
    // create the point cloud with holes provided after dense alignment // find the correspding point and discard it
	for (size_t i = 0; i < detectedHoles.size(); i++){
        pilot_holes_locations_detected->points.emplace_back(detectedHoles.at(i).position[0],detectedHoles.at(i).position[1],detectedHoles.at(i).position[2]);
	}
    // Octree 
    for (size_t i = 0; i < drillTargets.size(); i++){

            pcl::PointXYZ searchPoint{drillTargets.at(i).position[0],drillTargets.at(i).position[1],drillTargets.at(i).position[2]};

            float resolution = 128.0f;
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
            octree.setInputCloud (pilot_holes_locations_detected);
            octree.addPointsFromInputCloud ();
        
             int K = 1; // closest neighbour,

            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;
 
             if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                if( std::sqrt(pointNKNSquaredDistance[0]) < maxCorrespondenceDistance)  {
                    cv::Vec3f holeCenter{(*pilot_holes_locations_detected)[ pointIdxNKNSearch[0] ].x ,
                     (*pilot_holes_locations_detected)[ pointIdxNKNSearch[0] ].y,
                    (*pilot_holes_locations_detected)[ pointIdxNKNSearch[0] ].z};
                    drillTargets.at(i).position_aligned = holeCenter;             
                }
             }
       
    }
   
    // inherit the features of detectedhole; fullsize&pilotsize, center, diameter
    for (size_t i = 0; i < drillTargets.size(); i++){
       for (size_t j = 0; j < detectedHoles.size(); j++){
           if(drillTargets.at(i).position_aligned == detectedHoles.at(j).position){
               drillTargets.at(i).fullSized = detectedHoles.at(j).fullSized;
               drillTargets.at(i).piloted = detectedHoles.at(j).piloted;
               drillTargets.at(i).detectedHoleDiameter = detectedHoles.at(j).detectedHoleDiameter;
           }else{
                detectedHoles.at(j).misPrediction=true;
           }
       }
       
    }
}
*/
// same range image function can be called with different angular_resolution

pcl::PointCloud<pcl::PointXYZ>::Ptr createRangeImageBorderExtaction(const params& params,const cv::Affine3f& profilometerPose){

        
      
        // Read point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readPLYFile(params.cloudPath);

        // filter PC
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        float filterCloudRegion[2]{params.filterParam,std::numeric_limits<float>::max()}; 
        std::string axis = "z";
        filterPointCloud(cloud,cloud_filtered,axis,filterCloudRegion);



        float angularResolution_x = (float) (params.angular_resolution_border * (M_PI/180.0f));  
        float angular_resolution_y = (float) (params.angular_resolution_border * (M_PI/180.0f));
        float maxAngleWidth     = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
        float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians

        cv::Affine3f sensorPose = profilometerPose;
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float noiseLevel=0.00;
        float minRange = 0.0f;
        int borderSize = 100;


        pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
        pcl::RangeImage rangeImageBorder = *range_image_ptr;
   	    // Creating Range image
        rangeImageBorder.createFromPointCloud(*cloud_filtered, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

        
        rangeImageBorder.setUnseenToMaxRange();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr border_points  = borderExtraction(rangeImageBorder);


         /* 
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(border_points, 255, 255, 0);
       
        viewer->addPointCloud<pcl::PointXYZ> (border_points, color1, "sample cloud");
      
     
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();

        while (!viewer->wasStopped ()) {
         viewer->spinOnce ();
        }
        */
        return border_points;

}







std::vector<float> detect3DCirclePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){



  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType (pcl::SAC_RANSAC);
    
  seg.setRadiusLimits(0,5);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold (0.1);
   //seg.setEpsAngle(0.01);
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  std::vector<float> worldPoint{coefficients->values[0],coefficients->values[1],coefficients->values[2],
   coefficients->values[3],coefficients->values[4],coefficients->values[5],coefficients->values[6]};


   return worldPoint;

}



pcl::PointCloud<pcl::PointXYZ>::Ptr readPLYFile(const std::string& cloudPath){
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	Reader.read(cloudPath, *cloud);
	if ( cloud->width <1){ 
    	//PCL_ERROR ("Couldn't read file point_cloud.ply \n");
    	throw std::runtime_error("Couldn't read cloud ply file \n");
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(const std::string& cloudPath){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
    	//PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		throw std::runtime_error("Couldn't read cloud pcd file \n");
    	
  	}
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points "
            << std::endl;
	
	return cloud;
	
}



void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
													pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
													const std::string& axis,
													const float (&limits)[2]){
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
 	pass.setInputCloud (cloud);
  	pass.setFilterFieldName (axis);
  	pass.setFilterLimits (limits[0],limits[1]);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter (*cloud_filtered);
	
}




std::vector<drillTarget> readPosesFromFile (const std::string& pathTOAlignedHolesLoc){
    // read holes location from file
    std::vector<drillTarget> res;
	std::vector<cv::Vec3f> positions; 
	std::vector<Quaternion> qs;
    std::ifstream file(pathTOAlignedHolesLoc);
    if (file.is_open()) { 
        std::string line;
        while (getline(file,line)){
             if(line[0] == 'i' || line.empty())
                 continue;
            else{
                drillTarget d;
                //d.piloted = true; // read from file
                float id,x,y,z;
                Quaternion q;
                std::stringstream sin(line);
                sin >>id >>  d.position[0] >> d.position[1] >> d.position[2] >> d.quartenion.w >> d.quartenion.x >> d.quartenion.y >> d.quartenion.z;
                res.emplace_back(d);
            }
        }
    }
    
    
    return res;
}
Eigen::Vector3f getPointLocationInWorldFrame(const cv::Point2i& image_point, const float& range,const range_image& range_image_info){
      
    // given pixel cooordinates and range, calculate the 3d points
     Eigen::Vector3f res;

    float angular_resolution_x=(float) (0.01f * (M_PI/180.0f)); 
    float angular_resolution_y =(float) (0.01f * (M_PI/180.0f)); 

    float angle_x, angle_y;
    // calculate ray angles
    angle_y = (image_point.x+static_cast<float> (range_image_info.image_offset_y))*(float)angular_resolution_x - 0.5f*static_cast<float> (M_PI);
    angle_x = (cosf (angle_y)==0.0f ? 0.0f : ( (image_point.y  + static_cast<float> (range_image_info.image_offset_x))*(float) angular_resolution_y - static_cast<float> (M_PI))/cosf (angle_y));
  
    res = Eigen::Vector3f (range * sinf (angle_x) * cosf (angle_y), range * sinf (angle_y), range * cosf (angle_x)*cosf (angle_y));
    Eigen::Affine3f sensorPose{range_image_info.sensorPose};
    res = sensorPose * res; // convert into world coordinate

    return res;
}

cv::Point2f getPixelCoordinates(const Eigen::Vector3f& point3D,const range_image& range_image_info){
    

 
    // Given the points in 3d calculate the pixel coordinates
    cv::Point2i res;
    // 3d points in sensor coordinate system. In case, the points are given 
    //in world coordinate fram, transform them into sensor frame

    Eigen::Affine3f sensorPose{range_image_info.sensorPose};
    Eigen::Vector3f transformedPoint = sensorPose.inverse() * point3D; // convert into sensor coodinate
    auto range = transformedPoint.norm ();

    float angular_resolution_x_reciprocal_= 1.0/(float) (0.01f * (M_PI/180.0f)); // 1/ angular_resolution_x
    float angular_resolution_y_reciprocal_ =1.0/(float) (0.01f * (M_PI/180.0f)); // 1/ angular_resolution_y


    float angle_x = atanCalc (transformedPoint[0], transformedPoint[2]); 
    float angle_y = asin(transformedPoint[1]/range);

    res.y = (angle_x* cos(angle_y) + static_cast<float> (M_PI))*angular_resolution_x_reciprocal_ - static_cast<float> (range_image_info.image_offset_x);
    res.x = (angle_y + 0.5f*static_cast<float> (M_PI))*angular_resolution_y_reciprocal_ - static_cast<float> (range_image_info.image_offset_y);
    
    return res;
}

range_image readRangeImageFile(const std::string& auditFolder){
    std::string rangeFile = auditFolder + "/rangeImage.txt";
    range_image range_image_info;
    
    std::ifstream configFile(rangeFile);
            if (configFile.is_open()) {
                std::string line;
                while (getline(configFile, line)) {
                    if(line[0] == '#' || line.empty())
                        continue;
                    std::istringstream sin(line.substr(line.find(":") + 1));
                    if (line.find("image_offset_x") != -1)
                        sin >> range_image_info.image_offset_x;    
                    else if (line.find("image_offset_y") != -1)
                        sin >> range_image_info.image_offset_y;
                    else if (line.find("axis_x") != -1)
                        sin >> range_image_info.orientation[0];
                    else if (line.find("axis_y") != -1)
                        sin >> range_image_info.orientation[1];
                    else if (line.find("axis_z") != -1)
                        sin >> range_image_info.orientation[2];
                    else if (line.find("position_x") != -1)
                        sin >> range_image_info.translation[0];
                    else if (line.find("position_y") != -1)
                        sin >> range_image_info.translation[1];
                    else if (line.find("position_z") != -1)
                        sin >> range_image_info.translation[2];
                    else if (line.find("min") != -1)
                        sin >> range_image_info.min;
                    else if (line.find("max") != -1)
                        sin >> range_image_info.max;
                    }
            }
        cv::Affine3f profilometerPose{range_image_info.orientation,range_image_info.translation};
        range_image_info.sensorPose =profilometerPose;
        return range_image_info;
}






// Goes into math point cloud

float atanCalc (float y, float x){
  if (x==0 && y==0)
    return 0;
  float ret;
  if (fabsf (x) < fabsf (y)) {
  
    ret = atan(x/y) ;
    ret = static_cast<float> (x*y > 0 ? (180)/2-ret : -(180)/2-ret);
  }
  else
     ret = atan(y/x) ;
  if (x < 0)
    ret = static_cast<float> (y < 0 ? ret-(180) : ret+(180));
  
  return (ret);
}






Quaternion ToQuaternion(const float& yaw, const float& pitch, const float& roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

EulerAngles ToEulerAngles(const Quaternion& q) {   
    EulerAngles angles;
    // ZYX Convention
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


cv::Matx33f rotation_from_euler(float roll, float pitch, float yaw){
    // roll and pitch and yaw in radian
    float su = sin(roll);
    float cu = cos(roll);
    float sv = sin(pitch);
    float cv = cos(pitch);
    float sw = sin(yaw);
    float cw = cos(yaw);
    cv::Matx33f Rot_matrix(3, 3);
    Rot_matrix(0, 0) = cv*cw;
    Rot_matrix(0, 1) = su*sv*cw - cu*sw;
    Rot_matrix(0, 2) = su*sw + cu*sv*cw;
    Rot_matrix(1, 0) = cv*sw;
    Rot_matrix(1, 1) = cu*cw + su*sv*sw;
    Rot_matrix(1, 2) = cu*sv*sw - su*cw;
    Rot_matrix(2, 0) = -sv;
    Rot_matrix(2, 1) = su*cv;
    Rot_matrix(2, 2) = cu*cv;
    return Rot_matrix;
}




 std::vector<cv::Vec3f> findClosestPointsInSparsePC(pcl::PointCloud<pcl::PointXYZ>::Ptr sparsePointCloud,
                                 const std::vector<cv::Vec3f>& selectedPoints ){

     std::vector<cv::Vec3f> closestNeighbours;                               
    
    // Octree 
    for (size_t i = 0; i < selectedPoints.size(); i++){
   
        pcl::PointXYZ searchPoint{selectedPoints.at(i)[0],selectedPoints.at(i)[1],selectedPoints.at(i)[2]};

        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud (sparsePointCloud);
        octree.addPointsFromInputCloud ();
        
        int K = 1; // closest neighbour,

        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
 
         if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
            cv::Vec3f closestPoint{(*sparsePointCloud)[ pointIdxNKNSearch[0] ].x ,
            (*sparsePointCloud)[ pointIdxNKNSearch[0] ].y,
            (*sparsePointCloud)[ pointIdxNKNSearch[0] ].z};
            closestNeighbours.emplace_back(closestPoint);            
        }
    }
    return closestNeighbours;
}


