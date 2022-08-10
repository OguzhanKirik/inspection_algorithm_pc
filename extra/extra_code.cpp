
            /*
        //pcl::transformPointCloud(*pilot_holes_locations_detected, *pilot_holes_locations_cad_transformed, icp.getFinalTransformation());
        
        std::string cloudPath =  "/home/oguz/vs_code/dense_16_12/dense_acquisition/PointCloud_1.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");}

        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_main(cloud, 144, 144, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_Targets(pilot_holes_locations_cad, 250, 144, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_Targets_converted(pilot_holes_locations_cad_transformed, 12, 120, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_detected(pilot_holes_locations_detected, 144, 144, 255);

        viewer->addPointCloud<pcl::PointXYZ> (cloud, color_main, "color_main");
        viewer->addPointCloud<pcl::PointXYZ> (pilot_holes_locations_cad, color_Targets, "color_Targets");
        viewer->addPointCloud<pcl::PointXYZ> (pilot_holes_locations_cad_transformed, color_Targets_converted, "color_Targets_converted");
        viewer->addPointCloud<pcl::PointXYZ> (pilot_holes_locations_detected, color_detected, "color_detected");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "color_main");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "color_Targets");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "color_Targets_converted");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "color_detected");

            while (!viewer->wasStopped ()) {viewer->spinOnce ();}   

        */




       /*
        std::vector<pcl::PointXYZ> add_lines;
         std::vector<drillTarget> drillTargets_CAD_audit_set = setParameters(drillTargets_CAD_audit);
        for (size_t i = 0; i < drillTargets_CAD_audit_set.size(); i++){

            cv::Vec3f position_{drillTargets_CAD_audit_set.at(i).position[0],drillTargets_CAD_audit_set.at(i).position[1],(drillTargets_CAD_audit_set.at(i).position[2]+10)};
            cv::Vec3f position_shift{0,0,-25};
            cv::Vec3f position_second = drillTargets_CAD_audit_set.at(i).rotation_matrix * position_shift;
            cv::Vec3f position_second_= position_ + position_second;
            pcl::PointXYZ p_second{position_second_[0],position_second_[1],position_second_[2]};
            pcl::PointXYZ p_first{drillTargets_CAD_audit.at(i).position[0],drillTargets_CAD_audit.at(i).position[1],drillTargets_CAD_audit.at(i).position[2]};
            add_lines.emplace_back(p_first);
            add_lines.emplace_back(p_second);
            
        }

        for (int i = 1; i < add_lines.size(); i+=2){
                viewer->addArrow(add_lines.at(i-1),add_lines.at(i),90.0,4.0,90.0,cv::format("line_%d",i));
        }

        while (!viewer->wasStopped ()) {viewer->spinOnce ();}   
	*/
	


/*
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
                detectedHole.position_aligned = position;  
                if(BBs.at(k).isFullSized()) 
                     detectedHole.fullSized = true;
                 else    
                      detectedHole.piloted = true;
            
                 detectedHoles.emplace_back(detectedHole);
            }
            
        }   
        return detectedHoles; 
}
*/



 std::string pathToHoles=  "/home/oguz/vs_code/inspectDrillTargets/extra/quar/targets.txt";
        std::vector<drillTarget> drillTargets_CAD_audit = readPosesFromFile(pathToHoles); 
           
    
        std::vector<pcl::PointXYZ> add_lines;
         std::vector<drillTarget> drillTargets_CAD_audit_set = setParameters(drillTargets_CAD_audit);
        for (size_t i = 0; i < drillTargets_CAD_audit_set.size(); i++){

            cv::Vec3f position_{drillTargets_CAD_audit_set.at(i).position[0],drillTargets_CAD_audit_set.at(i).position[1],(drillTargets_CAD_audit_set.at(i).position[2]+10)};
            cv::Vec3f position_shift{0,0,-25};
            cv::Vec3f position_second = drillTargets_CAD_audit_set.at(i).rotation_matrix * position_shift;
            cv::Vec3f position_second_= position_ + position_second;
            pcl::PointXYZ p_second{position_second_[0],position_second_[1],position_second_[2]};
            pcl::PointXYZ p_first{drillTargets_CAD_audit.at(i).position[0],drillTargets_CAD_audit.at(i).position[1],drillTargets_CAD_audit.at(i).position[2]};
            add_lines.emplace_back(p_first);
            add_lines.emplace_back(p_second);

            Eigen::Vector3f p_dif = p_first.getArray3fMap() - p_second.getArray3fMap();
            p_dif.normalize();
            cv::Vec3f normals{p_dif.x(),p_dif.y(),p_dif.z()};
            drillTargets_CAD_audit_set.at(i).normals = normals;
            
        }


            std::vector<pcl::PointXYZ> add_arrows;
    for (size_t i = 0; i < drillTargets_CAD_audit_set.size(); i++){
    
        pcl::PointXYZ p1{drillTargets_CAD_audit_set.at(i).position[0] + (drillTargets_CAD_audit_set.at(i).normals[0] * -20),
       drillTargets_CAD_audit_set.at(i).position[1] + (drillTargets_CAD_audit_set.at(i).normals[1] * -20),
        drillTargets_CAD_audit_set.at(i).position[2] + (drillTargets_CAD_audit_set.at(i).normals[2] * -20)};
        pcl::PointXYZ p2{drillTargets_CAD_audit_set.at(i).position[0] + (drillTargets_CAD_audit_set.at(i).normals[0] * 20),
       drillTargets_CAD_audit_set.at(i).position[1] + (drillTargets_CAD_audit_set.at(i).normals[1] * 20),
        drillTargets_CAD_audit_set.at(i).position[2] + (drillTargets_CAD_audit_set.at(i).normals[2] * 20)};

        add_arrows.emplace_back(p1);
        add_arrows.emplace_back(p2);
        
    }
  



            std::string cloudPath = "/home/oguz/vs_code/inspectDrillTargets/extra/quar/PointCloud_0.pcd";
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
                throw std::runtime_error("Couldn't read cloud pcd file \n");}

            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->setBackgroundColor (0, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
            viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud_test");
            /*
            for (int i = 1; i < add_lines.size(); i+=2){
                viewer->addLine(add_lines.at(i-1),add_lines.at(i),90.0,4.0,90.0,cv::format("line_%d",i));
            }
            */
              for (int i = 1; i < add_arrows.size(); i+=2){
            viewer->addArrow(add_arrows.at(i-1),add_arrows.at(i),90.0,4.0,90.0,cv::format("line_%d",i));
             }
            while (!viewer->wasStopped ()) {viewer->spinOnce ();}   



    //std::string pathToHoles=  "/home/oguz/vs_code/inspectDrillTargets/extra/quar/targets.txt";
        //std::vector<drillTarget> drillTargets_CAD_audit = readPosesFromFile(pathToHoles); 
           
    /*
    std::vector<pcl::PointXYZ> add_lines;
        std::vector<drillTarget> drillTargets_CAD_audit_set = setParameters(drillTargets_CAD_audit);
    for (size_t i = 0; i < drillTargets_CAD_audit_set.size(); i++){

        cv::Vec3f position_{drillTargets_CAD_audit_set.at(i).position[0],drillTargets_CAD_audit_set.at(i).position[1],(drillTargets_CAD_audit_set.at(i).position[2]+10)};
        cv::Vec3f position_shift{0,0,-25};
        cv::Vec3f position_second = drillTargets_CAD_audit_set.at(i).rotation_matrix * position_shift;
        cv::Vec3f position_second_= position_ + position_second;
        pcl::PointXYZ p_second{position_second_[0],position_second_[1],position_second_[2]};
        pcl::PointXYZ p_first{drillTargets_CAD_audit.at(i).position[0],drillTargets_CAD_audit.at(i).position[1],drillTargets_CAD_audit.at(i).position[2]};
        add_lines.emplace_back(p_first);
        add_lines.emplace_back(p_second);

        Eigen::Vector3f p_dif = p_first.getArray3fMap() - p_second.getArray3fMap();
        p_dif.normalize();
        cv::Vec3f normals{p_dif.x(),p_dif.y(),p_dif.z()};
        drillTargets_CAD_audit_set.at(i).normals = normals;
    }


    std::vector<pcl::PointXYZ> add_arrows;
    for (size_t i = 0; i < drillTargets_CAD_audit_set.size(); i++){
    
        pcl::PointXYZ p1{drillTargets_CAD_audit_set.at(i).position[0] + (drillTargets_CAD_audit_set.at(i).normals[0] * -20),
        drillTargets_CAD_audit_set.at(i).position[1] + (drillTargets_CAD_audit_set.at(i).normals[1] * -20),
        drillTargets_CAD_audit_set.at(i).position[2] + (drillTargets_CAD_audit_set.at(i).normals[2] * -20)};
        pcl::PointXYZ p2{drillTargets_CAD_audit_set.at(i).position[0] + (drillTargets_CAD_audit_set.at(i).normals[0] * 20),
        drillTargets_CAD_audit_set.at(i).position[1] + (drillTargets_CAD_audit_set.at(i).normals[1] * 20),
        drillTargets_CAD_audit_set.at(i).position[2] + (drillTargets_CAD_audit_set.at(i).normals[2] * 20)};

        add_arrows.emplace_back(p1);
        add_arrows.emplace_back(p2);
        
    }
  

    std::string cloudPath =  params.auditFolder + cv::format("/dense_acquisition/PointCloud_1.pcd");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");}

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud_test");
    
    for (int i = 1; i < add_lines.size(); i+=2){
        viewer->addLine(add_lines.at(i-1),add_lines.at(i),90.0,4.0,90.0,cv::format("line%d",i));
    }
    
    for (int i = 1; i < add_arrows.size(); i+=2){
    viewer->addArrow(add_arrows.at(i-1),add_arrows.at(i),90.0,4.0,90.0,cv::format("line_%d",i));
    }
    
    while (!viewer->wasStopped ()) {viewer->spinOnce ();}  

    */


  /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_statistical_voxel (new pcl::PointCloud<pcl::PointXYZ>);
   // Downsampling
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud (cloud_filtered_statistical_removal_BB_filtered);
    voxel.setLeafSize (5.01f, 5.01f, 5.01f);
    voxel.filter (*cloud_filtered_statistical_voxel);
   // Downsampling
    */

    // pcl::PointCloud<pcl::PointXYZ> cloud_filtered_m;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_statistical_m_pointer(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::applyMorphologicalOperator<pcl::PointXYZ>(cloud_filtered_statistical_removal_BB_filtered,10.0f,pcl::MORPH_DILATE,*cloud_filtered_statistical_m_pointer );
   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_statistical_m_pointer(new pcl::PointCloud<pcl::PointXYZ>);





