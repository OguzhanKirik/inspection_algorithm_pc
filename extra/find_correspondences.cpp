 /*
            //##################
            // Show Results of matching
            std::string cloudPath =  params.auditFolder + cv::format("/dense_acquisition/PointCloud_1.pcd");
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
                throw std::runtime_error("Couldn't read cloud pcd file \n");}

            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->setBackgroundColor (0, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
            viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud_test");
            
            std::vector<pcl::PointXYZ> add_lines;
            for (size_t i = 0; i < detectedHoles_matched.size(); i++){
                if(detectedHoles_matched.at(i).misPrediction == false &&
                 detectedHoles_matched.at(i).differentCluster == false && 
                 detectedHoles_matched.at(i).unpiloted == false){
                    pcl::PointXYZ p_detected{detectedHoles_matched.at(i).position_aligned[0],detectedHoles_matched.at(i).position_aligned[1],detectedHoles_matched.at(i).position_aligned[2]};
                    pcl::PointXYZ p_cad{detectedHoles_matched.at(i).position[0],detectedHoles_matched.at(i).position[1],detectedHoles_matched.at(i).position[2]};
                    add_lines.emplace_back(p_detected);
                    add_lines.emplace_back(p_cad);
                }
            }
            for (int i = 1; i < add_lines.size(); i+=2){
                viewer->addLine(add_lines.at(i-1),add_lines.at(i),90.0,4.0,90.0,cv::format("line_%d",i));
            }
        
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_different_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < detectedHoles_matched.size(); i++){
                if(detectedHoles_matched.at(i).differentCluster == true ){
                    pcl::PointXYZ p{detectedHoles_matched.at(i).position_aligned[0],detectedHoles_matched.at(i).position_aligned[1],detectedHoles_matched.at(i).position_aligned[2]};
                    cloud_different_cluster->points.emplace_back(p);
            }
            }
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_different(cloud_different_cluster, 12, 0, 255);
            viewer->addPointCloud<pcl::PointXYZ> (cloud_different_cluster, color_different, "cloud_different");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_different");


            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_misprediction (new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < detectedHoles_matched.size(); i++){
                if(detectedHoles_matched.at(i).misPrediction == true ){
                    pcl::PointXYZ p{detectedHoles_matched.at(i).position_aligned[0],detectedHoles_matched.at(i).position_aligned[1],detectedHoles_matched.at(i).position_aligned[2]};
                    cloud_misprediction->points.emplace_back(p);
            }
            }
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_misPrediction(cloud_misprediction, 255, 0, 0);
            viewer->addPointCloud<pcl::PointXYZ> (cloud_misprediction, color_misPrediction, "color_misPrediction");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "color_misPrediction");
            while (!viewer->wasStopped ()) {viewer->spinOnce ();}   
            
            // Show Results of matching
            //##################
            */

