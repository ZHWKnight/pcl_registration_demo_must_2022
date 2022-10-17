#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
{
  if (event.getKeySym() == "space" && event.keyDown())
    next_iteration = true;
}

int main(int argc, char *argv[])
{
  // The point clouds we will be using
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZ>); // Original point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr(new pcl::PointCloud<pcl::PointXYZ>);  // Transformed point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>); // ICP output point cloud

  // Checking program arguments
  if (argc < 2)
  {
    printf("Usage :\n");
    printf("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR("Provide one ply file.\n");
    return (-1);
  }

  int iterations = 1; // Default number of ICP iterations
  if (argc > 2)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi(argv[2]);
    if (iterations < 1)
    {
      PCL_ERROR("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }

  pcl::console::TicToc time;
  time.tic();
  if (pcl::io::loadPLYFile(argv[1], *cloud_ori) < 0)
  {
    PCL_ERROR("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_ori->size() << " points) in " << time.toc() << " ms\n"
            << std::endl;

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

  // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  double theta = M_PI / 6; // The angle of rotation in radians (30°)
  transformation_matrix(0, 0) = std::cos(theta);
  transformation_matrix(0, 1) = -sin(theta);
  transformation_matrix(1, 0) = sin(theta);
  transformation_matrix(1, 1) = std::cos(theta);

  // A translation on Z axis (0.8 meters)
  transformation_matrix(2, 3) = 0.8;

  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_ori -> cloud_tr" << std::endl;
  print4x4Matrix(transformation_matrix);

  // Executing the transformation
  pcl::transformPointCloud(*cloud_ori, *cloud_tr, transformation_matrix);

  // We backup cloud_tr into cloud_out for comparison
  *cloud_out = *cloud_tr;

  // The Iterative Closest Point algorithm
  time.tic();
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_out);
  icp.setInputTarget(cloud_ori);
  icp.setMaximumIterations(iterations); // We set this variable to given number (default = 1) for align () function
  icp.align(*cloud_out);
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

  if (icp.hasConverged())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_out -> cloud_ori" << std::endl;
    transformation_matrix = icp.getFinalTransformation().cast<double>();
    print4x4Matrix(transformation_matrix);
  }
  else
  {
    PCL_ERROR("\nICP has not converged.\n");
    return (-1);
  }

  // Visualization
  pcl::visualization::PCLVisualizer viewer("ICP DEMO for Presentation in Fundamentals of Artificial Intelligence (2022) MUST");

  // Create two vertically separated viewports
  int vp_1(0);
  int vp_2(1);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, vp_1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, vp_2);

  // The color we will be using
  float bg_gray_level = 0.0;                  // Black
  float txt_gray_level = 1.0 - bg_gray_level; // White

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(
      cloud_ori, (int)255 * txt_gray_level, (int)255 * txt_gray_level, (int)255 * txt_gray_level);
  viewer.addPointCloud(cloud_ori, cloud_in_color_h, "cloud_ori_vp_1", vp_1);
  viewer.addPointCloud(cloud_ori, cloud_in_color_h, "cloud_ori_vp_2", vp_2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(cloud_tr, 20, 180, 20);
  viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_vp_1", vp_1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(cloud_out, 180, 20, 20);
  viewer.addPointCloud(cloud_out, cloud_icp_color_h, "cloud_icp_vp_2", vp_2);

  // Adding text descriptions in each viewport
  viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud",
                 10, 15, 16, txt_gray_level, txt_gray_level, txt_gray_level, "icp_info_1", vp_1);
  viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud",
                 10, 15, 16, txt_gray_level, txt_gray_level, txt_gray_level, "icp_info_2", vp_2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_count = "ICP iterations = " + ss.str();
  viewer.addText(iterations_count,
                 10, 60, 16, txt_gray_level, txt_gray_level, txt_gray_level, "iterations_count", vp_2);

  // Set background color
  viewer.setBackgroundColor(bg_gray_level, bg_gray_level, bg_gray_level, vp_1);
  viewer.setBackgroundColor(bg_gray_level, bg_gray_level, bg_gray_level, vp_2);

  // Set camera position and orientation
  viewer.setCameraPosition(0.0541919, 7.40154, -0.440562, -0.00350204, 0.0288117, 0.259362, 0.00659833, 0.0944561, 0.995507, 0);
  viewer.setSize(1920, 1080); // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

  // Display the visualiser
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();

    // The user pressed "space" :
    if (next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic();
      icp.align(*cloud_out);
      std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

      if (icp.hasConverged())
      {
        printf("\033[11A"); // Go up 11 lines in terminal output.
        printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
        std::cout << "\nICP transformation " << ++iterations << " : cloud_out -> cloud_ori" << std::endl;
        transformation_matrix *= icp.getFinalTransformation().cast<double>(); // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix(transformation_matrix);                                // Print the transformation between original pose and current pose

        ss.str("");
        ss << iterations;
        std::string iterations_count = "ICP iterations = " + ss.str();
        viewer.updateText(iterations_count, 10, 60, 16, txt_gray_level, txt_gray_level, txt_gray_level, "iterations_count");
        viewer.updatePointCloud(cloud_out, cloud_icp_color_h, "cloud_icp_vp_2");
      }
      else
      {
        PCL_ERROR("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
  }
  return (0);
}
