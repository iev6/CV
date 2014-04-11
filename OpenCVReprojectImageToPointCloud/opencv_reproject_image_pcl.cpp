/**
*       @file opencv_reproject_image_pcl.cpp
*       @brief Identify Planes using Kinect Data Input using OpenCV and PCL.
*       @author Giridhur,Giridar
*       @date 11/04/2014
*/

/*
        opencv_reproject_image_pcl.cpp - Reproject an image to Point Cloud
        using OpenCV and PCL. The program receives from command line an 
        rgb-image (left image of stereo rig) a disparity-image (obtained with 
        some stereo matching algorithm) and the matrix Q (Generated at calibration 
        stage). It displays the 3D reconstruction of the scene using PCL.
        and calculates planes and orientation in the image
        
        This application is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 2.1 of the License, or (at your option) any later version.
        
        This application is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.
        
        You should have received a copy of the GNU Lesser General Public
        License along with this application; if not, write to the Free Software
        Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

//Kids, using this kind of #define is quite dirty, don't do it at home!!
#define CUSTOM_REPROJECT
/*** To understand the CUSTOM_REPROJECT code, please read Chapter 12 of the book
  Learning OpenCV: Computer Vision with the OpenCV Library. (Page 435) 
  I am using it because cv::reprojectImageTo3D is not giving me the expected
  results for some reason.
  
  If you want to use this program with cv::reprojectImageTo3D please comment
  the previous #define CUSTOM_REPROJECT and recompile.
    
***/

struct imagedata
{ int x,y; } img_orig_struct[76800];
long long int aux[10000000]={0};
int ctr_in_aux(unsigned long long int ctr,unsigned long long int ctr_aux)
  {
	   long long int i=0;
	   for(i=0;i<=ctr_aux;i++)
	   if (aux[i]==ctr) return 1;
	   
	   return 0;
   } //function to check ctr validity
   
//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main( int argc, char** argv )
{
 //Check arguments
  if (argc != 4)
  {
    std::cerr << "Usage: " << argv[0] << " <rgb-image-filename> <disparity-image-filename> <path-to-Q-matrix>" << std::endl;
    return 1;
  }

  //Load Matrix Q
  cv::FileStorage fs(argv[3], cv::FileStorage::READ);
  cv::Mat Q;
  
  fs["Q"] >> Q;
  
  //If size of Q is not 4x4 exit
  if (Q.cols != 4 || Q.rows != 4)
  {
    std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
    return 1;
  }

#ifdef CUSTOM_REPROJECT
  //Get the interesting parameters from Q
  double Q03, Q13, Q23, Q32, Q33;
  Q03 = Q.at<double>(0,3);
  Q13 = Q.at<double>(1,3);
  Q23 = Q.at<double>(2,3);
  Q32 = Q.at<double>(3,2);
  Q33 = Q.at<double>(3,3);
  
  std::cout << "Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;
  
#endif  
  
  
  std::cout << "Read matrix in file " << argv[3] << std::endl;

  //Show the values inside Q (for debug purposes)
  /*
  for (int y = 0; y < Q.rows; y++)
  {
    const double* Qy = Q.ptr<double>(y);
    for (int x = 0; x < Q.cols; x++)
    {
      std::cout << "Q(" << x << "," << y << ") = " << Qy[x] << std::endl;
    }
  }
  */
  
  //Load rgb-image
  
  cv::Mat img_rgb = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  
  if (img_rgb.data == NULL)
  {
    std::cerr << "ERROR: Could not read rgb-image: " << argv[1] << std::endl;
    return 1;
  }
  
  //Load disparity image
  cv::Mat img_disparity = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
  if (img_disparity.data == NULL)
  {
    std::cerr << "ERROR: Could not read disparity-image: " << argv[2] << std::endl;
    return 1;
  }
  
  //Both images must be same size
  if (img_rgb.size() != img_disparity.size())
  {
    std::cerr << "ERROR: rgb-image and disparity-image have different sizes " << std::endl;
    return 1;
  }
  
  //Show both images (for debug purposes)
  cv::namedWindow("rgb-image");
  cv::namedWindow("disparity-image");
  cv::imshow("rbg-image", img_rgb);
  cv::imshow("disparity-image", img_disparity);
  std::cout << "Press a key to continue..." << std::endl;
  cv::waitKey(0);
  cv::destroyWindow("rgb-image");
  cv::destroyWindow("disparity-image");
  
#ifndef CUSTOM_REPROJECT
  //Create matrix that will contain 3D corrdinates of each pixel
  cv::Mat recons3D(img_disparity.size(), CV_32FC3);
  
  //Reproject image to 3D
  std::cout << "Reprojecting image to 3D..." << std::endl;
  cv::reprojectImageTo3D( img_disparity, recons3D, Q, false, CV_32F );
#endif  
  //Create point cloud and fill it
  std::cout << "Creating Point Cloud..." <<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  double px, py, pz;
  uchar pr, pg, pb;
  unsigned long long int ctr=1,ctr_aux=0;
  for (int i = 0; i < img_rgb.rows; i++)
  {
    uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
#ifdef CUSTOM_REPROJECT
    uchar* disp_ptr = img_disparity.ptr<uchar>(i);
#else
    double* recons_ptr = recons3D.ptr<double>(i);
#endif
    for (int j = 0; j < img_rgb.cols; j++)
    {
      //Get 3D coordinat1340974es
#ifdef CUSTOM_REPROJECT
      //
      uchar d = disp_ptr[j];
      //
      if ( d == 0 ) {  aux[ctr_aux]=ctr; ctr_aux++; continue; }//Discard bad pixels
      double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
      px = static_cast<double>(j) + Q03;
      py = static_cast<double>(i) + Q13;
      
      pz = Q23;
      
      px = px/pw;
      py = py/pw;
      pz = pz/pw;
     
#else
      px = recons_ptr[3*j];
      py = recons_ptr[3*j+1];
      pz = recons_ptr[3*j+2];
#endif
      
      //Get RGB info
      pb = rgb_ptr[3*j];
      pg = rgb_ptr[3*j+1];
      pr = rgb_ptr[3*j+2];
      
      //Insert info into point cloud structure
      pcl::PointXYZRGB point;
      point.x = px;
      point.y = py;
      point.z = pz;
      uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
              static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
      //
      img_orig_struct[ctr].x=j;
      img_orig_struct[ctr].y=i;
      
      ctr++;
    }
  }
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;
    std::cout<<"argo fuk yourself"<<std::endl;
  char as;
  std::cin>>as;
  std::cout<<endl;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_that_we_need = *point_cloud_ptr;
    pcl::io::savePCDFileASCII ("test_pcd.pcd",cloud_that_we_need) ;
     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);
  pcl::PointCloud<pcl::Normal> cloud_that_we_need_2 = *cloud_normals1;
  pcl::io::savePCDFileASCII ("test_pcd_normals.pcd",cloud_that_we_need_2) ;

//Start of trial Code
cv::Mat img_nrmls=cv::Mat::zeros(img_rgb.rows,img_rgb.cols, CV_8UC3);
pcl::PointNormal a;
pcl::PointCloud<pcl::Normal>::iterator b1;
pcl::PointCloud<pcl::PointXYZRGB>::iterator b2;
b1 = cloud_normals1->begin();
b2=point_cloud_ptr->begin();
uint16_t i;

cv::Mat_<cv::Vec3b>::iterator it= img_nrmls.begin<cv::Vec3b>(),it_end=img_nrmls.end<cv::Vec3b>();
/*  for (i = 0; i < cloud_normals1->points.size (); i++)
      {
        double n_x = cloud_normals1->points[i].normal_x;
        double n_y = cloud_normals1->points[i].normal_y;
        double n_z = cloud_normals1>points[i].normal_z;
        n_x= (n_x+1)*128;
        Pointer[position].b= (int) n_x= (n_x+1)*128;
       }
       */
       ctr=0; 
       int x,y;
     /*
        while( b2!=point_cloud_ptr->end())
         {     
			// if (ctr_in_aux(ctr,ctr_aux)) { ctr++;continue;}
			// else 
			 {
				 y=img_orig_struct[ctr].x;
			  x=img_orig_struct[ctr].y;   
			    img_nrmls.at<cv::Vec3b>(x,y)[0]=b2->b;
			    img_nrmls.at<cv::Vec3b>(x,y)[1]=b2->g;
                img_nrmls.at<cv::Vec3b>(x,y)[2]=b2->r;			    
			    ctr++;
			  }
			    b2++;
			  
			}
			
			*/
//Visualizing normals
while(b1!=cloud_normals1->end())
 {
	 
	 if (pcl::isFinite<pcl::Normal>( *b1 ))
                {
					
				
			 	y=img_orig_struct[ctr].x;
			 	x=img_orig_struct[ctr].y;   

				img_nrmls.at<cv::Vec3b>(x,y)=cv::Vec3b((int)(128*(1+(b1->normal_x))),(int)(128*(1+(b1->normal_y))),(int)(128*(1+(b1->normal_z))));
			     
			   			     
			     }
                	
                	else
                	{
				
					 y=img_orig_struct[ctr].x;
			 		 x=img_orig_struct[ctr].y;   
			 		 img_nrmls.at<cv::Vec3b>(x,y) = cv::Vec3b( 255,255,255);
			         }
      b1++;
      ctr++;
 }
 
 
 
 
 
cv::namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );
 //bool imwrite(const string& filename, InputArray img, const vector<int>& params=vector<int>() )¶
 cv::imwrite( "img_nrmls.jpg",img_nrmls);
 cv::imshow("Gray image", img_nrmls);
 cvWaitKey(0);
//End of trial code
// pcl::PointXYZRGB *point = point_cloud_ptr<pcl::PointXYZRGB>.at(0,0);
// std::cout<<"argo fuck yourself";
  // cloud_normals1 holds our normals 
  //Create visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
 //viewer = normalsVis(point_cloud_ptr, cloud_normals1);
  // 
  //Main loop
 /* while ( !viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  */
  return 0;
}
