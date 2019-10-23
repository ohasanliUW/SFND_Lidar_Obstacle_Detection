/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include "../../debug.h"
#include <chrono>
#include <string>
#include "kdtree.h"

#define CLUSTER_LOG(format, ...) \
    LOG("CLUSTER LOG: " format, ## __VA_ARGS__)
#define CLUSTER_ERROR(format, ...) \
    ERROR("CLUSTER ERROR: " format, ## __VA_ARGS__)

#define PAUSE                                           \
    do                                                  \ 
    {                                                   \
        std::cout << '\n' << "Press a key to continue...";   \
    } while (std::cin.get() != '\n');                        \

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

    //viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, window.z_min, window.z_max, 0, 0, 0, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];

  		cloud->push_back(point);

  	}
  	//cloud->width = cloud->points.size();
  	//cloud->height = 1;

  	return cloud;

}


void render3DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%3==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, node->point[2]),
                            pcl::PointXYZ(node->point[0], window.y_max, node->point[2]),
                            0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else if (depth%3==1)
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], node->point[2]),
                            pcl::PointXYZ(window.x_max, node->point[1], node->point[2]),
                            1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		} else {
			viewer->addLine(pcl::PointXYZ(node->point[0], node->point[1], window.z_min),
                            pcl::PointXYZ(node->point[0], node->point[1], window.z_max),
                            0,1,0,"line"+std::to_string(iteration));
			lowerWindow.z_max = node->point[2];
			upperWindow.z_min = node->point[2];
        }
		iteration++;

		render3DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render3DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

void
clusterHelper(const std::vector<std::vector<float>>& points,
              uint pos,
              std::vector<bool>& processed,
              KdTree* tree,
              float distanceTol,
              std::vector<int>& cluster,
              pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    processed[pos] = true;
    cluster.push_back(pos);

    std::vector<int> nearest = tree->search(points[pos], distanceTol);
            PAUSE;

    for (auto n : nearest) {
        if (!processed[n]) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr dummy{new pcl::PointCloud<pcl::PointXYZ>()};
            dummy->push_back(pcl::PointXYZ(points[n][0], points[n][1], points[n][2]));
            renderPointCloud(viewer, dummy, "dummy", Color(0,1,1));
            clusterHelper(points, n, processed, tree, distanceTol, cluster, viewer);
        }
    }
}

std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>>& points,
                 KdTree* tree,
                 float distanceTol,
                 pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    std::vector<bool> processed (points.size(), false);
	std::vector<std::vector<int>> clusters;

    for (auto i = 0; i < points.size(); i++) {
        if (!processed[i]) {
            std::vector<int> cluster;
            clusterHelper(points, i, processed, tree, distanceTol, cluster, viewer);
            clusters.push_back(cluster);
        }
    }

	return clusters;
}

int main ()
{
	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min = -10;
  	window.z_max =  10;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);
#if 0
	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7, 0}, {-6.3, 7.1, 0.2}, {-6.3,8.4, 0}, {-6.1, 8.6, -0.2}, {-5.2,7.1, 0.1}, {-5, 7, 0.2}, {-5.7,6.3, 2.5}, {7.2,6.1, -2}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

  	int it = 0;
  	render3DTree(tree->root,viewer,window, it);
  
  	std::cout << "Test Search" << std::endl;
  	std::vector<int> nearby = tree->search({-6,7},3.0);
  	for(int index : nearby)
      std::cout << index << ",";
  	std::cout << std::endl;
  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 5.0, viewer);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
#endif

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
}
