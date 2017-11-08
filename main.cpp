// usage: ./main swan.png swan.csv

#include "graph.h"
#include "image.h"
#include "callback.h"
#include "gurobi_c++.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;
#define EPS  0.0001
bool trigger;
Mat img;
std::vector<std::vector<CvPoint>> seeds;
int brushwidth;
int nclick = 0;



//std::vector<std::pair<uint32_t, uint32_t>> master_pixels;

std::string itos(int i) {std::stringstream s; s << i; return s.str(); }

// single click on image to select seed, f? void*?
void onMouse(int event, int x, int y, int flag, void*)
{
  if  ( event == EVENT_LBUTTONDOWN )
    {
			seeds.emplace_back();
			nclick++;
			trigger = true;
      //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
			//cout << "nlick = " << nclick <<endl;
    }
  if(event == CV_EVENT_LBUTTONUP)
    {
        trigger = false;
    }
    
  if ( event == EVENT_MOUSEMOVE )
    {
			if(trigger == true)
        {
					if (nclick == 1)
	  				line(img, cvPoint(x, y), cvPoint(x, y), Scalar(255, 0, 0), 2*brushwidth, CV_AA, 0);
					if (nclick == 2)
	  				line(img, cvPoint(x, y), cvPoint(x, y), Scalar(0, 255, 0), 2*brushwidth, CV_AA, 0);
					if (nclick == 3)
	  				line(img, cvPoint(x, y), cvPoint(x, y), Scalar(0, 0, 255), 2*brushwidth, CV_AA, 0);
					if (nclick == 4)
	  				line(img, cvPoint(x, y), cvPoint(x, y), Scalar(255, 255, 0), 2*brushwidth, CV_AA, 0);
					if (nclick == 5)
	  				line(img, cvPoint(x, y), cvPoint(x, y), Scalar(255, 0, 255), 2*brushwidth, CV_AA, 0);
					if (nclick == 6)
	  				line(img, cvPoint(x, y), cvPoint(x, y), Scalar(0, 255, 255), 2*brushwidth, CV_AA, 0);
					if (nclick > 6)
	  				line(img, cvPoint(x, y), cvPoint(x, y), Scalar(0, 0, 0), 2*brushwidth, CV_AA, 0);
	  			for (int i= -brushwidth; i < brushwidth; i++)
						for (int j= -brushwidth; j < brushwidth; j++)
						{
							if (x+i >=0 & x+i < img.size().width & y+i >=0 & y+j < img.size().height)
	  						seeds.back().push_back(cvPoint(x+i, y+j));
						}
	  			//cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
					//cout << "nclick = " << nclick <<endl;
	  			imshow("Select master nodes", img);
	  			//hold on;
	}
    }    
}



// function to create gurobi model and solve
void master_problem(
    Graph& rag, ///< the graph of superpixels
    int numseg,
    std::vector<vector<Graph::vertex_descriptor>> root_nodes, ///< master nodes of all segments 
    std::vector<std::vector<Graph::vertex_descriptor>>& segments ///< the selected segments will be stored in here
)
 {
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    // Turn off display and heuristics and enable adding constraints in our callback function
    model.set(GRB_IntParam_OutputFlag, 1); // shut up
    model.set(GRB_DoubleParam_Heuristics, 0);
    model.set(GRB_DoubleParam_TimeLimit, 100.0);
    model.set(GRB_IntParam_LazyConstraints, 1);

		double avgcolor[numseg];
		
		for (int i=0; i<numseg; i++)
		{
			avgcolor[i] =0;
			for(int j=0; j < root_nodes[i].size(); j++)
				avgcolor[i] += rag[(Graph::vertex_descriptor)root_nodes[i][j]].color;
			
			avgcolor[i] = avgcolor[i]/root_nodes[i].size();
		}

    GRBLinExpr objective(0.0);
    //std::cout << "root_nodes[0]: " <<root_nodes[0]<< std::endl;
    //add RAG node as variables
    for(int i = 0; i< num_vertices(rag); ++i) 
    {
	  	for (int j=0; j<numseg; j++)
	  	{
	    	rag[(Graph::vertex_descriptor)i].var.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_"+itos(i)+"_"+itos(j)));
	    	objective += std::abs(rag[(Graph::vertex_descriptor)i].color - avgcolor[j])*rag[(Graph::vertex_descriptor)i].var[j];
	  	}
      
    }
    // create objective function
    model.setObjective(objective, GRB_MINIMIZE);

    
    std::cout << "adding initial constraints..." << std::endl;

      // add = 1 constraints for every node
    for(int i = 0; i< num_vertices(rag); ++i)  
      {
		GRBLinExpr constraints(0.0);
		for (int j=0; j<numseg; j++)
	  	{
	    	constraints += rag[(Graph::vertex_descriptor)i].var[j];
	  	}  
		model.addConstr(constraints == 1.0);
      }
    

		// set all root nodes to be = 1
		for (int itr=0; itr<root_nodes.size(); itr++)
      {
				for (int j=0; j< root_nodes[itr].size(); j++)
				 rag[(Graph::vertex_descriptor)root_nodes[itr][j]].var[itr].set(GRB_DoubleAttr_LB, 1.0);
	  	}
		model.update();

    model.write("model.lp");
    
    // set callback, rootnode is the first node for every brush
		vector<Graph::vertex_descriptor> root_node;
		for (int itr=0; itr<root_nodes.size(); itr++)
			root_node.push_back(root_nodes[itr][0]);
    myGRBCallback cb = myGRBCallback(rag, root_node);
    model.setCallback(&cb);


    // Optimize model
    std::cout << "optimizing..." << std::endl;
    model.optimize();
    
   // here ends solving gurobi, first check if all soluions are binary
	for (int i=0; i<num_vertices(rag); ++i) {
    for (int j=0; j<numseg; ++j) {
      if (std::abs(rag[(Graph::vertex_descriptor)i].var[j].get(GRB_DoubleAttr_X) - 1.0) > EPS && std::abs(rag[(Graph::vertex_descriptor)i].var[j].get(GRB_DoubleAttr_X) - 0) > EPS)
        std::cout << "Found fractional solution at node " << i <<" equals " <<rag[(Graph::vertex_descriptor)i].var[j].get(GRB_DoubleAttr_X)<<std::endl;
    }
   }
	
	// output segment.png
   for (int i=0; i<num_vertices(rag); ++i) {
    for (int j=0; j<numseg; ++j) {
      if (std::abs(rag[(Graph::vertex_descriptor)i].var[j].get(GRB_DoubleAttr_X) - 1.0) < EPS)
        segments[j].push_back((Graph::vertex_descriptor)i);
    }
   }
 }

 
 
int main(int argc, char** argv) {
    if (argc != 4)
    {
        std::cout << "Usage: program input.png input.csv" << std::endl;
        return 1;
    }
    //Image image("swan.png", "swan.csv");
    Image image(argv[1], argv[2]);
		
		brushwidth = atoi(argv[3]);


    img = imread("superpixels_avgcolor.png");
    namedWindow("Select master nodes");
    setMouseCallback("Select master nodes", onMouse, NULL);
    imshow("Select master nodes", img);
		
    waitKey(0);
    cvDestroyWindow("Select master nodes");
    
    // master_nodes records the superpixel of seeds.
    std::vector<vector<Graph::vertex_descriptor>> master_nodes;
    for(int i=0; i < seeds.size(); i++)
		{
			master_nodes.emplace_back();
			auto seed = seeds[i];
 			for(int j=0; j < seed.size(); j++)
    	{
        Graph::vertex_descriptor superpixel = image.pixelToSuperpixel(seed[j].x, seed[j].y);
        if (std::find(master_nodes[i].begin(), master_nodes[i].end(), superpixel) == master_nodes[i].end()) // no duplicates
            master_nodes[i].push_back(superpixel);
      }
    }

	for (int i=0; i<master_nodes.size(); i++)
		std::cout << "Root node " << i <<": "<<master_nodes[i][0]<<std::endl;

    Graph rag = image.graph();
    
    size_t n = num_vertices(rag);
    
    // segments is the final segmentation, it's a set that contains 
    std::vector<std::vector<Graph::vertex_descriptor>> segments(master_nodes.size());
    
    // gurobi model and solve
    master_problem(rag, master_nodes.size(),master_nodes, segments);

		vector<Graph::vertex_descriptor> master_node;
		for (int i=0; i<master_nodes.size(); i++)
			master_node.push_back(master_nodes[i][0]);
    image.writeSegments(master_node, segments, rag);
	std::cout << "write segments.png sucess 2." << std::endl;
	return 0;
}

