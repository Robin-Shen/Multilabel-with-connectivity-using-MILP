// usage: ./main swan.png swan.csv brushwidth timelimit lambda

#include "graph.h"
#include "image.h"
#include "callback.h"
#include "gurobi_c++.h"
#include "l0_gradient_minimization.h"
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
	  			imshow("nregion", img);
	  			//hold on;
	}
    }    
}

// function to create gurobi model and solve
void master_problem(
    Graph& rag, ///< the graph of superpixels
    int numseg,
    std::vector<vector<Graph::vertex_descriptor>> root_nodes, ///< master nodes of all segments 
    std::vector<std::vector<Graph::vertex_descriptor>>& segments, ///< the l0 heuristic segments
    std::vector<std::vector<std::vector<Graph::vertex_descriptor>>>& final_segments, ///< the final 4 segments will be stored in here
    double tlimit,
    double lambda
)
 {
    int l=0;
    std::vector<GRBEnv> envs(4);
    std::vector<GRBModel> model;
     
    for(auto it = envs.begin(); it!=envs.end();++it) {
        auto env = *it;
        model.emplace_back(env);
    }

    
    // Turn off display and heuristics and enable adding constraints in our callback function
    for (l=0; l< 4; l++)
    {
      model[l].set(GRB_IntParam_OutputFlag, 1); 
      model[l].set(GRB_DoubleParam_TimeLimit, tlimit);
      model[l].set(GRB_IntParam_LazyConstraints, 1);
    }
    
    
    double avgcolor[numseg];
    for (int i=0; i<numseg; i++)
	{
	  avgcolor[i] =0;
	  for(int j=0; j < root_nodes[i].size(); j++)
	    avgcolor[i] += rag[(Graph::vertex_descriptor)root_nodes[i][j]].color;
	  avgcolor[i] = avgcolor[i]/root_nodes[i].size();
	}

    std::vector <GRBLinExpr> objective(4);
    //std::cout << "root_nodes[0]: " <<root_nodes[0]<< std::endl;
    
    //add RAG node as variables
    for (l=0; l<4; l++)
    {
      objective[l] = 0.0;
      for(int i = 0; i< num_vertices(rag); ++i) 
	  for (int j=0; j<numseg; j++)
	  {
	    rag[(Graph::vertex_descriptor)i].var[l].push_back(model[l].addVar(0.0, 1.0, 0.0, GRB_BINARY, "x^"+itos(l)+"_"+itos(i)+"_"+itos(j)));
	    objective[l] += std::abs(rag[(Graph::vertex_descriptor)i].color - avgcolor[j])*rag[(Graph::vertex_descriptor)i].var[l][j];
	  }    
    }
    
    std::vector<std::vector<GRBVar>> x_p, x_n;
    
    // for problem 1-3, add pairwise term
    for (l=1; l<4; l++)
    {
      x_p.emplace_back(), x_n.emplace_back();    
      for(int i = 0; i< num_vertices(rag); ++i) 
	      for (int j=0; j<numseg; j++)
	      {
	        x_p[l-1].push_back(model[l].addVar(0.0, 1.0, 0.0, GRB_BINARY, "xp^"+itos(l)+"_"+itos(i)+"_"+itos(j)));
	        x_n[l-1].push_back(model[l].addVar(0.0, 1.0, 0.0, GRB_BINARY, "xn^"+itos(l)+"_"+itos(i)+"_"+itos(j)));
	      }    
    }
    
    
    // loop over edge sets to add objective
    graph_traits<Graph>::edge_iterator ei, ei_end;
    for (l=1; l<4; l++)
    {
      for (tie(ei, ei_end) = edges(rag); ei != ei_end; ++ei)
      {
        int s = source(*ei, rag);
        int t = target(*ei, rag);
        for (int j=0; j<numseg; j++)
    	        {
    	          objective[l] += lambda*(x_p[l-1][numseg*s+j] + x_n[l-1][numseg*t+j]);
    	        }
      }
    }
    
    
    // create objective function
    for (l=0; l<4; l++)
    {
      model[l].setObjective(objective[l], GRB_MINIMIZE);
    }
    
    // add = 1 constraints for every node
    for (l=0; l<4; l++)
    {
      for(int i = 0; i< num_vertices(rag); ++i)  
      {
	GRBLinExpr constraints(0.0);
	for (int j=0; j<numseg; j++)
	  {
	    constraints += rag[(Graph::vertex_descriptor)i].var[l][j];
	  }  
	model[l].addConstr(constraints == 1.0);
      }
    }



    // add absolute constraints for problem 1-3
    for (l=1; l<4; l++)
    {
      for (tie(ei, ei_end) = edges(rag); ei != ei_end; ++ei)
      {
        int s = source(*ei, rag);
        int t = target(*ei, rag);
        for (int j=0; j<numseg; j++)
    	        {
    	          GRBLinExpr left(0.0), right(0.0);
    	          left += rag[(Graph::vertex_descriptor)s].var[l][j] - rag[(Graph::vertex_descriptor)t].var[l][j];
    	          right += x_p[l-1][numseg*s+j] - x_n[l-1][numseg*t+j];
    	          model[l].addConstr(left == right);
    	        }
      }
    }
    


    // set all root nodes to be = 1
    for (l=0; l<4; l++)
    {
      for (int itr=0; itr<root_nodes.size(); itr++)
      {
	for (int j=0; j< root_nodes[itr].size(); j++)
	  rag[(Graph::vertex_descriptor)root_nodes[itr][j]].var[l][itr].set(GRB_DoubleAttr_LB, 1.0);
      }
      model[l].update();
    }
    

    
    // read solution from l0 heuristic
    for (l=0; l<4; l++)
    {
      for (int i=0; i<numseg; i++)
      {
	for(int j=0; j<segments[i].size(); ++j) 
	  {
	    int k = segments[i][j];
	    rag[(Graph::vertex_descriptor)k].var[l][i].set(GRB_DoubleAttr_Start, 1.0); 
	  }
      
      }
    }
    
    model[0].write("model0.lp");
    model[1].write("model1.lp");
    model[2].write("model2.lp");
    model[3].write("model3.lp");
    
    
    // set callback, rootnode is the first node for every brush
    vector<Graph::vertex_descriptor> root_node;
    for (int itr=0; itr<root_nodes.size(); itr++)
    {
      root_node.push_back(root_nodes[itr][0]);
      
    }
    
    //std::vector<myGRBCallback> cbs(4);
    
    
    std::vector<myGRBCallback> cbs;
    for (int l=0; l<3; l++)
    {
      cbs.push_back(myGRBCallback(rag, root_node, l));
      model[l].setCallback(&(cbs[l]));
      model[l].optimize();
    }

    // model 3 has no callback
    model[3].optimize();
    
    /* here ends solving gurobi, first check if LP's all soluions are binary
    for (int i=0; i<num_vertices(rag); ++i) {
      for (int j=0; j<numseg; ++j) {
	if (std::abs(rag[(Graph::vertex_descriptor)i].var[3][j].get(GRB_DoubleAttr_X) - 1.0) > EPS && std::abs(rag[(Graph::vertex_descriptor)i].var[3][j].get(GRB_DoubleAttr_X) - 0) > EPS)
	  std::cout << "Found fractional solution at node " << i <<" equals " <<rag[(Graph::vertex_descriptor)i].var[3][j].get(GRB_DoubleAttr_X)<<std::endl;
    }
   }
    */
    
   // initialize final_segments[l]
   for (l=0; l<4; l++)
    {
      for (int j=0; j<numseg; ++j)
	final_segments[l].emplace_back();
    }
    
   // output segment.png
   for (l=0; l<4; l++)
    {
      for (int i=0; i<num_vertices(rag); ++i) {
	for (int j=0; j<numseg; ++j) {
	  if (std::abs(rag[(Graph::vertex_descriptor)i].var[l][j].get(GRB_DoubleAttr_X) - 1.0) < EPS)
	    final_segments[l][j].push_back((Graph::vertex_descriptor)i);
	 }
      }
    }
    
 }


 
int main(int argc, char** argv) {
    if (argc != 6)
    {
        std::cout << "Usage: program input.png input.csv brushwidth timelimit lambda" << std::endl;
        return 1;
    }

    //Image image("swan.png", "swan.csv");
    Image image(argv[1], argv[2]);
		
    brushwidth = atoi(argv[3]);
    double tlimit = atof(argv[4]);
    double lambda = atof(argv[5]);

    img = imread("superpixels.png");
    namedWindow("nregion");
    setMouseCallback("nregion", onMouse, NULL);
    imshow("nregion", img);
		cv::imwrite( "brushes.png", img);
    waitKey(0);
    //cvDestroyWindow("nregion");
    
    // master_nodes records the superpixel of seeds.
    std::vector<std::vector<Graph::vertex_descriptor>> master_nodes;
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
    
    std::vector<std::vector<Graph::vertex_descriptor>> segments(master_nodes.size());
#if 1
    l0_gradient_minimization(rag, master_nodes, 0.1);     

    for (int i=0; i<num_vertices(rag); ++i) {
     for (int j=0; j<segments.size(); ++j) {
      if( rag[(typename Graph::vertex_descriptor)i].merge_label == j) {
        segments[j].push_back((Graph::vertex_descriptor)i);
      }
    }
   }
#endif

    vector<Graph::vertex_descriptor> master_node;
    for (int i=0; i<master_nodes.size(); i++)
	master_node.push_back(master_nodes[i][0]);
    
    // output l0 results
    cv::Mat output = image.writeSegments(master_node, segments, rag);
    output.copyTo(img);
    imshow("lo_heuristic reluts", img);
    
    std::vector<std::vector<std::vector<Graph::vertex_descriptor>>> final_segments(4);
    // gurobi model and solve
    master_problem(rag, master_nodes.size(),master_nodes, segments, final_segments, tlimit, lambda);

    

    output = image.writeSegments(master_node, final_segments[0], rag);
    output.copyTo(img);
    imshow("ILP results 1", img);
    
    output = image.writeSegments(master_node, final_segments[1], rag);
    output.copyTo(img);
    imshow("ILP results 2", img);
    
    output = image.writeSegments(master_node, final_segments[2], rag);
    output.copyTo(img);
    imshow("ILP results 3", img);
    
    output = image.writeSegments(master_node, final_segments[3], rag);
    output.copyTo(img);
    imshow("ILP results 4", img);
    
    waitKey(0);
    return 0;

}

