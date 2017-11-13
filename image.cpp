//#include <vl/slic.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>
#include "graph.h"
#include "image.h"
    
#include <iostream>     // cout, endl
#include <fstream>      // fstream
#include <vector>
#include <string>
#include <algorithm>    // copy
#include <iterator>     // ostream_operator
#include <boost/tokenizer.hpp>
#include <stdlib.h>
#include <algorithm> 

Image::Image(std::string png_file, std::string csv_file)
{
    cv::Mat pngimage = cv::imread( png_file, CV_LOAD_IMAGE_GRAYSCALE ); 
    cv::Mat pngimage3 = cv::imread( png_file, CV_LOAD_IMAGE_COLOR ); 
    width = pngimage.size().width;
    height = pngimage.size().height;
    unsigned int imagesize =  width * height;
    
    float* image = new float[imagesize];
    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            image[x + y * width] = pngimage.at<uchar>(y, x) / 255.0; // norm to floats from 0 to 1
        }
    }
    
    
    // segmentation is a list of superpixel labels 
    
    // use the class var

 // check if no superpixel is provided, we will use the pixel image directly
    if (csv_file.compare("null") == 0)
    {
      segmentation.resize(imagesize);
      for (int i=0; i<imagesize; i++)
	segmentation[i] = i;
      
      std::cout << "No superpixels provided!!!!!!!!" << std::endl;
    }
    //else, we read the superpixel file
    else
    {  

	    using namespace std;
	    using namespace boost;
	    string data(csv_file);

	    ifstream in(data.c_str());
	    if (!in.is_open()) exit(1);

	    typedef tokenizer <escaped_list_separator<char>> Tokenizer;
	    vector< string > vec;
	    vec.clear();
	    string line;

	    while (getline(in,line))
	    {
		Tokenizer tok(line);
		for (Tokenizer::iterator it(tok.begin()), end(tok.end()); it != end; ++it)
		{
		    vec.push_back((*it));
		} 
	    }
	    segmentation.resize(vec.size());
	    std::transform (vec.begin(), vec.end(), segmentation.begin(), [](string s) -> int {return atoi(s.c_str());});
    }
    #if 0
    for(auto it = segmentation.begin(); it != segmentation.end(); ++it) {
        std::cout << *it << std::endl;
    }

    #endif

    // relabel the segmentation by -1, if using my csv
    #if 0
	for (unsigned int x = 0; x < width; ++x)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            segmentation[x + y * width] -= 1;
        }
    }
    #endif

    superpixelcount = 0;
    for (size_t i = 0; i < imagesize; ++i)
    {
        if (superpixelcount < segmentation[i])
        {
            superpixelcount = segmentation[i];
        }
    }
    superpixelcount++;// because we start with 0
    std::cout << "Generated " << superpixelcount << " superpixels." << std::endl;
   // relabel superpixels 

	
    avgcolor.resize(superpixelcount, 0.0);
    std::vector<unsigned int> numpixels(superpixelcount, 0);
    
	

    for (unsigned int x = 0; x < width; ++x)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            avgcolor[segmentation[x + y * width]] += pngimage.at<uchar>(y,x);
            numpixels[segmentation[x + y * width]] += 1;
        }
    }
    
    for (size_t i = 0; i < superpixelcount; ++i)
    {
        avgcolor[i] /= double(numpixels[i]);
		//std::cout << "Superpixel " << i << " avgcolor: "<<  avgcolor[i]  << std::endl;
    }
    
    cv::Mat pngimage2;
    pngimage.copyTo(pngimage2);
    if (csv_file.compare("null") != 0){
    for (unsigned int x = 0; x < width; ++x)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            auto current = x + y * width;
            auto right = x + 1 + y * width;
            auto left = x - 1 + y * width;
            auto below = x + (y + 1) * width;
            auto above = x + (y - 1) * width;
            if (x + 1 < width && segmentation[current] != segmentation[right])
            {
                pngimage.at<uchar>(y,x) = 0; // set pixel at the border to black
                pngimage2.at<uchar>(y,x) = 0; // set pixel at the border to black
                pngimage3.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 0); // colour pixel black
            }
            else if (x >= 1 && segmentation[current] != segmentation[left])
            {
                pngimage.at<uchar>(y,x) = 0; // set pixel at the border to black
                pngimage2.at<uchar>(y,x) = 0; // set pixel at the border to black
                pngimage3.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 0); // colour pixel black
            }
            else if (y + 1 < height && segmentation[current] != segmentation[below])
            {
                pngimage.at<uchar>(y,x) = 0; // set pixel at the border to black
                pngimage2.at<uchar>(y,x) = 0; // set pixel at the border to black
                pngimage3.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 0); // colour pixel black
            }
            else if (y  >= 1 && segmentation[current] != segmentation[above])
            {
                pngimage.at<uchar>(y,x) = 0; // set pixel at the border to black
                pngimage2.at<uchar>(y,x) = 0; // set pixel at the border to black
                pngimage3.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 0); // colour pixel black
            }
            else
            {
                pngimage2.at<uchar>(y,x) = avgcolor[segmentation[x + y * width]];
            }
        }
    }
    }
    cv::imwrite( "superpixels.png", pngimage3);
    cv::imwrite( "superpixels_avgcolor.png", pngimage2);
}



Graph Image::graph()
{
    Graph g(superpixelcount);
    for (auto p = vertices(g); p.first != p.second; ++p.first)
    {
        g[*p.first].color = avgcolor[*p.first]/255.0;
    }

    // store the corresponding pixels for each superpixel
    for (unsigned int x = 0; x < width; ++x)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            Graph::vertex_descriptor superpixel = segmentation[x + y * width];
            g[superpixel].pixels.push_back(Pixel{x, y});
        }
    }
    // add edges
    for (unsigned int x = 0; x < width; ++x)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            auto current = x + y * width;
            auto right = x + 1 + y * width;
            auto below = x + (y + 1) * width;
            if (x + 1 < width
                && segmentation[current] != segmentation[right])
            {
                // add edge to the superpixel on the right, weight counts the neighboring pixels between two superpixels
                auto edge = add_edge(segmentation[current], segmentation[right], g); // returns a pair<edge_descriptor, bool>
                auto weight = boost::get(boost::edge_weight, g, edge.first);
                boost::put(boost::edge_weight, g, edge.first, weight + 1);
            }
            if (y + 1 < height
                && segmentation[current] != segmentation[below])
            {
                // add edge to the superpixel below
                auto edge = add_edge(segmentation[current], segmentation[below], g);
                auto weight = boost::get(boost::edge_weight, g, edge.first);
                boost::put(boost::edge_weight, g, edge.first, weight + 1);
            }
        }
    }
    
    // initialize .var[l][]
    for (int i=0; i<superpixelcount; i++)
      for (int j=0; j<4; j++)
	g[(Graph::vertex_descriptor)i].var.emplace_back();
      
    return g;
}



cv::Mat Image::writeSegments(std::vector<Graph::vertex_descriptor> master_nodes, std::vector<std::vector<Graph::vertex_descriptor>> segments, Graph& g)
{
    std::vector<std::vector<size_t>> pixeltosegment(height);
    for (size_t i = 0; i < pixeltosegment.size(); ++i)
        pixeltosegment[i].resize(width);
    for (unsigned int x = 0; x < width; ++x)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            Graph::vertex_descriptor superpixel = segmentation[x + y*width];
            size_t segment = 0;
            while (std::find(segments[segment].begin(), segments[segment].end(), superpixel) == segments[segment].end())
                ++segment;
            pixeltosegment[y][x] = segment;
        }
    }
    
    cv::Mat pngimage = cv::imread( "superpixels.png" , CV_LOAD_IMAGE_COLOR );
    for (unsigned int x = 0; x < width; ++x)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            // if the pixel is at the boundary of a master node
            if (std::find(master_nodes.begin(), master_nodes.end(), segmentation[x + y*width]) != master_nodes.end()
                && ((x+1 < width && segmentation[x + y*width] != segmentation[x+1 + y*width])
                || (y+1 < height && segmentation[x + y*width] != segmentation[x + (y+1)*width])
                || (x > 0 && segmentation[x + y*width] != segmentation[x-1 + y*width])
                || (y > 0 && segmentation[x + y*width] != segmentation[x + (y-1)*width])))
            {
                //thicker lines
                for(int i = -2; i < 2; ++i) {
                    for(int j = -2; j < 2; ++j) {
                        if(x+j >= 0 && x+j < width && y+i >= 0 && y+i < height)
                            pngimage.at<cv::Vec3b>(y+i,x+j) = cv::Vec3b(0, 0, 255); // colour pixel blue
                    }
                }
            }
            // if the pixel is at a boundary between segments
            else if ((x > 0 && pixeltosegment[y][x] != pixeltosegment[y][x-1])
                || (x+1 < width && pixeltosegment[y][x] != pixeltosegment[y][x+1])
                || (y > 0 && pixeltosegment[y][x] != pixeltosegment[y-1][x])
                || (y+1 < height && pixeltosegment[y][x] != pixeltosegment[y+1][x]))
            {
                for(int i = -2; i < 2; ++i) {
                    for(int j = -2; j < 2; ++j) {
                        if(x+j >= 0 && x+j < width && y+i >= 0 && y+i < height)
                            pngimage.at<cv::Vec3b>(y+i,x+j) = cv::Vec3b(255, 0, 0); // colour pixel at segment boundary red
                    }
                }
            }
        }
    }
    
    std::cout << "write segments.png" << std::endl;
    cv::imwrite( "segments.png", pngimage);
    std::cout << "write segments.png sucess." << std::endl;

    return pngimage;
}


uint32_t Image::pixelToSuperpixel(uint32_t x, uint32_t y)
{
    return segmentation[x + y * width];
}
