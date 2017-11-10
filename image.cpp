//#include <vl/slic.h>
#include "png++/png.hpp"
#include <iostream>
#include <cmath>
#include "graph.h"
#include "image.h"
#include "gurobi_c++.h"
    
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
    png::image<png::gray_pixel> pngimage(png_file); // greyscale pixels are integers from 0 to 255
    width = pngimage.get_width();
    height = pngimage.get_height();
    unsigned int imagesize = pngimage.get_width() * pngimage.get_height();
    
    // image is a vector containing pixel intensity
    float* image = new float[imagesize];
    for (png::uint_32 x = 0; x < pngimage.get_width(); ++x)
    {
        for (png::uint_32 y = 0; y < pngimage.get_height(); ++y)
        {
            image[x + y * pngimage.get_width()] = pngimage[y][x] / 255.0; // norm to floats from 0 to 1
        }
    }
    
    // segmentation is a list of superpixel labels 
    //std::vector<int> segmentation(imagesize);
    // use the class var

    
    // segmentation records the label of each pixel
    //segmentation.resize(vec.size());
    segmentation.resize(imagesize);
    
    
    // check if no superpixel is provided, we will use the pixel image directly
    if (csv_file.compare("null") == 0)
    {
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

      // read data from csv_file, needs csv to start from label 0, and be consistent
      ifstream in(data.c_str());
    
    
    
    
      if (!in.is_open()) exit(1);

      typedef tokenizer< escaped_list_separator<char> > Tokenizer;
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
      std::transform (vec.begin(), vec.end(), segmentation.begin(), [](string s) -> int {return stoi(s);});
    }
    
    

	// relabel the segmentation by -1, if using my csv
	#if 0
	for (png::uint_32 x = 0; x < pngimage.get_width(); ++x)
    {
        for (png::uint_32 y = 0; y < pngimage.get_height(); ++y)
        {
            segmentation[x + y * pngimage.get_width()] -= 1;
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
    superpixelcount++;
    std::cout << "Generated " << superpixelcount << " superpixels." << std::endl;
    

	
    avgcolor.resize(superpixelcount, 0.0);
    std::vector<unsigned int> numpixels(superpixelcount, 0);
    
	

    for (png::uint_32 x = 0; x < pngimage.get_width(); ++x)
    {
        for (png::uint_32 y = 0; y < pngimage.get_height(); ++y)
        {
            avgcolor[segmentation[x + y * pngimage.get_width()]] += pngimage[y][x];
            numpixels[segmentation[x + y * pngimage.get_width()]] += 1;
        }
    }
    
    for (size_t i = 0; i < superpixelcount; ++i)
    {
        avgcolor[i] /= numpixels[i];
		//std::cout << "Superpixel " << i << " avgcolor: "<<  avgcolor[i]  << std::endl;
    }
    
    png::image<png::gray_pixel> pngimage2(pngimage);
    // if no superpixels are provided
    if (csv_file.compare("null") != 0){
    for (png::uint_32 x = 0; x < pngimage.get_width(); ++x)
    {
        for (png::uint_32 y = 0; y < pngimage.get_height(); ++y)
        {
            auto current = x + y * width;
            auto right = x + 1 + y * width;
            auto left = x - 1 + y * width;
            auto below = x + (y + 1) * width;
            auto above = x + (y - 1) * width;
            if (x + 1 < width && segmentation[current] != segmentation[right])
            {
                pngimage[y][x] = 0; // set pixel at the border to black
                pngimage2[y][x] = 0; // set pixel at the border to black
            }
            else if (x >= 1 && segmentation[current] != segmentation[left])
            {
                pngimage[y][x] = 0; // set pixel at the border to black
                pngimage2[y][x] = 0; // set pixel at the border to black
            }
            else if (y + 1 < height && segmentation[current] != segmentation[below])
            {
                pngimage[y][x] = 0; // set pixel at the border to black
                pngimage2[y][x] = 0; // set pixel at the border to black
            }
            else if (y  >= 1 && segmentation[current] != segmentation[above])
            {
                pngimage[y][x] = 0; // set pixel at the border to black
                pngimage2[y][x] = 0; // set pixel at the border to black
            }
            else
            {
                pngimage2[y][x] = avgcolor[segmentation[x + y * pngimage.get_width()]];
            }
        }
    }
    }
    
    pngimage.write("superpixels.png");
    pngimage2.write("superpixels_avgcolor.png");
    
    
}





Graph Image::graph()
{
    Graph g(superpixelcount);
    for (auto p = vertices(g); p.first != p.second; ++p.first)
    {
        g[*p.first].color = avgcolor[*p.first]/255.0; // changes here
    }

    // store the corresponding pixels for each superpixel
    for (png::uint_32 x = 0; x < width; ++x)
    {
        for (png::uint_32 y = 0; y < height; ++y)
        {
            Graph::vertex_descriptor superpixel = segmentation[x + y * width];
            g[superpixel].pixels.push_back(Pixel{x, y});
        }
    }
    // add edges
    for (png::uint_32 x = 0; x < width; ++x)
    {
        for (png::uint_32 y = 0; y < height; ++y)
        {
            auto current = x + y * width;
            auto right = x + 1 + y * width;
            auto below = x + (y + 1) * width;
            if (x + 1 < width
                && segmentation[current] != segmentation[right])
            {
                // add edge to the superpixel on the right
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
    return g;
}




void Image::writeSegments(std::vector<Graph::vertex_descriptor> master_nodes, std::vector<std::vector<Graph::vertex_descriptor>> segments, Graph& g)
{
    std::vector<std::vector<size_t>> pixeltosegment(height);
    for (size_t i = 0; i < pixeltosegment.size(); ++i)
        pixeltosegment[i].resize(width);
    for (png::uint_32 x = 0; x < width; ++x)
    {
        for (png::uint_32 y = 0; y < height; ++y)
        {
            Graph::vertex_descriptor superpixel = segmentation[x + y*width];
            size_t segment = 0;
            while (std::find(segments[segment].begin(), segments[segment].end(), superpixel) == segments[segment].end())
                ++segment;
            pixeltosegment[y][x] = segment;
        }
    }
    
    png::image<png::rgb_pixel> pngimage("superpixels.png");
    for (png::uint_32 x = 0; x < width; ++x)
    {
        for (png::uint_32 y = 0; y < height; ++y)
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
                            pngimage[y+i][x+j] = png::rgb_pixel(0, 0, 255); // colour pixel blue
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
                            pngimage[y+i][x+j] = png::rgb_pixel(255, 0, 0); // colour pixel at segment boundary red
                    }
                }
            }
        }
    }
    
    std::cout << "write segments.png" << std::endl;
    pngimage.write("segments.png");
	std::cout << "write segments.png sucess." << std::endl;
}


uint32_t Image::pixelToSuperpixel(uint32_t x, uint32_t y)
{
    return segmentation[x + y * width];
}
