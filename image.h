#ifndef IMAGE_H
#define IMAGE_H

#include "graph.h"
#include <opencv2/imgproc/imgproc.hpp>
/**
 * Class representing png image
 */
class Image {
public:
    Image(
    std::string png_file, ///< PNG image to read
    std::string csv_file// the csv file containing superpixel information
    );
    
    /**
     * Creates a Boost graph consisting of the generated superpixels
     * The number of adjacent pixels between superpixels is stored as edge weight.
     */
    Graph graph(); 

    /*
     * Writes segements into segments.png
     */
    cv::Mat writeSegments(
        std::vector<Graph::vertex_descriptor> master_nodes,  ///< master nodes of all segments 
        std::vector<std::vector<Graph::vertex_descriptor>> segments, ///< optimal segmentation, where each segment is a vector consisting of the superpixels contained in it
        Graph& g ///< the graph of superpixels 
        );

    uint32_t pixelToSuperpixel(uint32_t x, uint32_t y);
    
private:
    unsigned int width;
    unsigned int height;
    unsigned int superpixelcount;
    std::vector<unsigned int> segmentation;
    std::vector<double> avgcolor;
    std::string filename;

    
};

#endif
