#ifndef CALLBACK_H
#define CALLBACK_H
#include "graph.h"
#include "gurobi_c++.h"



//graph is a pointer to the SuperpixelGraph in main.cpp, graph_ is just function paremeter

class myGRBCallback: public GRBCallback /// inherit public class
{
  public:
    Graph& graph;
    std::vector<Graph::vertex_descriptor> root_nodes; 
    void find_cuts(Graph& graph, Graph& subgraph, std::vector<int> component, int itr, int num_components, std::vector<int> component_num);
    //int vertex_find(Graph& subgraph, int vertex);
    myGRBCallback(Graph& graph_, std::vector<Graph::vertex_descriptor>& root_nodes_) : graph(graph_), root_nodes(root_nodes_) {};
  protected:
    void callback ();

};
#endif
