/* This hpp file declares a graph which is constructed 
from the roadmap built in robotic scenarios.*/

#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string> // std::string, std::to_string
#include <fstream>
#include <map>

class Graph_t
{
	// the size of the graph
	int m_nNodes;

	// specify neighbors(edges) and edge cost of the graph
	std::vector<std::vector<int>> m_nodeNeighbors;
	std::vector<std::vector<float>> m_edgeCosts;
	std::vector<std::vector<float>> m_nodeStates;
	// specify the weight for each label (labeled graph)
	std::map<int, std::pair<int, float>> m_labelWeights;
	std::vector<std::vector<std::vector<int>>> m_edgeLabels;

	// start and goal set
	int m_start;
	// goal set 
	std::vector<int> m_goalSet;
	std::vector<std::vector<int>> m_targetPoses;

	// samples and roadmap reader
	std::ifstream m_inFile_;

	// most promising labels
	std::vector<int> m_mostPromisingLabels;


public:
	// Constructor
	Graph_t() {}
	Graph_t(std::string samples_file, std::string roadmap_file, std::string labelWeight_file, 
		std::string mostPromisingLabels_file, std::string goals_file, int nsamples);

	// construct the graph
	void specify_nodeStates(std::string samples_file, std::string goals_file, int nsamples);
	void specify_neighborCostsAndLabels(std::string roadmap_file);
	void specify_labelWeight(std::string labelWeight_file);
	void specify_mostPromisingLabels(std::string mostPromisingLabels_file);
	void print_graph();

	// getters
	int getnNodes() { return m_nNodes; }
	std::vector<float> getState(int idx) { return m_nodeStates[idx]; }
	int getStart() { return m_start; }
	std::vector<int> getGoalSet() { return m_goalSet; }
	std::vector<std::vector<int>> getTargetPoses() { return m_targetPoses; }
	std::vector<int> getNodeNeighbors(int id) { return m_nodeNeighbors[id]; }
	float getEdgeCost(int id1, int id2) { return m_edgeCosts[id1][id2]; }
	std::vector<int> getEdgeLabels(int id1, int id2) { return m_edgeLabels[id1][id2]; }
	std::map<int, std::pair<int, float>> getLabelWeights() { return m_labelWeights; }
	float getSingleWeight(int l) { return m_labelWeights[l].second; }
	std::vector<int> getMostPromisingLabels() { return m_mostPromisingLabels; }
	// Destructor
	~Graph_t() {}
};


#endif