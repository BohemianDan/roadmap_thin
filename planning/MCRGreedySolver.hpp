/* This hpp file declares MCR greedy search on a given labeled graph
with specified start and goal set */

#ifndef MCRGREEDYSOLVER_H
#define MCRGREEDYSOLVER_H

#include <vector>
#include <queue>
#include <cstring>
#include <fstream>

#include "Graph.hpp"

struct MCRGNode_t
{
	int m_id;
	float m_g;

	// label and cardinality
	std::vector<int> m_labels;
	int m_labelCardinality;

	MCRGNode_t *m_parent;

	MCRGNode_t(int id, float g, std::vector<int> ls, int c, MCRGNode_t *p)
	{
		m_id = id;
		m_g = g;
		m_labels = ls;
		m_labelCardinality = c;
		m_parent = p;
	}
};

struct MCRGNode_comparison
{
	bool operator()(const MCRGNode_t* a, const MCRGNode_t* b)
	{
		if (a->m_labelCardinality == b->m_labelCardinality)
		{
			return (a->m_g) > (b->m_g);
			// if (a->m_f == b->m_f)
			// {
			// 	return (a->m_h) > (b->m_h);
			// }
			// else
			// 	return (a->m_f) > (b->m_f);
		}
		else
			return (a->m_labelCardinality) > (b->m_labelCardinality);
	}
};


class MCRGreedySolver_t
{
	std::vector<int> m_path;
	std::vector<std::vector<float>> m_trajectory;

	std::priority_queue<MCRGNode_t*, std::vector<MCRGNode_t*>, MCRGNode_comparison> m_open;	
	std::vector<MCRGNode_t*> m_closed;
	std::vector<bool> m_expanded;

	std::vector<float> m_G;
	std::vector<float> m_H;
	std::vector<int> m_smallestCardinality;
	int m_start;
	std::vector<int> m_goalSet;
	std::vector<std::vector<int>> m_targetPoses;
	std::map<int, std::vector<int>> m_goalmap;

	std::ofstream m_outFile_;
	bool m_isFailure;

	std::vector<int> m_goalLabels;
	std::vector<int> m_goalIdxReached;
	float m_survivability;
	int m_obstaclesCollided;
	bool m_isPathSuccess;
	float m_pathCost;	

public:
	MCRGreedySolver_t(Graph_t &g, int start, std::vector<int> goalSet);
	~MCRGreedySolver_t();

	void computeH(Graph_t &g);
	void MCRGreedy_search(Graph_t &g);
	void back_track_path();
	void pathToTrajectory(Graph_t &g);
	void writeTrajectory(std::string trajectory_file);
	void computeLabelsAndSurvival(Graph_t &g);
	void printLabels();
	void print_path();
	void print_cost();
	void print_goalIdxReached();
	void print_survivability();
	void printAll();
	
	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);

	// // harvest the results
	// void checkPathSuccess(int nhypo);

	// getters
	std::vector<std::vector<float>> getTrajectory() {return m_trajectory;}
	bool getFailureIndicator() {return m_isFailure;}
	int getObstaclesCollided() {return m_obstaclesCollided;}
	bool getIsPathSuccess() {return m_isPathSuccess;}
	float getPathCost() {return m_pathCost;}
};


#endif