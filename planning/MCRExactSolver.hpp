/* This hpp file declares MCR exact search on a given labeled graph
with specified start and goal*/

#ifndef MCRSOLVER_H
#define MCRSOLVER_H

#include <vector>
#include <queue>
#include <cstring>
#include <fstream>

#include "Graph.hpp"

struct MCRENode_t
{
	int m_id;
	float m_g;
	// label and cardinality
	std::vector<int> m_labels;
	int m_labelCardinality;

	MCRENode_t *m_parent;

	MCRENode_t(int id, float g, std::vector<int> ls, int c, MCRENode_t *p)
	{
		m_id = id;
		m_g = g;
		m_labels = ls;
		m_labelCardinality = c;
		m_parent = p;
	}
};

struct MCRENode_comparison
{
	bool operator()(const MCRENode_t* a, const MCRENode_t* b)
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

class MCRExactSolver_t
{
	std::vector<int> m_path;
	std::vector<std::vector<float>> m_trajectory;

	std::priority_queue<MCRENode_t*, std::vector<MCRENode_t*>, MCRENode_comparison> m_open;	
	std::vector<MCRENode_t*> m_closed;
	std::vector<bool> m_visited;
	std::vector<std::vector<std::vector<int>>> m_recordSet;
	std::vector<float> m_H;

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
	MCRExactSolver_t(Graph_t &g, int start, std::vector<int> goalSet);
	~MCRExactSolver_t();

	void computeH(Graph_t &g);
	void MCRExact_search(Graph_t &g);
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
	bool check_superset(int, std::vector<int>);	
	bool check_subset(std::vector<int>, std::vector<int>);

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