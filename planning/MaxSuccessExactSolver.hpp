/* This hpp file declares MaxSuccess Exact search on a given labeled graph
with specified start and goal set */

#ifndef MAXSUCCESSEXACTSOLVER_H
#define MAXSUCCESSEXACTSOLVER_H

#include <vector>
#include <queue>
#include <cstring>
#include <fstream>
#include <map>

#include "Graph.hpp"

struct MaxSuccExactNode_t
{
	int m_id;
	float m_g;
	// label and survivability
	std::vector<int> m_labels;
	float m_survival; // total survivability of labels
	int m_labelCardinality;

	//goal uncertainty
	std::vector<int> m_goalIndexes;
	float m_reachability;
	float m_successValue;
	bool m_isGoal;

	MaxSuccExactNode_t *m_parent;

	MaxSuccExactNode_t(int id, float g, std::vector<int> labels, float survival, 
		std::vector<int> goalIdxes, float reachability, bool isGoal, MaxSuccExactNode_t *parent)
	{
		m_id = id;
		m_g = g;
		m_labels = labels;
		m_survival = survival;
		m_labelCardinality = m_labels.size();
		m_goalIndexes = goalIdxes;
		m_reachability = reachability;
		m_successValue = m_survival * m_reachability;
		m_isGoal = isGoal;
		m_parent = parent;
	}
};

struct MaxSuccExactNode_comparison
{
	bool operator()(const MaxSuccExactNode_t* a, const MaxSuccExactNode_t* b)
	{
		if (a->m_successValue == b->m_successValue)
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
			return (a->m_successValue) < (b->m_successValue);
	}
};


class MaxSuccessExactSolver_t
{
	std::vector<int> m_path;
	std::vector<std::vector<float>> m_trajectory;

	std::priority_queue<MaxSuccExactNode_t*, std::vector<MaxSuccExactNode_t*>, 
													MaxSuccExactNode_comparison> m_open;
	std::vector<MaxSuccExactNode_t*> m_closed;
	std::vector<bool> m_visited;
	std::vector<std::vector<std::vector<int>>> m_recordSet;

	// No m_G: since we are not keep tracking the g-value for the best recorded node since we allow
	//         the same node to be expanded multiple times
	// Dynamic m_H: The heuristics are changing depending on the goals the path from start to 
	// that node aims at.
	int m_start;

	std::vector<int> m_goalSet;
	std::vector<std::vector<int>> m_targetPoses;
	std::map<int, std::vector<int>> m_goalmap;

	std::vector<int> m_goalhypos;
	std::map<int, int> m_goalCounts;
	std::map<int, std::vector<float>> m_goalMean;

	std::map<int, std::pair<int, float>> m_labelWeights;
	int m_nobstacles;

	std::ofstream m_outFile_;
	bool m_isFailure;

	std::vector<int> m_goalLabels;
	std::vector<int> m_goalIdxReached;
	float m_survivability;
	float m_pathSuccessValue;
	int m_obstaclesCollided;
	bool m_isPathSuccess;
	float m_pathCost;

public:
	MaxSuccessExactSolver_t(Graph_t &g);
	~MaxSuccessExactSolver_t();

	std::vector<int> update_goalIdxes(std::vector<int> currGoalIndexes, std::vector<int> edgelabels);	

	void countObsAndGoalHypos();
	void computeGoalMean(Graph_t &g);
	float computeH(std::vector<float> temp_state, std::vector<int> goalIdxes);
	float computeSurvival(std::vector<int> labels);
	float computeReach(std::vector<int> goalIdxes); // the highest reachability among available goals
	void MSExact_search(Graph_t &g);
	void back_track_path();
	void pathToTrajectory(Graph_t &g);
	void writeTrajectory(std::string trajectory_file);
	void printLabels();
	void print_path();
	void print_cost();
	void print_goalIdxReached();
	void print_survivability();
	void print_successValue();
	void printAll();

	std::vector<int> cal_remainingPoses(std::vector<int> neighbor_goalIdxes, std::vector<int> goalposeSet);

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

	// test
	void printToVerify();
};


#endif