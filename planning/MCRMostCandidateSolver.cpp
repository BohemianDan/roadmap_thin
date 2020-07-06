/* This cpp file defines MCR most candidate greedy search on a given labeled graph
with specified start and goal */

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "Graph.hpp"
#include "MCRMostCandidateSolver.hpp"
#include "Timer.hpp"


MCRMostCandidateSolver_t::MCRMostCandidateSolver_t(Graph_t &g, int start, std::vector<int> goalSet)
{
	// initialize the start & goalSet
	m_start = start;
	std::vector<int> temp_goalSet = goalSet;
	std::vector<std::vector<int>> temp_targetPoses = g.getTargetPoses();
	for (int i=0; i < temp_targetPoses.size(); i++)
	{
		m_trueMap[temp_goalSet[i]] = temp_targetPoses[i];
	}
	m_mostPromisingLabels = g.getMostPromisingLabels();
	// construct m_goalSet and m_targetPoses
	for (int i=0; i < temp_targetPoses.size(); i++)
	{
		std::vector<int> tps = temp_targetPoses[i];
		if ( std::find(tps.begin(), tps.end(), m_mostPromisingLabels[0]) != tps.end() )
		{
			m_targetPoses.push_back(m_mostPromisingLabels[0]);
			m_goalSet.push_back(temp_goalSet[i]);
		}
		else {continue;}
	}
	for (int i=0; i < m_goalSet.size(); i++)
	{
		m_goalmap[m_goalSet[i]] = m_targetPoses[i];
	}


	// essential elements for MCR most candidate greedy search
	m_G = std::vector<float>(g.getnNodes(), std::numeric_limits<float>::max());
	m_G[m_start] = 0.0;
	m_smallestCardinality = std::vector<int>(g.getnNodes(), std::numeric_limits<int>::max());
	m_smallestCardinality[m_start] = 0;
	m_open.push(new MCRMCNode_t(m_start, m_G[m_start], {}, 0, nullptr));
	m_expanded = std::vector<bool>(g.getnNodes(), false);

	m_isFailure = false;

}


MCRMostCandidateSolver_t::~MCRMostCandidateSolver_t()
{
	while (!m_open.empty())
	{
		MCRMCNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

// void MCRMostCandidateSolver_t::computeH(Graph_t &g)
// {
// 	std::vector<float> goal_mean = std::vector<float>(g.getState(0).size(), 0.0);
// 	for (auto const &goal : m_goalSet)
// 	{
// 		std::vector<float> v_goal = g.getState(goal);
// 		for (int j=0; j < v_goal.size(); j++)
// 		{
// 			goal_mean[j] = goal_mean[j] + v_goal[j];
// 		}
// 	}
// 	for (int j=0; j < goal_mean.size(); j++)
// 	{
// 		goal_mean[j] = goal_mean[j] / m_goalSet.size();
// 	}

// 	std::vector<float> temp_v;
// 	for (int i=0; i < g.getnNodes(); i++)
// 	{
// 		if (std::find(m_goalSet.begin(), m_goalSet.end(), i) != m_goalSet.end())
// 		{
// 			m_H.push_back(0.0);
// 			continue;
// 		}
// 		// compute euclidean distance
// 		float temp_h = 0.0;
// 		temp_v = g.getState(i);
// 		for (int j=0; j < goal_mean.size(); j++)
// 		{
// 			temp_h += pow(goal_mean[j]-temp_v[j], 2);
// 		}
// 		temp_h = sqrt(temp_h);
// 		m_H.push_back(temp_h);
// 	}
// }

void MCRMostCandidateSolver_t::MCRMCGreedy_search(Graph_t &g)
{

	while (!m_open.empty())
	{
		MCRMCNode_t *current = m_open.top();
		m_open.pop();
		// Now check if the current node has been expanded
		if (m_expanded[current->m_id] == true)
		{
			// This node has been expanded with the smallest label cardinality for its id
			// No need to put it into the closed list
			delete current;
			continue;
		}
		m_closed.push_back(current);
		m_expanded[current->m_id] = true;

		// a goal in the goalSet has been found
		if ( std::find(m_goalSet.begin(), m_goalSet.end(), current->m_id) != m_goalSet.end() )
		{
			std::cout << "MCRMLC PATH FOUND\n";
			back_track_path(); // construct your path
			pathToTrajectory(g);
			m_goalLabels = calTrueLabels(g);
			computeLabelsAndSurvival(g);
			// printLabels();
			// print the pose the goal indicates
			m_goalIdxReached = m_trueMap[current->m_id];
			m_pathCost = current->m_g;

			return;
		}
		// get neighbors of the current node
		std::vector<int> neighbors = g.getNodeNeighbors(current->m_id);
		for (auto const &neighbor : neighbors)
		{
			// check if the neighbor node has been visited or expanded before
			if ( m_expanded[neighbor] ) {continue;}
			// check neighbor's labels
			std::vector<int> neighborLabels =
				label_union(current->m_labels, g.getEdgeLabels(current->m_id, neighbor));
			int labelsSize = neighborLabels.size();
			// If the neighbor has a smller labels cardinality, update the smallest cardinality
			// record and put into open
			if (labelsSize < m_smallestCardinality[neighbor])
			{
				m_smallestCardinality[neighbor] = labelsSize;
				m_G[neighbor] = m_G[current->m_id]+g.getEdgeCost(current->m_id, neighbor);
				m_open.push(new MCRMCNode_t(neighbor, m_G[neighbor], neighborLabels, labelsSize, current));
				continue;
			}
			if (labelsSize == m_smallestCardinality[neighbor])
			{
				if (m_G[current->m_id]+g.getEdgeCost(current->m_id, neighbor) < m_G[neighbor])
				{
					m_G[neighbor] = m_G[current->m_id]+g.getEdgeCost(current->m_id, neighbor);
					m_open.push(new MCRMCNode_t(neighbor, m_G[neighbor], neighborLabels, labelsSize, current));

				}
			}

		}
	}
	// You are reaching here since the open list is empty and the goal is not found
	std::cout << "The problem is not solvable. Search failed...\n";
	m_isFailure = true;
}

// void MCRMostCandidateSolver_t::checkPathSuccess(int nhypo)
// {
// 	m_obstaclesCollided = 0;
// 	m_isPathSuccess = true;
// 	// compute the obstacles collided
// 	// loop through the m_goalLabels
// 	for (auto const &l : m_goalLabels)
// 	{
// 		if (l % nhypo == 0)
// 		{
// 			m_obstaclesCollided += 1;
// 		}
// 	}
// 	if (m_obstaclesCollided != 0 or m_goalIdxReached != 0)
// 	{
// 		m_isPathSuccess = false;
// 	}

// }


void MCRMostCandidateSolver_t::back_track_path()
{
	// start from the goal
	MCRMCNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	m_path.push_back(current->m_id);

	// print the path for checking purpose
	// std::cout << "path: \n";
	// for (auto const &waypoint : m_path)
	// {
	// 	std::cout << waypoint << " ";
	// }
	// std::cout << "\n";	
}

void MCRMostCandidateSolver_t::pathToTrajectory(Graph_t &g)
{
	// start from the start
	for (int i=m_path.size()-1; i >=0; i--)
	{
		m_trajectory.push_back(g.getState(m_path[i]));

	}
	// // print the trajectory for checking purpose
	// std::cout << "The trajectory: \n";
	// for (auto const &t : m_trajectory)
	// {
	// 	for (auto const &d : t)
	// 	{
	// 		std::cout << d << "   ";
	// 	}
	// 	std::cout << "\n";
	// }
}

void MCRMostCandidateSolver_t::writeTrajectory(std::string trajectory_file)
{
	m_outFile_.open(trajectory_file);
	if (m_outFile_.is_open())
	{
		for (auto const &t : m_trajectory)
		{
			for (auto const &d : t)
			{
				m_outFile_ << d << " ";
			}
			m_outFile_ << "\n";
		}
	}
	m_outFile_.close();

}

std::vector<int> MCRMostCandidateSolver_t::calTrueLabels(Graph_t &g)
{
	std::vector<int> true_labels;
	std::vector<int> s;
	for (int i=m_path.size()-1; i >=1; i--)
	{
		s = g.getEdgeLabels(m_path[i], m_path[i-1]);
		true_labels = normal_label_union(true_labels, s);
	}

	return true_labels;
}

void MCRMostCandidateSolver_t::print_path()
{
	// print the path for checking purpose
	std::cout << "path: \n";
	for (int i=m_path.size()-1; i >=0; i--)
	{
		std::cout << m_path[i] << " ";
	}
	// for (auto const &waypoint : m_path)
	// {
	// 	std::cout << waypoint << " ";
	// }
	std::cout << "\n";
}

void MCRMostCandidateSolver_t::printLabels()
{
	std::cout << "labels: " << "< ";
	for (auto const &l : m_goalLabels)
	{
		std::cout << l << " ";
	}
	std::cout << ">\n";	
}

void MCRMostCandidateSolver_t::print_cost()
{
	std::cout << "cost: " << m_pathCost << "\n";
}
		
void MCRMostCandidateSolver_t::print_goalIdxReached()
{
	std::cout << "The reaching target poses are: "; 
	for (auto const &p : m_goalIdxReached)
	{
		std::cout << p << " ";
	}
	std::cout << "\n";
}

void MCRMostCandidateSolver_t::print_survivability()
{
	std::cout << "survivability: " << m_survivability << "\n";
}

void MCRMostCandidateSolver_t::printAll()
{
	print_path();
	print_cost();
	printLabels();
	print_goalIdxReached();
	print_survivability();
}

void MCRMostCandidateSolver_t::computeLabelsAndSurvival(Graph_t &g)
{
	/// Let's compute the survivability
	std::map<int, std::pair<int, float>> label_weights = g.getLabelWeights();
	std::vector<float> temp_probs(label_weights[label_weights.size()-1].first+1, 0.0);
	for (auto const &l : m_goalLabels)
	{
		temp_probs[label_weights[l].first] += label_weights[l].second;
	}
	m_survivability = 1.0;
	for (auto const &temp_prob : temp_probs)
	{
		m_survivability = m_survivability * (1 - temp_prob);
	}	
}


std::vector<int> MCRMostCandidateSolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
{
	// sort the sets first before applying union operation
	std::sort(s1.begin(), s1.end());
	std::sort(s2.begin(), s2.end());

	// Declaring resultant vector for union
	std::vector<int> v(s1.size()+s2.size());
	// using function set_union() to compute union of 2
	// containers v1 and v2 and store result in v
	auto it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), v.begin());

	// resizing new container
	v.resize(it - v.begin());

	// std::cout << "the union label before processing: \n";
	// for (auto const &e : v)
	// {
	// 	std::cout << e << " ";
	// }
	// std::cout << "\n";

	// now return the true label set with only considering the most promising labels
	std::vector<int> vv;
	for (auto const &e : v)
	{
		if ( std::find(m_mostPromisingLabels.begin(), m_mostPromisingLabels.end(), e) != m_mostPromisingLabels.end() )
		{
			vv.push_back(e);
		}
	}
	// std::cout << "the union label after processing: \n";
	// for (auto const &e : vv)
	// {
	// 	std::cout << e << " ";
	// }
	// std::cout << "\n";

	return vv;
}

std::vector<int> MCRMostCandidateSolver_t::normal_label_union(std::vector<int> s1, std::vector<int> s2)
{
	// sort the sets first before applying union operation
	std::sort(s1.begin(), s1.end());
	std::sort(s2.begin(), s2.end());

	// Declaring resultant vector for union
	std::vector<int> v(s1.size()+s2.size());
	// using function set_union() to compute union of 2
	// containers v1 and v2 and store result in v
	auto it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), v.begin());

	// resizing new container
	v.resize(it - v.begin());
	return v;	
}