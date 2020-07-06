/* This cpp file defines MCR exact search on a given labeled graph
with specified start and goal */

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "Graph.hpp"
#include "MCRExactSolver.hpp"
#include "Timer.hpp"


MCRExactSolver_t::MCRExactSolver_t(Graph_t &g, int start, std::vector<int> goalSet)
{
	//initialize the start & goalSet
	m_start = start;
	m_goalSet = goalSet;
	m_targetPoses = g.getTargetPoses();
	for (int i=0; i < m_goalSet.size(); i++)
	{
		m_goalmap[m_goalSet[i]] = m_targetPoses[i];
	}
	// essential elements for MCR exact search
	computeH(g); // heuristics
	m_open.push( new MCRENode_t(m_start, 0.0, {}, 0, nullptr) );
	m_visited = std::vector<bool>(g.getnNodes(), false);
	m_visited[m_start] = true;
	int n_nodes = g.getnNodes();
	for (int hh=0; hh < n_nodes; hh++)
	{
		m_recordSet.push_back(std::vector<std::vector<int>>());
	}


	m_isFailure = false;


}

MCRExactSolver_t::~MCRExactSolver_t()
{
	while (!m_open.empty())
	{
		MCRENode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}


void MCRExactSolver_t::MCRExact_search(Graph_t &g)
{
	while (!m_open.empty())
	{
		MCRENode_t *current = m_open.top();
		m_open.pop();
		// We do not check the expansion status of the node since each node can be expanded more than once
		// We don't check the superset-subset relationship either since they will be checked when deciding whether to put in the open list
		m_closed.push_back(current);
		// a goal in the goalSet has been found
		if ( std::find(m_goalSet.begin(), m_goalSet.end(), current->m_id) != m_goalSet.end() )
		{
			std::cout << "MCRE PATH FOUND\n";
			back_track_path(); // construct your path
			pathToTrajectory(g);
			m_goalLabels = current->m_labels;
			computeLabelsAndSurvival(g);
			// printLabels();
			// print the pose the goal indicates
			m_goalIdxReached = m_goalmap[current->m_id];
			m_pathCost = current->m_g;

			return;			
		}
		// If it is not the goal, let's keep moving
		// look at each neighbor of the current node
		std::vector<int> neighbors = g.getNodeNeighbors(current->m_id);
		for (auto const &neighbor : neighbors)
		{
			// check neighbor's labels
			std::vector<int> neighborLabels = 
				label_union(current->m_labels, g.getEdgeLabels(current->m_id, neighbor));

			// check whether we need to put this neighbor into the priority queue (based on labels)
			// Every time we look at a neighbor, check if the labels it carries
			// is a super set of any of the set in the m_recordSet ( except for the first time :) )

			// The first time visited
			if (m_visited[neighbor] == false)
			{
				int labelsSize = neighborLabels.size();
				float temp_g = current->m_g + g.getEdgeCost(current->m_id, neighbor);
				// now put it to the open list
				m_open.push( new MCRENode_t(neighbor, temp_g, neighborLabels, labelsSize, current) );
				m_recordSet[neighbor].push_back(neighborLabels);
				m_visited[neighbor] = true;
				continue;
			}
			else // not the first time visited
			{
				// check if the label set is a super set of any set that 
				// we have seen before in m_recordSet
				if (!check_superset(neighbor, neighborLabels))
				{
					int labelsSize = neighborLabels.size();
					float temp_g = current->m_g + g.getEdgeCost(current->m_id, neighbor);
					// now put it to the open list
					m_open.push( new MCRENode_t(neighbor, temp_g, neighborLabels, labelsSize, current) );
					m_recordSet[neighbor].push_back(neighborLabels);

				}		
			}
		}

	}
	// You are reaching here since the open list is empty and the goal is not found
	std::cout << "The problem is not solvable. Search failed...\n";
	m_isFailure = true;
}

void MCRExactSolver_t::computeH(Graph_t &g)
{
	std::vector<float> goal_mean = std::vector<float>(g.getState(0).size(), 0.0);
	for (auto const &goal : m_goalSet)
	{
		std::vector<float> v_goal = g.getState(goal);
		for (int j=0; j < v_goal.size(); j++)
		{
			goal_mean[j] = goal_mean[j] + v_goal[j];
		}
	}
	for (int j=0; j < goal_mean.size(); j++)
	{
		goal_mean[j] = goal_mean[j] / m_goalSet.size();
	}

	std::vector<float> temp_v;
	for (int i=0; i < g.getnNodes(); i++)
	{
		if (std::find(m_goalSet.begin(), m_goalSet.end(), i) != m_goalSet.end())
		{
			m_H.push_back(0.0);
			continue;
		}
		// compute euclidean distance
		float temp_h = 0.0;
		temp_v = g.getState(i);
		for (int j=0; j < goal_mean.size(); j++)
		{
			temp_h += pow(goal_mean[j]-temp_v[j], 2);
		}
		temp_h = sqrt(temp_h);
		m_H.push_back(temp_h);
	}
}

// void MCRExactSolver_t::checkPathSuccess(int nhypo)
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

void MCRExactSolver_t::back_track_path()
{
	// start from the goal
	MCRENode_t *current = m_closed[m_closed.size()-1];
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

void MCRExactSolver_t::pathToTrajectory(Graph_t &g)
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

void MCRExactSolver_t::writeTrajectory(std::string trajectory_file)
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

void MCRExactSolver_t::print_path()
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

void MCRExactSolver_t::printLabels()
{
	std::cout << "labels: " << "< ";
	for (auto const &l : m_goalLabels)
	{
		std::cout << l << " ";
	}
	std::cout << ">\n";	
}

void MCRExactSolver_t::print_cost()
{
	std::cout << "cost: " << m_pathCost << "\n";
}
		
void MCRExactSolver_t::print_goalIdxReached()
{
	std::cout << "The reaching target poses are: "; 
	for (auto const &p : m_goalIdxReached)
	{
		std::cout << p << " ";
	}
	std::cout << "\n";
}

void MCRExactSolver_t::print_survivability()
{
	std::cout << "survivability: " << m_survivability << "\n";
}

void MCRExactSolver_t::printAll()
{
	print_path();
	print_cost();
	printLabels();
	print_goalIdxReached();
	print_survivability();
}

void MCRExactSolver_t::computeLabelsAndSurvival(Graph_t &g)
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

std::vector<int> MCRExactSolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
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

bool MCRExactSolver_t::check_superset(int neighbor, std::vector<int> neighborLabels)
{
	bool isSuperset = false;
	for (auto const s: m_recordSet[neighbor])
	{
		if ( check_subset(neighborLabels, s) ) { return true; }
	}

	return isSuperset;
}

bool MCRExactSolver_t::check_subset(std::vector<int> set, std::vector<int> subset)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	std::sort(set.begin(), set.end());
	std::sort(subset.begin(), subset.end());
	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
}