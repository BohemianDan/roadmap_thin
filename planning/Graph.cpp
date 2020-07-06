/* This cpp file declares a graph which is constructed 
from the roadmap built in robotic scenarios.*/

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>
#include <typeinfo>
// #include <cstdlib>

#include "Graph.hpp"

Graph_t::Graph_t(std::string samples_file, std::string roadmap_file, std::string labelWeight_file, 
	std::string mostPromisingLabels_file, std::string goals_file, int nsamples)
{
	specify_nodeStates(samples_file, goals_file, nsamples);
	// After getting all the node states (samples)
	m_nNodes = m_nodeStates.size();
	specify_neighborCostsAndLabels(roadmap_file);
	specify_labelWeight(labelWeight_file);
	specify_mostPromisingLabels(mostPromisingLabels_file);
}


void Graph_t::specify_nodeStates(std::string samples_file, std::string goals_file, int nsamples)
{
	// read in the samples file
	// std::cout << samples_file << "\n";
	m_inFile_.open(samples_file);
	std::string temp_str;
	int temp_nodeIdx;
	float c;
	int d;
	// Check that the file was opened successfully
	if (!m_inFile_)
	{
		std::cerr << "Unable to open the samples file\n";
		exit(1); // call system to stop
	}
	/// read through samples file to construct m_nodeStates
	while (std::getline(m_inFile_, temp_str))
	{

		std::stringstream ss(temp_str);
		ss >> temp_nodeIdx;
		std::vector<float> temp_v;
		while (ss >> c)
		{
			temp_v.push_back(c);
		}
		m_nodeStates.push_back(temp_v);
	}
	m_inFile_.close();

	/// specify start
	m_start = nsamples;

	/// Now read in the goals file 
	m_inFile_.open(goals_file);
	if (!m_inFile_)
	{
		std::cerr << "Unable to open the goals file\n";
		exit(1); // call system to stop		
	}
	/// read through goals file to construct m_goalSet & m_targetPoses
	while (std::getline(m_inFile_, temp_str))
	{
		std::stringstream ss(temp_str);
		ss >> temp_nodeIdx;
		m_goalSet.push_back(temp_nodeIdx);
		std::vector<int> temp_d;
		while (ss >> d)
		{
			temp_d.push_back(d);
		}
		m_targetPoses.push_back(temp_d);
	}
	m_inFile_.close();

}


void Graph_t::specify_mostPromisingLabels(std::string mostPromisingLabels_file)
{
	m_inFile_.open(mostPromisingLabels_file);
	// std::cout << mostPromisingLabels_file << "\n";
	if (!m_inFile_)
	{
		std::cerr << "Unable to open the mostPromisingLabels file\n";
		exit(1); // call system to stop
	}
	std::string temp_str;
	int c;
	while (std::getline(m_inFile_, temp_str))
	{
		std::stringstream ss(temp_str);
		while (ss >> c)
		{
			m_mostPromisingLabels.push_back(c);
		}
	}
}


void Graph_t::specify_neighborCostsAndLabels(std::string roadmap_file)
{
	int iter = 0;
	// create empty neighbors and cost vector for each node
	while (iter != m_nNodes)
	{
		m_nodeNeighbors.push_back(std::vector<int>());
		m_edgeLabels.push_back(std::vector<std::vector<int>>(m_nNodes,
			 std::vector<int>()));
		m_edgeCosts.push_back(std::vector<float>(m_nNodes, std::numeric_limits<float>::max()));
		iter++;
	}

	// read in the roadmap
	// std::cout << roadmap_file << "\n";
	m_inFile_.open(roadmap_file);
	std::string temp_str;
	int temp_n1;
	int temp_n2;
	float temp_cost;
	int c;
	// Check that the file was opened successfully
	if (!m_inFile_)
	{
		std::cerr << "Unable to open the roadmap file\n";
		exit(1); // call system to stop
	}
	while (std::getline(m_inFile_, temp_str))
	{
		std::stringstream ss(temp_str);
		ss >> temp_n1 >> temp_n2 >> temp_cost;
		m_nodeNeighbors[temp_n1].push_back(temp_n2);
		m_nodeNeighbors[temp_n2].push_back(temp_n1);
		m_edgeCosts[temp_n1][temp_n2] = temp_cost;
		m_edgeCosts[temp_n2][temp_n1] = temp_cost;
		while (ss >> c)
		{
			m_edgeLabels[temp_n1][temp_n2].push_back(c);
			m_edgeLabels[temp_n2][temp_n1].push_back(c);

		}
	}

	m_inFile_.close();
}

void Graph_t::specify_labelWeight(std::string labelWeight_file)
{
	// read the label weight
	m_inFile_.open(labelWeight_file);
	// std::cout << labelWeight_file << "\n";
	std::string temp_str;
	int temp_labelIdx;
	int temp_objIdx;
	float temp_weight;
	// Check that the file was opened successfully
	if (!m_inFile_)
	{
		std::cerr << "Unable to open the label weight file\n";
		exit(1); // call system to stop
	}
	while (std::getline(m_inFile_, temp_str))
	{
		std::stringstream ss(temp_str);
		ss >> temp_labelIdx >> temp_objIdx >> temp_weight;
		m_labelWeights[temp_labelIdx] = std::pair<int, float>(temp_objIdx, temp_weight);
	}
	// while (!m_inFile_.eof())
	// {
	// 	m_inFile_ >> temp_labelIdx >> temp_objIdx >> temp_weight;
	// 	m_labelWeights[temp_labelIdx] = std::pair<int, float>(temp_objIdx, temp_weight);
	// }
	m_inFile_.close();
}

void Graph_t::print_graph()
{
	std::cout << "___most promissing labels___\n";
	for (auto const &mpl : m_mostPromisingLabels)
	{
		std::cout << mpl << " ";
	}
	std::cout << "\n";
	// print the m_nodeStates
	// std::cout << "_________node states_________\n";
	// for (auto const &n : m_nodeStates)
	// {
	// 	for (auto const &e : n)
	// 	{
	// 		std::cout << e << " ";
	// 	}
	// 	std::cout << "\n";
	// }
	// // std::cout << "\n\n\n";

	// // print the m_nodeNeighbors
	// std::cout << "_________node neighbors_________\n";
	// for (auto const &nbs : m_nodeNeighbors)
	// {
	// 	for (auto const &nb : nbs)
	// 	{
	// 		std::cout << nb << " ";
	// 	}
	// 	std::cout << "\n";
	// }
	// // std::cout << "\n\n\n";

	// // print the m_edgeCosts
	// std::cout << "_________edge cost_________\n";
	// for (auto const &costs : m_edgeCosts)
	// {
	// 	for (auto const &c : costs)
	// 	{
	// 		std::cout << c << "\t";
	// 	}
	// 	std::cout << "\n";
	// }
	// // std::cout << "\n\n\n";

	// // print the m_edgeLabels
	// std::cout << "_________edge labels_________\n";
	// for (auto const &edges : m_edgeLabels)
	// {
	// 	for (auto const &e : edges)
	// 	{
	// 		for (auto const &l : e)
	// 		{
	// 			std::cout << l << ",";
	// 		}
	// 		std::cout << "\t";
	// 	}
	// 	std::cout << "\n";
	// }

	// print the label weights
	std::cout << "_________label weights_________\n";
	for ( auto const &lw : m_labelWeights )
	{
		std::cout << lw.first << " " << lw.second.first << " " << lw.second.second << "\n";
	}

	// print start, goalSet and targetPoses
	std::cout << "____start, goalSet and targetPoses________\n";
	std::cout << "Start: " << m_start << "\n";
	std::cout << "GoalSet: " << "\n";
	for (auto const &g : m_goalSet)
	{
		std::cout << g << " ";
	}
	std::cout << "\n";
	std::cout << "Target Poses: " << "\n";
	for (auto const &tps : m_targetPoses)
	{
		for (auto const &tp : tps)
		{
			std::cout << tp << " ";
		}
		std::cout << "\n";

	}
	std::cout << "\n";
}