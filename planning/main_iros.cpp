#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "Graph.hpp"
#include "AstarSolver.hpp"
#include "MCRGreedySolver.hpp"
#include "MCRExactSolver.hpp"
#include "MaxSuccessExactSolver.hpp"
#include "MaxSuccessGreedySolver.hpp"
#include "MCRMostCandidateSolver.hpp"
#include "Timer.hpp"

int main(int argc, char** argv)
{
	Timer t;
	std::string traj;
	std::ofstream outFile;

	std::string img_idx = std::string(argv[2]);
	std::string roadmap_idx = std::string(argv[3]);
	std::string samples_file = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/samples.txt";
	// std::cout << samples_file << "\n";
	std::string roadmap_file = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/roadmap.txt";
	std::string goals_file = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/goals.txt";
	std::string labelWeight_file = "../iros_20_motoman_experiment/data/" + img_idx + "/labelWeights.txt";
	std::string mostPromisingLabels_file = "../iros_20_motoman_experiment/data/" + img_idx + "/mostPromisingLabels.txt";
	

	int nsamples = atoi(argv[1]);

	t.reset();
	// import the graph
	float graphConstructionTime;
	Graph_t g(samples_file, roadmap_file, labelWeight_file, mostPromisingLabels_file, goals_file, nsamples);
	graphConstructionTime = t.elapsed();
	std::cout << "Time to import the graph for " 
						<< g.getnNodes() << " nodes: " << graphConstructionTime << "\n\n";
	
	std::vector<float> planningTime;

	t.reset();
	AstarSolver_t astar_solver(g, g.getStart(), g.getGoalSet());
	astar_solver.Astar_search(g);
	astar_solver.printAll();
	std::cout << "Astar time: " << t.elapsed() << "\n\n";
	planningTime.push_back(graphConstructionTime + t.elapsed());

	t.reset();
	MCRGreedySolver_t mcr_gsolver(g, g.getStart(), g.getGoalSet());
	mcr_gsolver.MCRGreedy_search(g);
	mcr_gsolver.printAll();
	std::cout << "MCRG time: " << t.elapsed() << "\n\n";
	planningTime.push_back(graphConstructionTime + t.elapsed());

	t.reset();
	MCRExactSolver_t mcr_esolver(g, g.getStart(), g.getGoalSet());
	mcr_esolver.MCRExact_search(g);
	mcr_esolver.printAll();
	std::cout << "MCRE time: " << t.elapsed() << "\n\n";
	planningTime.push_back(graphConstructionTime + t.elapsed());

	t.reset();
	MaxSuccessGreedySolver_t maxsuccess_gsolver(g);
	maxsuccess_gsolver.MSGreedy_search(g);
	maxsuccess_gsolver.printAll();
	std::cout << "MSG time: " << t.elapsed() << "\n\n";
	planningTime.push_back(graphConstructionTime + t.elapsed());

	t.reset();
	MaxSuccessExactSolver_t maxsuccess_esolver(g);
	maxsuccess_esolver.MSExact_search(g);
	maxsuccess_esolver.printAll();
	std::cout << "MSE time: " << t.elapsed() << "\n\n";
	planningTime.push_back(graphConstructionTime + t.elapsed());

	t.reset();
	MCRMostCandidateSolver_t mcrmc_gsolver(g, g.getStart(), g.getGoalSet());
	mcrmc_gsolver.MCRMCGreedy_search(g);
	mcrmc_gsolver.printAll();
	std::cout << "MCR-MLC time: " << t.elapsed() << "\n\n";
	planningTime.push_back(graphConstructionTime + t.elapsed());

	// write in the trajectories
	traj = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/Astartraj.txt";
	astar_solver.writeTrajectory(traj);
	traj = "../iros_20_motoman_experiment/data//" + img_idx + "/roadmap" + roadmap_idx + "/MCRGtraj.txt";
	mcr_gsolver.writeTrajectory(traj);
	traj = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/MCREtraj.txt";
	mcr_esolver.writeTrajectory(traj);
	traj = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/MSGtraj.txt";
	maxsuccess_gsolver.writeTrajectory(traj);
	traj = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/MSEtraj.txt";
	maxsuccess_esolver.writeTrajectory(traj);
	traj = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/MCRMCGtraj.txt";
	mcrmc_gsolver.writeTrajectory(traj);

	// write in time
	std::string time_file = "../iros_20_motoman_experiment/data/" + img_idx + "/roadmap" + roadmap_idx + "/times.txt";

	outFile.open(time_file);
	if (outFile.is_open())
	{
		for (auto const &t : planningTime)
		{
			outFile << t << "\n";
		}
	}
	outFile.close();	

	return 0;

}