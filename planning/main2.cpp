#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <queue>
#include <cstring>
#include <algorithm>


bool check_subset(std::vector<int> set, std::vector<int> subset)
{
	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
}

int main(int argc, char** argv)
{
	std::vector<int> set1{3,4,1,2,8};
	std::vector<int> set2{3,1,8};
	std::sort(set1.begin(), set1.end());
	std::sort(set2.begin(), set2.end());
	for (auto const &e : set1)
	{
		std::cout << e << " ";
	}
	std::cout << "\n";
	for (auto const &e : set2)
	{
		std::cout << e << " ";
	}
	std::cout << "\n";



	if (check_subset(set1, set2) == true)
		std::cout << "True\n";
	else
		std::cout << "False\n";

	return 0;
}