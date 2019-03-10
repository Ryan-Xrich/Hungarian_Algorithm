#pragma once
#ifndef KM_MATCH_H
#define KM_MATCH_H

#include<iostream>
#include<vector>
#include<algorithm>
#include<fstream>
#include<cmath>
#include<ctime>
#include<string>

class KM_MATCH
{
	friend class PARTNER;
public:
	KM_MATCH() { this->matrixSize = 0; }
	KM_MATCH(const size_t & s);	// Initialise an empty matching with the cost matrix size
	~KM_MATCH();


	size_t matrixSize;
	std::vector<std::vector<float>> edge;	// Edge cost matrix
	std::vector<std::vector<bool>> subgraph;// Equality subgraph for Hungarian tree
	std::vector<bool> x_on_path;	// On-path sign
	std::vector<bool> y_on_path;	// On-path sign
	std::vector<int> path;			// Store the augment path
	std::vector<float> A, B;		// Dual variables to assign feasible values

	void Initialise();				// Initialise all other variables using the matrixsize
	size_t GetMatrix(const char & mode);	// Return the size of the cost matrix, 0 for failed

	// Generate a random matrix for static Hungarian algorithm. Accept two variables: matrix size and max cost.
	void GenerateMatrix(const unsigned int & sizeM, const float & maxCost);

	// Generate a random matrix for incremental Hungarian algorithm. Accept one variable: max cost.
	void GenerateMatrix(const float & maxCost);

	// Generate a random matrix for dynamic Hungarian algorithm. Accept three variables: dynamic mode, max cost and ratio of change.
	void GenerateMatrix(const char & mode, const float & maxCost, const unsigned int & changeRatio);

	void SaveMatrix();				// Save the cost matrix into a file
	void SaveResult();				// Save the result matching into a file
	void PrintResult();				// Print the result matching on screen
	bool FindAugmentPath(int xi);	// Function to find the augment path
	void ResetMatchPath();			// Reset all paths
	void ClearOnPathSign();			// Reset all on-path signs
	void ClearAll();				// Warning!!! This function will reset all contents!

	bool KuhnMunkresMatch();		// Static Hungarian algorithm
	void IncrInitialise();			// Initialisation for incremental Hungarian algorithm
	bool IncrAssignment();			// Incremental Hungarian algorithm

	bool RowMatch(const unsigned int & iNew);	// Dynamic Hungarian algorithm with a row change
	bool ColumnMatch(const unsigned int & jNew);// Dynamic Hungarian algorithm with a column change
	bool SingleMatch(const unsigned int & iNew, const unsigned int & jNew, const float & cOld);// Dynamic Hungarian algorithm with a value change

	bool isSqrt(const int & n);		// Check if the number has root square

	// Count the number of changes in the matrix, number of changes saved in rowCount and columnCount
	void FindMatrixChange(const std::vector<float> & temp, std::vector<bool> & row, std::vector<bool> & column, unsigned int & rowCount, unsigned int & columnCount);
};

#endif // !KM_MATCH_H