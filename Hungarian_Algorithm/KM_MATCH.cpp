#include "KM_MATCH.h"
#include "pch.h"

// Only used for fcn_test
// Initialize an empty matching with the cost matrix size
KM_MATCH::KM_MATCH(const size_t & s)
{
	unsigned int i;
	std::vector<float> temp_f;
	std::vector<bool> temp_b;

	this->matrixSize = s;

	for (i = 0; i < matrixSize; i++)
	{
		temp_f.push_back(0);
		temp_b.push_back(0);
		this->x_on_path.push_back(0);
		this->y_on_path.push_back(0);
		this->path.push_back(-1);
		this->A.push_back(0);
		this->B.push_back(99999);
	}
	for (i = 0; i < matrixSize; i++)
	{
		this->edge.push_back(temp_f);
		this->subgraph.push_back(temp_b);
	}
}

KM_MATCH::~KM_MATCH()
{
	// Variables are all from Standard Template Library, no need to destruct.
}

// Initialize all other variables using the matrixsize
void KM_MATCH::Initialise()
{
	unsigned int i;
	std::vector<bool> temp_b;
	x_on_path.clear();
	y_on_path.clear();
	path.clear();
	A.clear();
	B.clear();

	// Assign the initial values
	for (i = 0; i < matrixSize; i++)
	{
		temp_b.push_back(0);
		this->x_on_path.push_back(0);
		this->y_on_path.push_back(0);
		this->path.push_back(-1);
		this->A.push_back(0);
		this->B.push_back(99999);
	}
	for (i = 0; i < matrixSize; i++)
	{
		this->subgraph.push_back(temp_b);
	}
}

// Return the size of the cost matrix, 0 for failed
// -s static Hungarian algorithm
// -i incremental Hungarian algorithm
// -d dynamic Hungarian algorithm
size_t KM_MATCH::GetMatrix(const char & mode)
{
	unsigned int i, j;
	std::vector<float> temp;// Used to store the cost matrix from the file
	float num;

	switch (mode)
	{
	case 's':// static Hungarian algorithm
	{
		std::ifstream matrixRead("matrix.dat");
		if (!matrixRead.is_open())// If the matrix file doesn't exist
		{
			std::cout << "File matrix.dat read failed!" << std::endl;
			std::ofstream matrixWrite("matrix.dat");// Create empty file
			matrixWrite.close();
			matrixRead.close();
			matrixSize = 0;
			return 0;
		}
		else
		{
			if (matrixRead.peek() == EOF)// Check if the file is empty
			{
				std::cout << "Empty file!" << std::endl;
				matrixSize = 0;
				return 0;
			}
			else
			{
				temp.clear();
				while (matrixRead.peek() != EOF)// Read all data from the file and store in the vector
				{
					matrixRead >> num;
					temp.push_back(num);
				}
				matrixRead.close();

				if (isSqrt(temp.size()))// Check if the matrix is square
				{
					matrixSize = (unsigned int) sqrt(temp.size());// Assign the valid matrix size
					for (i = 0; i < matrixSize; i++)// Copy the cost matrix from the temp to the edge cost matrix
					{
						std::vector<float> temp_f(temp.begin() + i * matrixSize, temp.begin() + (i + 1) * matrixSize);
						edge.push_back(temp_f);
					}
					Initialise();// Initialize all other variables using the matrixsize
				}
				else
				{
					std::cout << "Matrix is not square!" << std::endl;
					matrixSize = 0;
					return 0;
				}
				std::cout << "File matrix.dat read successful!\n" << std::endl;
				if (KuhnMunkresMatch())// Do static Hungarian algorithm
					PrintResult();// Print the matching on the screen
				else
					std::cout << "Assignment error!" << std::endl;
			}
		}
		break;
	}
	case 'i':// incremental Hungarian algorithm
	{
		if (matrixSize = 0)
			return 0;
		std::ifstream incrementalMatrixRead("incremental_matrix.dat");
		if (!incrementalMatrixRead.is_open())
		{
			std::cout << "File incremental_matrix.dat read failed!" << std::endl;
			std::ofstream incrementalMatrixWrite("incremental_matrix.dat");
			incrementalMatrixWrite.close();
			incrementalMatrixRead.close();
			matrixSize = 0;
			return 0;
		}
		else
		{
			if (incrementalMatrixRead.peek() == EOF)// Check if the file is empty
			{
				std::cout << "Empty file!" << std::endl;
				matrixSize = 0;
				return 0;
			}
			else
			{
				temp.clear();
				while (incrementalMatrixRead.peek() != EOF)// Read all data from the file and store in the vector
				{
					incrementalMatrixRead >> num;
					temp.push_back(num);
				}
				incrementalMatrixRead.close();

				if (isSqrt(temp.size()) && sqrt(temp.size()) == (matrixSize + 1))// Check if the matrix is square
				{
					for (i = 0; i < matrixSize; i++)
					{
						for (j = 0; j < matrixSize; j++)
						{
							if (temp.at(i*(matrixSize + 1) + j) != edge[i][j])// Compare the original data with the data from the file
							{
								std::cout << "Matrix is not valid!" << std::endl;
								matrixSize = 0;
								return 0;
							}
						}
						edge[i].push_back(temp[i*matrixSize]);
					}
					matrixSize++;
					// Copy the cost matrix from the temp to the edge cost matrix
					std::vector<float> temp_f(temp.begin() + (matrixSize - 1)*matrixSize, temp.begin() + matrixSize * matrixSize);
					edge.push_back(temp_f);
				}
				else
				{
					std::cout << "Matrix is not valid!" << std::endl;
					matrixSize = 0;
					return 0;
				}
				std::cout << "\nFile incremental_matrix.dat read successful!\n" << std::endl;
				IncrInitialise();// Modify other parameters
				if (IncrAssignment())// Do incremental Hungarian algorithm
					PrintResult();// Print the re-matching on the screen
				else
					std::cout << "Assignment error!" << std::endl;
			}
		}
		break;
	}
	case 'd':// dynamic Hungarian algorithm
	{
		if (matrixSize = 0)
			return 0;
		std::vector<bool> row(matrixSize);// Check if the rows have been changed
		std::vector<bool> column(matrixSize);// Check if the columns have been changed
		unsigned int rowCount(0), columnCount(0);// Store the number of change in row and column
		float cOld(99999);// Store the old cost for single value change

		std::ifstream dynamicMatrixRead("dynamic_matrix.dat");
		if (!dynamicMatrixRead.is_open())
		{
			std::cout << "File dynamic_matrix.dat read failed!" << std::endl;
			std::ofstream dynamicMatrixWrite("dynamic_matrix.dat");
			dynamicMatrixWrite.close();
			dynamicMatrixRead.close();
			return 0;
		}
		else
		{
			if (dynamicMatrixRead.peek() == EOF)
			{
				std::cout << "Empty file!" << std::endl;
				return 0;
			}
			else
			{
				temp.clear();
				while (dynamicMatrixRead.peek() != EOF)// Read all data from the file and store in the vector
				{
					dynamicMatrixRead >> num;
					temp.push_back(num);
				}
				dynamicMatrixRead.close();

				if (isSqrt(temp.size()) && matrixSize == sqrt(temp.size()))// Check if the matrix is square
				{
					// Count the number of changes in the matrix, number of changes saved in rowCount and columnCount
					FindMatrixChange(temp, row, column, rowCount, columnCount);

					while (!(rowCount == 0 && columnCount == 0))// Repeat the dynamic Hungarian algorithm until all changes are re-matched
					{
						if (rowCount == 1 && columnCount == 1)// Single change value
						{
							for (i = 0; i < matrixSize; i++)
							{
								for (j = 0; j < matrixSize; j++)
								{
									if (row.at(i) == 1 && column.at(j) == 1)
									{
										cOld = edge[i][j];// Store the old cost
										edge[i][j] = temp.at(i*matrixSize + j);// Assign the new cost
										SingleMatch(i, j, cOld);// Dynamic Hungarian algorithm with single value change re-matching
										row[i] = 0;
										column[j] = 0;
										rowCount--;
										columnCount--;
									}
								}
							}
						}
						else
						{
							if (rowCount < columnCount && rowCount != 0)// Row change
							{
								for (i = 0; i < matrixSize; i++)
								{
									if (row.at(i) == 1)
									{
										for (j = 0; j < matrixSize; j++)// Copy the changed row values into the edge cost matrix
										{
											edge[i][j] = temp.at(i*matrixSize + j);
										}
										RowMatch(i);// Dynamic Hungarian algorithm with a single row change
										FindMatrixChange(temp, row, column, rowCount, columnCount);
										break;
									}
								}
							}
							else
							{
								if (rowCount >= columnCount && columnCount != 0)// Column change
								{
									for (j = 0; j < matrixSize; j++)
									{
										if (column.at(j) == 1)
										{
											for (i = 0; i < matrixSize; i++)// Copy the changed column values into the edge cost matrix
											{
												edge[i][j] = temp.at(i*matrixSize + j);
											}
											ColumnMatch(j);// Dynamic Hungarian algorithm with a single column change
											FindMatrixChange(temp, row, column, rowCount, columnCount);
											break;
										}
									}
								}
							}
						}
					}
					std::cout << "\nFile dynamic_matrix.dat read successful!\n" << std::endl;
					PrintResult();
					return matrixSize;
				}
				else
				{
					std::cout << "Matrix is not valid!" << std::endl;
					matrixSize = 0;
					return 0;
				}
			}
		}
		break;
	}
	default:
	{
		matrixSize = 0;
	}
	}// End of switch cases

	return matrixSize;
}

// Generate a random matrix for static Hungarian algorithm
// Accept two variables: matrix size and max cost.
void KM_MATCH::GenerateMatrix(const unsigned int & sizeM, const float & maxCost)
{
	unsigned int i, j;
	std::vector<float> temp;// Temporarily store the random float vector 

	srand((unsigned)time(0));// Random Seed

	if (maxCost >= 0)// Check if the max cost parameter is valid
	{
		matrixSize = sizeM;// Assign the matrix size
		for (j = 0; j < matrixSize; j++)
		{
			temp.push_back(rand() % (int)maxCost);// Initialize the temp vector
		}
		edge.push_back(temp);// Store the random values to the edge cost matrix

		for (i = 0; i < matrixSize - 1; i++)
		{
			for (j = 0; j < matrixSize; j++)
			{
				temp[j] = rand() % (int)maxCost;
			}
			edge.push_back(temp);// Store the random values to the edge cost matrix
		}

		Initialise();

		//system("pause");// Only used for speed test

		if (KuhnMunkresMatch())// Do static Hungarian algorithm
			PrintResult();
		else
			std::cout << "Assignment error!" << std::endl;
	}
	else
	{
		std::cout << "Matrix generating error!" << std::endl;
		matrixSize = 0;
		return;
	}
}

// Generate a random matrix for incremental Hungarian algorithm. Accept one variable: max cost.
void KM_MATCH::GenerateMatrix(const float & maxCost)
{
	unsigned int i;
	std::vector<float> temp;// Temporarily store the random float vector 

	srand((unsigned)time(0));// Random Seed

	if (maxCost >= 0)// Check if the max cost parameter is valid
	{
		matrixSize++;
		for (i = 0; i < matrixSize - 1; i++)
		{
			edge[i].push_back(rand() % (int)maxCost);
			temp.push_back(rand() % (int)maxCost);
		}
		temp.push_back(rand() % (int)maxCost);
		edge.push_back(temp);// Store the random values to the edge cost matrix

		IncrInitialise();
		if (IncrAssignment())// Do incremental Hungarian algorithm
			PrintResult();
		else
			std::cout << "Assignment error!" << std::endl;
	}
	else
	{
		std::cout << "Matrix generating error!" << std::endl;
		matrixSize = 0;
		return;
	}
}

// Generate a random matrix for dynamic Hungarian algorithm. Accept three variables: dynamic mode, max cost and ratio of change.
void KM_MATCH::GenerateMatrix(const char & mode, const float & maxCost, const unsigned int & changeRatio)
{
	unsigned int i, j;
	std::vector<float> temp;// Temporarily store the random float vector

	srand((unsigned)time(0));// Random Seed

	if (maxCost >= 0 && changeRatio >= 0)// Check if the max cost and change ratio parameters are valid
	{
		switch (mode)
		{
		case 'r':
		{
			// Now, perform dynamic Hungarian algorithm with a single row change
			unsigned int row = rand() % matrixSize;

			for (i = 0; i < matrixSize; i++)
			{
				if (i == row)
				{
					for (j = 0; j < matrixSize; j++)
					{
						edge[i][j] = rand() % (int)maxCost;// Modify the edge cost of this row
					}
					RowMatch(row);// Do dynamic Hungarian algorithm with one row change
					break;
				}
			}
			std::cout << "\nDynamic matrix generation successful!\nSingle row change:\n" << std::endl;
			PrintResult();
			break;
		}
		case 'c':
		{
			// Now, perform dynamic Hungarian algorithm with a single column change
			unsigned int column = rand() % matrixSize;

			for (j = 0; j < matrixSize; j++)
			{
				if (j == column)
				{
					for (i = 0; i < matrixSize; i++)
					{
						edge[i][j] = rand() % (int)maxCost;// Modify the edge cost of this column
					}

					//system("pause");// Only used for speed test
					
					ColumnMatch(column);// Do dynamic Hungarian algorithm with one column change
					break;
				}
			}
			std::cout << "\nDynamic matrix generation successful!\nSingle column change:\n" << std::endl;
			PrintResult();
			break;
		}
		case 's':
		{
			// Now, perform dynamic Hungarian algorithm with a single value change
			float cOld;
			unsigned int row = rand() % matrixSize;
			unsigned int column = rand() % matrixSize;

			for (i = 0; i < matrixSize; i++)
			{
				for (j = 0; j < matrixSize; j++)
				{
					if (row == i && column == j)
					{
						cOld = edge[i][j];
						edge[i][j] = rand() % (int)maxCost;// Modify the edge cost of this single value

						//system("pause");// Only used for speed test

						SingleMatch(i, j, cOld);// Do dynamic Hungarian algorithm with single value change
					}
				}
			}
			std::cout << "\nDynamic matrix generation successful!\nSingle value change:\n" << std::endl;
			PrintResult();
			break;
		}
		default:
		{
			std::vector<float> temp;
			double ratio = double(changeRatio / 100);// Calculate the change ratio

			if (ratio >= 1)// If every values in the cost matrix are changed, do one static Hungarian algorithm
			{
				unsigned int tempSize = matrixSize;
				ClearAll();// This function will delete all data of this object
				GenerateMatrix(tempSize, maxCost);// Do one static Hungarian algorithm
			}
			else
			{
				std::vector<bool> row(matrixSize);
				std::vector<bool> column(matrixSize);
				unsigned int rowCount(0), columnCount(0);
				float cOld(99999);

				for (i = 0; i < matrixSize; i++)
				{
					for (j = 0; j < matrixSize; j++)
					{
						if ((rand() % 100) <= maxCost)// Use random number to determine which numbers need to be changed
						{
							temp.push_back(rand() % (int)maxCost);// Modify the edge cost of this single value
						}
						else
						{
							temp.push_back(edge[i][j]);
						}
					}
				}

				FindMatrixChange(temp, row, column, rowCount, columnCount);

				while (!(rowCount == 0 && columnCount == 0))
				{
					if (rowCount == 1 && columnCount == 1)// Single change value
					{
						for (i = 0; i < matrixSize; i++)
						{
							for (j = 0; j < matrixSize; j++)
							{
								if (row.at(i) == 1 && column.at(j) == 1)
								{
									cOld = edge[i][j];
									edge[i][j] = temp.at(i*matrixSize + j);
									SingleMatch(i, j, cOld);// Do dynamic Hungarian algorithm with single value change
									row[i] = 0;
									column[j] = 0;
									rowCount--;
									columnCount--;
								}
							}
						}
					}
					else
					{
						if (rowCount < columnCount && rowCount != 0)// Row change
						{
							for (i = 0; i < matrixSize; i++)
							{
								if (row.at(i) == 1)
								{
									for (j = 0; j < matrixSize; j++)
									{
										edge[i][j] = temp.at(i*matrixSize + j);
									}
									RowMatch(i);// Do dynamic Hungarian algorithm with one row change
									FindMatrixChange(temp, row, column, rowCount, columnCount);
									break;
								}
							}
						}
						else
						{
							if (rowCount >= columnCount && columnCount != 0)// Column change
							{
								for (j = 0; j < matrixSize; j++)
								{
									if (column.at(j) == 1)
									{
										for (i = 0; i < matrixSize; i++)
										{
											edge[i][j] = temp.at(i*matrixSize + j);
										}
										ColumnMatch(j);// Do dynamic Hungarian algorithm with one column change
										FindMatrixChange(temp, row, column, rowCount, columnCount);
										break;
									}
								}
							}
						}
					}
				}
				std::cout << "\nDynamic matrix generation successful!\n" << std::endl;
				PrintResult();
			}
			break;
		}
		}
	}
	else
	{
		std::cout << "Matrix generating error!" << std::endl;
		matrixSize = 0;
		return;
	}
}

// Save the cost matrix into a file
void KM_MATCH::SaveMatrix()
{
	unsigned int loopTime = 0;
	char fileName[32];
	struct tm t;
	time_t now;
	time(&now);				// Get system time
	localtime_s(&t, &now);	// Get local time

	strftime(fileName, sizeof(fileName), "%Y-%m-%d_%H.%M.%S", &t);	// Format the date and time
	strcat_s(fileName, sizeof(fileName), ".dat");					// Format the file name

	std::string temp = fileName;// Generate the string representing the file name to be write
	std::ifstream matrixRead(temp);
	while (matrixRead.is_open())
	{
		temp += (char)('0' + loopTime);// Deal with the existing file problem
		loopTime++;
		matrixRead.close();
		std::ifstream matrixRead(temp);
	}
	matrixRead.close();

	std::ofstream matrixWrite(temp, std::ios::out);// Normal condition, prepare for writing data to the file
	for (unsigned int i = 0; i < matrixSize; i++)
	{
		for (unsigned int j = 0; j < matrixSize; j++)
		{
			matrixWrite << edge[i][j];
			if (j < (matrixSize - 1))
				matrixWrite << " ";
			else
				matrixWrite << "\n";
		}
	}
	matrixWrite.close();
	std::cout << "Matrix saved at " << temp << std::endl;
}

// Save the result matching into a file
void KM_MATCH::SaveResult()
{
	unsigned int loopTime = 0;
	char fileName[32];
	struct tm t;
	time_t now;
	time(&now);				// Get system time
	localtime_s(&t, &now);	// Get local time
	
	strftime(fileName, sizeof(fileName), "%Y-%m-%d_%H.%M.%S", &t);	// Format the date and time
	strcat_s(fileName, sizeof(fileName), ".txt");					// Format the file name

	std::string temp = fileName;
	std::ifstream matrixRead(temp);
	while (matrixRead.is_open())
	{
		temp += (char)('0' + loopTime);// Deal with the existing file problem
		loopTime++;
		matrixRead.close();
		std::ifstream matrixRead(temp);
	}
	matrixRead.close();

	std::ofstream matrixWrite(temp, std::ios::out);// Normal condition, prepare for writing data to the file
	float cost = 0;
	for (unsigned int i = 0; i < matrixSize; i++)
	{
		cost += edge[path[i]][i];
		matrixWrite << "x" << path[i] + 1 << " <----> y" << i + 1 << std::endl;
	}
	matrixWrite << "\nTotal cost is " << cost << "." << std::endl;
	matrixWrite.close();
	std::cout << "Result matching saved at " << fileName << std::endl;
}

// Print the result matching on screen
void KM_MATCH::PrintResult()
{
	double cost = 0;
	for (unsigned int i = 0; i < matrixSize; i++)
	{
		cost += edge[(path[i])][i];
		std::cout << "x" << path[i] + 1 << " <----> y" << i + 1 << std::endl;
	}
	std::cout << "\nTotal cost is " << cost << "." << std::endl;
}

// Function to find the augment path
bool KM_MATCH::FindAugmentPath(int xi)
{
	for (unsigned int yj = 0; yj < matrixSize; yj++)
	{
		// yj not on augment path && exist an edge between xi and yj
		if (!this->y_on_path[yj] && this->subgraph[xi][yj])
		{
			this->y_on_path[yj] = true;// Set yj on the augment path
			int xt = this->path[yj];// Store path connected to yj
			this->path[yj] = xi;
			if (xt == -1 || FindAugmentPath(xt))// If yj doesn't have an augment path, or there's a path on xt
			{
				return true;
			}
			this->path[yj] = xt;// Get back the previous path connected to yj
			if (xt != -1)// If yj does have an augment path
				this->x_on_path[xt] = true;// Set xi on the augment path
		}
	}
	return false;
}

// Reset all paths
void KM_MATCH::ResetMatchPath()
{
	for (unsigned int i = 0; i < matrixSize; i++)
	{
		path[i] = -1;
	}
}

// Reset all on-path signs
void KM_MATCH::ClearOnPathSign()
{
	for (unsigned int i = 0; i < matrixSize; i++)
	{
		x_on_path[i] = false;
		y_on_path[i] = false;
	}
}

// Warning!!! This function will reset all contents!
void KM_MATCH::ClearAll()
{
	unsigned int i;
	for (i = 0; i < matrixSize; i++)
	{
		edge[i].clear();
		subgraph[i].clear();
	}
	x_on_path.clear();
	y_on_path.clear();
	path.clear();
	A.clear();
	B.clear();
	matrixSize = 0;
}

// Static Hungarian algorithm
bool KM_MATCH::KuhnMunkresMatch()
{
	// Begin with an empty matching
	unsigned int i, j;

	// Assign feasible values to the dual variables
	for (i = 0; i < matrixSize; i++)
	{
		for (j = 0; j < matrixSize; j++)
		{
			B[j] = std::min(B[j], edge[i][j]);
		}
	}
	// The elements in B are the smallest number in each column

	unsigned int loopTime = 0;

	while (loopTime <= (matrixSize * matrixSize * matrixSize))// To avoid infinite loop
	{
		float dx = 99999;// Assign the largest dx initial value
		for (i = 0; i < matrixSize; i++)// Update the sub graph
		{
			for (j = 0; j < matrixSize; j++)
			{
				subgraph[i][j] = ((A[i] + B[j]) >= edge[i][j]);
			}
		}

		int match = 0;
		this->ResetMatchPath();
		for (unsigned int xi = 0; xi < matrixSize; xi++)
		{
			this->ClearOnPathSign();
			if (this->FindAugmentPath(xi))// If there's an augment path to xi
			{
				match++;// Find one match
			}
			else
			{
				x_on_path[xi] = true;
				break;
			}
		}

		if (match == matrixSize)// Count the number of matching
		{
			return true;
		}

		for (i = 0; i < matrixSize; i++)
		{
			for (j = 0; j < matrixSize; j++)
			{
				if (x_on_path[i] && !y_on_path[j])
				{
					dx = std::min(dx, edge[i][j] - A[i] - B[j]);// Update dx
				}
			}
		}

		dx = dx / 2;// Calculate the final dx value
		for (i = 0; i < matrixSize; i++)// Modify the dual variables
		{
			if (x_on_path[i])// If xi on augment path
				A[i] += dx;
			else
				A[i] -= dx;
			if (y_on_path[i])// If yj on augment path
				B[i] -= dx;
			else
				B[i] += dx;
		}

		loopTime++;
	}
	return false;
}

// Initialisation for incremental Hungarian algorithm
void KM_MATCH::IncrInitialise()
{
	unsigned int i;
	std::vector<bool> temp_b;

	for (i = 0; i < matrixSize; i++)
	{
		temp_b.push_back(0);
		if (i < (matrixSize - 1))
		{
			subgraph[i].push_back(0);
		}
		else
		{
			subgraph.push_back(temp_b);
		}
	}
	x_on_path.push_back(0);
	y_on_path.push_back(0);
	path.push_back(-1);
}

// Only used for fcn_test
/*
void KM_MATCH::IncrInitialise()
{
	matrixSize++;

	unsigned int i, j;
	std::vector<float> temp_f;
	std::vector<bool> temp_b;

	for (i = 0; i < matrixSize; i++)
	{
		temp_f.push_back(0);
		temp_b.push_back(0);
		if (i < (matrixSize - 1))
		{
			edge[i].push_back(0);
			subgraph[i].push_back(0);
		}
		else
		{
			edge.push_back(temp_f);
			subgraph.push_back(temp_b);
		}
	}
	x_on_path.push_back(0);
	y_on_path.push_back(0);
	path.push_back(-1);
}
*/

// Incremental Hungarian algorithm
bool KM_MATCH::IncrAssignment()
{
	// Perform initialization:

	// Begin with an perfect matching
	unsigned int i, j;
	float Aplus = 99999;	// An+1
	float Bplus = 99999;	// Bn+1

	// Assign feasible values to the dual variable B. Note: B must be modified before A.
	for (i = 0; i < (matrixSize - 1); i++)
	{
		Bplus = std::min(Bplus, edge[i][matrixSize - 1] - A[i]);
	}

	Bplus = std::min(Bplus, edge[matrixSize - 1][matrixSize - 1]);
	B.push_back(Bplus);// Store the dual variable to the vector

	// Assign feasible values to the dual variable A
	for (j = 0; j < matrixSize; j++)
	{
		Aplus = std::min(Aplus, edge[matrixSize - 1][j] - B[j]);
	}
	A.push_back(Aplus);// Store the dual variable to the vector


	unsigned int loopTime = 0;

	while (loopTime <= (matrixSize * matrixSize * matrixSize))// To avoid infinite loop
	{
		float dx = 99999;// Assign the largest dx initial value
		for (i = 0; i < matrixSize; i++)// Update the sub graph
		{
			for (j = 0; j < matrixSize; j++)
			{
				subgraph[i][j] = ((A[i] + B[j]) >= edge[i][j]);
			}
		}

		int match = 0;
		this->ResetMatchPath();
		for (unsigned int xi = 0; xi < matrixSize; xi++)
		{
			this->ClearOnPathSign();
			if (this->FindAugmentPath(xi))// If there's an augment path to xi
			{
				match++;// Find one match
			}
			else
			{
				x_on_path[xi] = true;
				break;
			}
		}

		if (match == matrixSize)// Count the number of matching
		{
			return true;
		}

		for (i = 0; i < matrixSize; i++)
		{
			for (j = 0; j < matrixSize; j++)
			{
				if (x_on_path[i] && !y_on_path[j])
				{
					dx = std::min(dx, edge[i][j] - A[i] - B[j]);// Update dx
				}
			}
		}

		dx = dx / 2;// Calculate the final dx value
		for (i = 0; i < matrixSize; i++)// Modify the dual variables
		{
			if (x_on_path[i])// If xi on augment path
				A[i] += dx;
			else
				A[i] -= dx;
			if (y_on_path[i])// If yj on augment path
				B[i] -= dx;
			else
				B[i] += dx;
		}

		loopTime++;
	}
	return false;
}

// Dynamic Hungarian algorithm with a row change
bool KM_MATCH::RowMatch(const unsigned int & iNew)
{
	unsigned int i, j;
	float ANew = 99999;

	if (iNew >= matrixSize)// Ensure the change is in the matrix range
		return false;
	else
	{
		for (i = 0; i < matrixSize; i++)
		{
			if (path[i] == iNew)// Find path_i*
			{
				path[i] = -1;// Remove edge path_i*
				for (j = 0; j < matrixSize; j++)
				{
					ANew = std::min(ANew, edge[iNew][j] - B[j]);// Modify the dual variable Ai*
				}
				A[iNew] = ANew;
				break;
			}
		}
		unsigned int loopTime = 0;

		while (loopTime <= (matrixSize * matrixSize * matrixSize))// To avoid infinite loop
		{
			float dx = 99999;// Assign the largest dx initial value
			for (i = 0; i < matrixSize; i++)// Update the sub graph
			{
				for (j = 0; j < matrixSize; j++)
				{
					subgraph[i][j] = ((A[i] + B[j]) >= edge[i][j]);
				}
			}

			int match = 0;
			this->ResetMatchPath();
			for (unsigned int xi = 0; xi < matrixSize; xi++)
			{
				this->ClearOnPathSign();
				if (this->FindAugmentPath(xi))// If there's an augment path to xi
				{
					match++;// Find one match
				}
				else
				{
					x_on_path[xi] = true;
					break;
				}
			}

			if (match == matrixSize)// Count the number of matching
			{
				return true;
			}

			for (i = 0; i < matrixSize; i++)
			{
				for (j = 0; j < matrixSize; j++)
				{
					if (x_on_path[i] && !y_on_path[j])
					{
						dx = std::min(dx, edge[i][j] - A[i] - B[j]);// Update dx
					}
				}
			}

			dx = dx / 2;// Calculate the final dx value
			for (i = 0; i < matrixSize; i++)// Modify the dual variables
			{
				if (x_on_path[i])// If xi on augment path
					A[i] += dx;
				else
					A[i] -= dx;
				if (y_on_path[i])// If yj on augment path
					B[i] -= dx;
				else
					B[i] += dx;
			}

			loopTime++;
		}
		return false;
	}
}

// Dynamic Hungarian algorithm with a column change
bool KM_MATCH::ColumnMatch(const unsigned int & jNew)
{
	unsigned int i, j;
	float BNew = 99999;

	if (jNew >= matrixSize)// Ensure the change is in the matrix range
		return false;
	else
	{
		path[jNew] = -1;// Remove edge path_j*
		for (i = 0; i < matrixSize; i++)
		{
			BNew = std::min(BNew, edge[i][jNew] - A[i]);// Modify the dual variable Bj*
		}
		B[jNew] = BNew;

		unsigned int loopTime = 0;

		while (loopTime <= (matrixSize * matrixSize * matrixSize))// To avoid infinite loop
		{
			float dx = 99999;// Assign the largest dx initial value
			for (i = 0; i < matrixSize; i++)// Update the sub graph
			{
				for (j = 0; j < matrixSize; j++)
				{
					subgraph[i][j] = ((A[i] + B[j]) >= edge[i][j]);
				}
			}

			int match = 0;
			this->ResetMatchPath();
			for (unsigned int xi = 0; xi < matrixSize; xi++)
			{
				this->ClearOnPathSign();
				if (this->FindAugmentPath(xi))// If there's an augment path to xi
				{
					match++;// Find one match
				}
				else
				{
					x_on_path[xi] = true;
					break;
				}
			}

			if (match == matrixSize)// Count the number of matching
			{
				return true;
			}

			for (i = 0; i < matrixSize; i++)
			{
				for (j = 0; j < matrixSize; j++)
				{
					if (x_on_path[i] && !y_on_path[j])
					{
						dx = std::min(dx, edge[i][j] - A[i] - B[j]);// Update dx
					}
				}
			}

			dx = dx / 2;// Calculate the final dx value
			for (i = 0; i < matrixSize; i++)// Modify the dual variables
			{
				if (x_on_path[i])// If xi on augment path
					A[i] += dx;
				else
					A[i] -= dx;
				if (y_on_path[i])// If yj on augment path
					B[i] -= dx;
				else
					B[i] += dx;
			}

			loopTime++;
		}
		return false;
	}
}

// Dynamic Hungarian algorithm with a value change
bool KM_MATCH::SingleMatch(const unsigned int & iNew, const unsigned int & jNew, const float & cOld)
{
	unsigned int i, j;
	float BNew = 99999;

	if (iNew >= matrixSize || jNew >= matrixSize)// Ensure the change is in the matrix range
		return false;
	else
	{
		if (edge[iNew][jNew] > cOld && path[jNew] == iNew)// If C_New > C_Old, and exists an augment path between j* and i*
		{
			path[jNew] = -1;// Disconnect all paths from j*
		}
		else
		{
			if (edge[iNew][jNew] < cOld && A[iNew] + B[jNew] > edge[iNew][jNew])// If C_New < C_Old, and Ai* + Bj* > C_New
			{
				for (i = 0; i < matrixSize; i++)
				{
					BNew = std::min(BNew, edge[i][jNew] - A[i]);// Calculate Bj*
				}
				B[jNew] = BNew;
			}
			if (path[jNew] != iNew)// If there doesn't exist an augment path between j* and i*
			{
				path[jNew] = -1;// Disconnect all paths from j*
			}
		}
		unsigned int loopTime = 0;

		while (loopTime <= (matrixSize * matrixSize * matrixSize))// To avoid infinite loop
		{
			float dx = 99999;// Assign the largest dx initial value
			for (i = 0; i < matrixSize; i++)// Update the sub graph
			{
				for (j = 0; j < matrixSize; j++)
				{
					subgraph[i][j] = ((A[i] + B[j]) >= edge[i][j]);
				}
			}

			int match = 0;
			this->ResetMatchPath();
			for (unsigned int xi = 0; xi < matrixSize; xi++)
			{
				this->ClearOnPathSign();
				if (this->FindAugmentPath(xi))// If there's an augment path to xi
				{
					match++;// Find one match
				}
				else
				{
					x_on_path[xi] = true;
					break;
				}
			}

			if (match == matrixSize)// Count the number of matching
			{
				return true;
			}

			for (i = 0; i < matrixSize; i++)
			{
				for (j = 0; j < matrixSize; j++)
				{
					if (x_on_path[i] && !y_on_path[j])
					{
						dx = std::min(dx, edge[i][j] - A[i] - B[j]);// Update dx
					}
				}
			}

			dx = dx / 2;// Calculate the final dx value
			for (i = 0; i < matrixSize; i++)// Modify the dual variables
			{
				if (x_on_path[i])// If xi on augment path
					A[i] += dx;
				else
					A[i] -= dx;
				if (y_on_path[i])// If yj on augment path
					B[i] -= dx;
				else
					B[i] += dx;
			}

			loopTime++;
		}
		return false;
	}
}

// Check if the number has root square
bool KM_MATCH::isSqrt(const int & n)
{
	int a = int(sqrt(n) + 0.5);
	return a * a == n;
}

// Count the number of changes in the matrix, number of changes saved in rowCount and columnCount
void KM_MATCH::FindMatrixChange(const std::vector<float> & temp, std::vector<bool> & row, std::vector<bool> & column, unsigned int & rowCount, unsigned int & columnCount)
{
	unsigned int i, j;
	rowCount = 0;
	columnCount = 0;

	for (i = 0; i < matrixSize; i++)// Initialize the row and column vectors
	{
		row[i] = 0;
		column[i] = 0;
	}

	for (i = 0; i < matrixSize; i++)
	{
		for (j = 0; j < matrixSize; j++)
		{
			if (temp.at(i*matrixSize + j) != edge[i][j])// Compare the values with the edge cost matrix
			{
				row[i] = 1;
				column[j] = 1;
			}
		}
	}

	for (i = 0; i < matrixSize; i++)// Update the row and column count numbers
	{
		if (row[i] == 1)
		{
			rowCount++;
		}
		if (column[i] == 1)
		{
			columnCount++;
		}
	}
}
