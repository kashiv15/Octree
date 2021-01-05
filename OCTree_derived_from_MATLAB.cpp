// OCTree_for_Mapping_Paper.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// Implemetation of Octree in c++ 
#include "OCTree_derived_from_MATLAB.h"

// Octree class 
class Octree {

	// Properties of an octree.
	Eigen::MatrixXf points, binBoundaries, newBounds = Eigen::MatrixXf::Zero(8, 6);;
	int binCount, binCapacity, maxDepth;
	float minSize, maxSize;
	std::vector<int> binDepths, binParents;
	string style ;

	// Defining a data type, Eigen::MatrixXb, i.e., dynamic matrix of booleans
	typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;
	typedef Eigen::Matrix<unsigned short int, Eigen::Dynamic, Eigen::Dynamic> MatrixXusi;
	typedef Eigen::Matrix<unsigned long int, Eigen::Dynamic, Eigen::Dynamic> MatrixXuli;
	MatrixXusi pointBins;

	std::vector<int> buildNewBoundaries = { 1,2,3,4,5,6,1,2,6,4,5,9,1,5,3,4,8,6,1,5,6,4,8,9,4,2,3,7,5,6,4,2,6,7,5,9,4,5,3,7,8,6,4,5,6,7,8,9 };
	MatrixXb binMap = MatrixXb::Constant(3, 8, 0);

public:

	// Constructor
	Octree()
	{

	}

	// Constructor with argument carrying the number of points in cloud
	Octree(int x, int y, Eigen::MatrixXf *r, string style_)
	{
		// Number of points is "x". 
		// "points" stores the 3D points to be binned by the OCTree: Rows: number of points - Columns: 3
		points = Eigen::MatrixXf::Zero(x, 3);
		// "pointBins" keeps track of where points are stored. Rows: Rows: number of points - Columns: 1
		//pointBins = Eigen::MatrixXf::Zero(x, 1);
		pointBins = MatrixXusi::Constant(x,1,1);
		// "binBoundaries" stores the boundaries of a bin in 3D space using two vertices. Rows: number of bins - Columns: 6 -- Columns 1:3 store first vertex, Columns 4:6 store the second vertex.
		// Note that here, there is only one bin. Later, this matrix is resized to reflect the current number of bins.
		binBoundaries = Eigen::MatrixXf::Zero(1, 6);
		binBoundaries.block(0,0,1,3) = (*r).col(0).transpose();
		//binBoundaries.block(0, 0, 1, 3) = {0.2,3.2,0};
		binBoundaries.block(0,3,1,3)= (*r).col(1).transpose();
		//cout << "binboundaries" << binBoundaries << endl;
		// "binCount" keeps track of the number of bins in the OCTree
		binCount = 1;
		
		// "binDephts" keeps track of how many times a bin has been divided. Rows: number of bins.
		binDepths.resize(binCount);

		// "binParents" keeps track of which bin the current bin originated from.
		binParents.resize(binCount);

		// "binDepths" is initialized to zero, since the first bin has not been divided at this point in the program's execution.
		binDepths[0] = 0;

		// "binParents" is initialized to one, since the first bin can be thought of as originating from itself.
		binParents[0] = 1;

		// "binCapacity" is user-defined, and is used as a condition for dividing a bin. If a bin stores more points than its capacity, it will be divided.
		binCapacity = y;

		// "style" is user-defined, and is used to determine how the edges of the bins are determined.
		style = style_;

		// Properties for all OcTree nodes/leaves
		maxDepth = 1000000;	// Maximum number of times the original bin (the first bin thought of as originating from itself) can be divided.
		minSize = 0.2;			// Minimum edge size in meters.
		maxSize = 1000000;	// Maximum edge size in meters.

		binMap << 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1;

		// Allocate memory, which is a function of the number of points.
		this->preallocateSpace();

		// The first bin is indexed by "1". The function "divide" takes a list of bin indecies.
		std::vector<int> d;
		d.resize(1);
		d[0] = 1;

		// Divide the original bin
		this->divide(d);
		std::cout << "binBoundaries: " << binBoundaries << std::endl;

	}

	void preallocateSpace()
	{
		int numPts = this->points.rows();
		int numBins = numPts;

		if (isfinite((float)this->binCapacity))
		{
			numBins = (int)ceil((2 * numPts) / this->binCapacity)+1;
		}

		this->binDepths.resize(numBins);
		this->binParents.resize(numBins);
		for (unsigned short int i = 0; i < numBins; i++)
		{
			this->binDepths[i] = 0;
			this->binParents[i] = 0;
		}
		this->binBoundaries.conservativeResize(numBins, 6);
		this->binBoundaries.block(1, 0, numBins - 1, 6) = Eigen::MatrixXf::Zero(numBins - 1, 6);
		//cout << "bin" << binBoundaries << endl;
	}

	//void deallocateSpace()
	//{
	//	// finish the deallocate function
	//	for (unsigned long int i = this->binCount; i < this->binDepths.size(); i++)
	//	this->binDepths.resize(numBins);
	//	this->binParents.resize(numBins);
	//	for (unsigned short int i = 0; i < numBins; i++)
	//	{
	//		this->binDepths[i] = 0;
	//		this->binParents[i] = 0;
	//	}
	//	this->binBoundaries.conservativeResize(numBins, 6);
	//	this->binBoundaries.block(1, 0, numBins - 1, 6) = Eigen::MatrixXf::Zero(numBins - 1, 6);
	//}

	void divide(std::vector<int> startingBins)
	{
		unsigned short int binNo, oldCount;
		Eigen::MatrixXf thisBounds = Eigen::MatrixXf::Zero(1, 6), binEdgeSize = Eigen::MatrixXf::Zero(1,3);
		MatrixXb pointBins_eq_binNo = MatrixXb::Constant(this->pointBins.rows(), 1,false);
		std::cout << "here 1" << std::endl;
		float minEdgeSize, maxEdgeSize;

		// Loop over each bin we will consider for division
		for (unsigned short int i = 0; i < startingBins.size(); i++)
		{
			std::cout << "here 1.1" << std::endl;
			binNo = startingBins[i];
			if (this->binDepths[binNo-1] + 1 >= this->maxDepth)
			{
			std::cout << "here 1.2" << std::endl;
				continue;
			}
		std::cout << "here 2" << std::endl;

			thisBounds = this->binBoundaries.block(binNo-1,0,1,6);
			std::cout << "thisBounds: " << thisBounds << std::endl;
			binEdgeSize = thisBounds.block(0, 3, 1, 3) - thisBounds.block(0, 0, 1, 3);
			cout << "binEdgeSize" << binEdgeSize << endl;
			minEdgeSize = binEdgeSize.minCoeff();
			std::cout << "minEdgeSize: " << minEdgeSize<< std::endl;
			maxEdgeSize = binEdgeSize.maxCoeff();
			std::cout << "maxEdgeSize: " << maxEdgeSize<< std::endl;

		std::cout << "here 3" << std::endl;
			if (minEdgeSize < minSize)
				continue;

		std::cout << "here 4" << std::endl;
			oldCount = this->binCount;
			for (unsigned long int j = 0; j < pointBins.rows(); j++)
			{
				pointBins_eq_binNo(j, 0) = pointBins(j, 0) == binNo ? true : false;
			}

		std::cout << "here 5" << std::endl;
			if (pointBins_eq_binNo.nonZeros() > this->binCapacity)
			{
				std::vector<int> newStartingBins = linspace<int>(oldCount, this->binCount, this->binCount - oldCount + 1);
				std::cout << "divide this bin" << std::endl;
				divideBin(binNo, &pointBins_eq_binNo);
				std::cout << "divide new bins" << std::endl;
				divide(newStartingBins);
				continue;
				//std::cin.ignore();
			}
		std::cout << "here 6" << std::endl;
			if (maxEdgeSize > this->maxSize)
			{
				std::vector<int> newStartingBins = linspace<int>(oldCount, this->binCount, this->binCount - oldCount + 1);
				std::cout << "got to end of divide" << std::endl;
				divideBin(binNo, &pointBins_eq_binNo);
				divide(newStartingBins);
				continue;
				//std::cin.ignore();
			}
			
		}

	}

void divideBin(int binNo, MatrixXb* pointBins_eq_binNo_)
{
		// Quickly, write down pointBins_eq_binNo so it is clear which one this function is referring to.
		MatrixXb pointBins_eq_binNo = MatrixXb::Constant(this->pointBins.rows(), 1, false);
		pointBins_eq_binNo = (*pointBins_eq_binNo_);
		unsigned long int pointBins_eq_binNo_nnz = pointBins_eq_binNo.nonZeros(), counter = 0;

		Eigen::MatrixXf thisBinsPoints, oldMin, oldMax, newDiv, minMidMax;
		MatrixXusi binAssignment;
		MatrixXuli binPtMask;
		MatrixXb gtMask = MatrixXb::Constant(pointBins_eq_binNo_nnz, 3, false);
		MatrixXb gtMask_binMap_eq = MatrixXb::Constant(pointBins_eq_binNo_nnz, 3, false);

		thisBinsPoints = Eigen::MatrixXf::Zero(pointBins_eq_binNo_nnz, 3);
		binAssignment = MatrixXusi::Constant(pointBins_eq_binNo_nnz, 1, 0);
		binPtMask = MatrixXuli::Constant(pointBins_eq_binNo_nnz, 1, 1);

		//for (unsigned long int i = 0; i < pointBins_eq_binNo_nnz; i++)
		//{
		//	binPtMask(i, 0) = this->pointBins(i, 0) == binNo ? 1 : 0;
		//}

		std::cout << "db1" << std::endl;

		for (unsigned long int i = 0; i < pointBins_eq_binNo_nnz; i++)
		{
			if (counter >= pointBins_eq_binNo_nnz)
			{
				break;
			}
			else
			{
				if (pointBins_eq_binNo(i, 0) != 0)
				{
					thisBinsPoints.block(counter, 0, 1, 3) = this->points.block(i, 0, 1, 3);
					counter++;
				}
			}
		}
		std::cout << "db2" << std::endl;

		oldMin = Eigen::MatrixXf::Zero(1, 3);
		oldMax = Eigen::MatrixXf::Zero(1, 3);
		std::cout << "db3" << std::endl;

		oldMin = this->binBoundaries.block(binNo - 1, 0, 1, 3);
		oldMax = this->binBoundaries.block(binNo - 1, 3, 1, 3);
		cout << "old min" << oldMin << endl;
		cout << "old max" << oldMax << endl;
		newDiv = Eigen::MatrixXf::Zero(1, 3);

		std::cout << "db4" << std::endl;
		if (this->style.compare("weighted") == 0)
		{
			newDiv = thisBinsPoints.colwise().mean();
			//cout << "new Div" << newDiv << endl;
		}
		else if (this->style.compare("normal") == 0)
		{
			newDiv = (oldMin + oldMax) / 2;
			cout << "new Div" << newDiv << endl;
		}
		std::cout << "db5" << std::endl;
		std::cout << "newdiv: " << newDiv << std::endl;

		minMidMax = Eigen::MatrixXf::Zero(1, 9);
		std::cout << "db6" << std::endl;
		minMidMax.block(0, 0, 1, 3) = oldMin;
		minMidMax.block(0, 3, 1, 3) = newDiv;
		minMidMax.block(0, 6, 1, 3) = oldMax;
		cout << "min mid max" << minMidMax << endl;
		std::cout << "db7" << std::endl;
		// This should be a constant, placed in a header file. These are indecies which are used to determine the verticies for the new bins.


		for (unsigned short int i = 1; i <= 8; i++)
		{
			for (unsigned short int j = 1; j <= 6; j++)
			{
				newBounds(i - 1, j - 1) = minMidMax(0, buildNewBoundaries[j + (i - 1) * 6 - 1] - 1);
			}
		}
		//std::cout << "new bounds" << newBounds<<endl;
		// This should be a constant, placed in a header file. These are vectors used to map thisBinsPoints to their respective bin index.

		// Determine to which of these 8 bins each current point belongs.
		for (unsigned long int i = 0; i < pointBins_eq_binNo_nnz; i++)
		{
			for (unsigned short int j = 0; j < 3; j++)
			{
				gtMask(i, j) = thisBinsPoints(i, j) > newDiv(0, j) ? 1 : 0;
			}
		}
		//cout << "gt mask" << gtMask << endl;
		std::cout << "db9" << std::endl;
		//for (unsigned short int k = 1; k <= 8; k++)
		//{
		//	for (unsigned long int i = 0; i < pointBins_eq_binNo_nnz; i++)
		//	{
		//		for (unsigned short int j = 0; j < 3; j++)
		//		{
		//			gtMask_binMap_eq(i + (k - 1)*pointBins_eq_binNo_nnz, j + (k - 1) * 3) = gtMask(i,j) == binMap(j,k-1) ? 1 : 0;
		//		}
		//		if (gtMask_binMap_eq.row(i + (k - 1)*pointBins_eq_binNo_nnz).nonZeros() > 2)
		//		{
		//			binAssignment(i,0) = k;
		//		}
		//	}
		//}
		for (unsigned short int k = 1; k <= 8; k++)
		{
			for (unsigned long int i = 0; i < pointBins_eq_binNo_nnz; i++) // check pointBins_eq_binNo_nnz, make sure it is decreasing
			{
				for (unsigned short int j = 0; j < 3; j++)
				{
					gtMask_binMap_eq(i, j) = gtMask(i, j) == binMap(j, k - 1) ? 1 : 0;
				}
				if (gtMask_binMap_eq.row(i).nonZeros() > 2)
				{
					binAssignment(i, 0) = k;
				}
			}
		}
		std::cout << "pointBins_eq_binNo_nnz: " << pointBins_eq_binNo_nnz << std::endl;
		std::cout << "db10" << std::endl;
		std::vector<int> newBinsInds = linspace<int>(this->binCount - 1, this->binCount + 8 - 1, 9);
		std::cout << "db10.1" << std::endl;
		this->binDepths.resize(this->binCount + 8);
		std::cout << "db10.2" << std::endl;
		this->binParents.resize(this->binCount + 8);
		std::cout << "db10.3" << std::endl;
		this->binBoundaries.conservativeResize(this->binCount + 8,6);
		for (unsigned short int i = 0; i < 8; i++)
		{
			this->binBoundaries.block(this->binCount + i - 1, 0, 1, 6) = newBounds.row(i);
			this->binDepths[this->binCount + i - 1] = this->binDepths[binNo - 1] + 1;
			this->binParents[this->binCount + i - 1] = binNo - 1;
		std::cout << "db10.4" << std::endl;
		}
		std::cout << "db11" << std::endl;
		for (int i = 0; i < binPtMask.rows(); i++)
		{
			if (binPtMask(i, 0) == 1)
			{
				this->pointBins(i, 0) = newBinsInds[binAssignment(i, 0)];
			}
		}
		std::cout << "db12" << std::endl;
		this->binCount = this->binCount + 8;
		
		//std::cin.ignore();
	}
	
};
// Driver code 
int main()
{
	// Create a matrix to house the necessary verticies to identify the size and position of the octree partition.
	Eigen::MatrixXf r;
	r = Eigen::MatrixXf::Zero(3, 2);

	// First parition is the entire map. 
	// Map dimensions: 20m(X) by 20m(Y) by 6m(Z) or 100 by 100 by 30 voxels with edges 0.2m.
	// If working in camera coordinates: 20m(Y,lateral) by 6m(Z,height) by 20m(X,depth) or 100 by 30 by 100 voxels with edges 0.2m.
	r(0, 1) = 20.0;
	r(1, 1) = 20.0;
	r(2, 1) = 5.6;
	r(0, 0) = 0.20;
	r(1, 0) = 3.20;
	r(2, 0) = 0;
	cout << "r matrix" << r << endl;

	// Open text file containing point cloud
	std::ifstream map_file_stream;
	if (!map_file_stream.is_open())
	{
		map_file_stream.open("map_10_9_2020_explored_points.txt");
	}

	// Here we will need the mapper to tell the partitioner how many points have been explored
	// This quantity is hard coded for now.
	int explored_points = 181080;
	std::vector<float> ex_x, ex_y, ex_z;
	float temp, temp1, temp2;

	// Read in point cloud from text file (will replace with pipeline to pointcloud)
	while (map_file_stream >> temp)
	{
		ex_x.push_back(temp);
		map_file_stream >> temp1;
		ex_y.push_back(temp1);
		map_file_stream >> temp2;
		ex_z.push_back(temp2);
	}
	for (int i = 0; i < ex_x.size(); i++) {
		std::cout << ex_x.at(i) << ' ';
	}
	// No need to partition in trivial cases.
	if (explored_points < 2)
	{
		std::cout << "Trivial case, something may be wrong with the map." << std::endl;
		exit(0);
	}

	// Initial OCTree contains all points
	Octree ocTree(explored_points/3, 2000, &r, "normal");
	//points = 60360 not 181080
	std::cin.ignore();

	return 0;
}
