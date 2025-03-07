#ifndef NORMALDISTRIBUTION_H
#define NORMALDISTRIBUTION_H

#include <iostream>
#include <vector>
#include <math.h>
#include <time.h>
#include <unistd.h>

class NormalDistribution
{
private:
	unsigned int numMeasures;
	std::vector<int> 	theHistogram;
	std::vector<double> distribution;
	double standardDeviation;
	std::vector<double> samples;
	unsigned int numSamples = 0, depthEstimated = 0;
	unsigned short int percentageOfAccuracy;
	float average = 0;
	double sumatory = 0;
	bool estimated = false, orientation = false;

	void initParameters();
    void calculateStandardDeviation();
	void calculateNormalDistribution();
	double transformData(const std::string);
public:	
	NormalDistribution() : percentageOfAccuracy(10) {}
	NormalDistribution(int);
	~NormalDistribution(){}

	void setAccuracy(const unsigned short int acc) { this->percentageOfAccuracy = acc; }
	void setNumMeasures(const unsigned int mess);
	bool getIsEstimated() const { return this->estimated; }
	float getAverage() const { return this->average; }
	unsigned int getDepthEstimation() const { return this->depthEstimated; }
	unsigned int getStandardDeviation() const { return trunc(this->standardDeviation * 100); }
	double standardDeviationNotTrunc() const { return this->standardDeviation; }

	double chooseTheRightDepth(const double, const double, const bool);
	void runNormalDistribution(const double, const double, const bool);		//, double);
	void runNormalDistributionForOrientation(std::string &);
	void depthWasEstimated();
};

#endif