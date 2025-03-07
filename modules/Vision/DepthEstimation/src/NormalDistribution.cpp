#include <DepthEstimation/NormalDistribution.hpp>

#define MINUM_SAMPLES 101

NormalDistribution::NormalDistribution(int maxDistance) : numMeasures(maxDistance), percentageOfAccuracy(10/*% accuracy*/)
{
    this->initParameters();
}

void NormalDistribution::initParameters()
{
    this->standardDeviation = this->numSamples = this->average = this->sumatory = 0;

    if ( !this->distribution.empty() ) 
    {
        this->theHistogram.clear();
        this->distribution.clear(); 
        this->samples.clear();
    }

    this->theHistogram = std::vector<int>(this->numMeasures);
    this->distribution = std::vector<double>(this->numMeasures);
}

void NormalDistribution::setNumMeasures(const unsigned int mess)
{
    this->estimated = false;
    this->numMeasures = mess;
    this->initParameters();
}

double NormalDistribution::chooseTheRightDepth(const double distanceInX, const double distanceInY, const bool rotations)
{
    double totalDepth;

    if ( (distanceInX > 0.1 and distanceInX < 7) and (distanceInY > 0.1 and distanceInY < 7) )
    {
        if ( !rotations ) totalDepth = distanceInY;     //it is calculated by distanceInX
        else
        {
            auto absDepth = fabs(distanceInX - distanceInY);
            if ( (absDepth < (distanceInX * 0.2)) and (absDepth < (distanceInY * 0.2)) ) totalDepth = (distanceInX + distanceInY)/2;
            else totalDepth = distanceInX > distanceInY ? distanceInX : distanceInY;
        }
    }else if ( distanceInX > 0.1 and distanceInX < 7 ) totalDepth = distanceInX;
        else totalDepth = distanceInY;

    return totalDepth;
}

void NormalDistribution::runNormalDistribution(const double distanceInX, const double distanceInY, const bool rotations)  //, double totalDistance)
{
    auto totalDistance = this->chooseTheRightDepth(distanceInX, distanceInY, rotations);

    if ( totalDistance > 0.1 and totalDistance < 7 )
    {
        if ( this->samples.size() < (!this->orientation ? MINUM_SAMPLES : MINUM_SAMPLES/2) )
        {
            this->samples.push_back(totalDistance);
            this->theHistogram[ (totalDistance * 100)-11 ]++;
            if ( ++this->numSamples == (!this->orientation ? MINUM_SAMPLES : MINUM_SAMPLES/2) )
            {
                for (int i = 0; i < this->numSamples; i++) this->average += this->samples[i];
                this->average /= this->numSamples;
                this->calculateStandardDeviation();
            }
        }
        else
        {
            if ( fabs((totalDistance - this->average) / this->standardDeviation) <= 2.5)  //filtered outliers values through calculated the standard deviation of the new "SAMPLE" with respect to our set
            {
                this->samples.push_back(totalDistance);
                this->theHistogram[ (totalDistance * 100)-11 ]++;

                this->average = ( (this->average*this->numSamples) + totalDistance ) / ++this->numSamples;
                
                this->calculateStandardDeviation();   
                this->calculateNormalDistribution();
            }else std::cout << "Out of standard desviation (" << this->standardDeviation << ")" << std::endl;         
        }
    }
}

////////////////////////////////// FUNCTIONS TO ORIENTATION ESTIMATION ////////////////////////////////////////
double NormalDistribution::transformData( const std::string turnTo )
{
    if ( turnTo == "front" ) return  0;
    if ( turnTo == "right" ) return  1;
    if ( turnTo == "left"  ) return -1;
}

void NormalDistribution::runNormalDistributionForOrientation(std::string &turnTo)
{
    auto orientation = this->transformData(turnTo);
    if ( this->samples.size() < MINUM_SAMPLES/2)
        {
            this->samples.push_back( orientation );
            if ( ++this->numSamples == MINUM_SAMPLES/2)
            {
                for (int i = 0; i < this->numSamples; i++) this->average += this->samples[i];
                this->average /= this->numSamples;
                this->calculateStandardDeviation();
            }
    }
    else
    {
        this->samples.push_back( orientation );

        this->average = ( (this->average*this->numSamples) + orientation ) / ++this->numSamples;
                
        this->calculateStandardDeviation();   
        this->calculateNormalDistribution();
    }
}
////////////////////////////////// FUNCTIONS TO ORIENTATION ESTIMATION ////////////////////////////////////////

void NormalDistribution::calculateNormalDistribution()
{  
    float Q = 1 / (this->standardDeviation * sqrt(2*M_PI));
    std::cout << "N(" << this->average << "," << this->standardDeviation << ")" << std::endl;

    for (int i = 0; i < this->numMeasures; i++) this->distribution[i] = Q * exp(-0.5 * (pow( (((i+1) - this->average)/this->standardDeviation),2) )); 
    this->depthWasEstimated();
}

void NormalDistribution::calculateStandardDeviation()
{
    if ( this-> numSamples == (!this->orientation ? MINUM_SAMPLES : MINUM_SAMPLES/2) )
        for (int i = 0; i < this->numSamples; i++) this->sumatory += pow( (this->samples[i] - this->average), 2);
    else this->sumatory += pow( (this->samples[this->samples.size() -1] - this->average), 2);

    this->standardDeviation = sqrt( this->sumatory/(this->numSamples-1) );

    //std::cout << "THE HISTOGRAM!!!" << std::endl;
    //for (int i = 0; i < this->numMeasures; i++) std::cout << this->theHistogram[i] << " dd ";
}

void NormalDistribution::depthWasEstimated()
{
    if ( this->standardDeviation <= fabs( this->average*((float)this->percentageOfAccuracy/100) ) )
    {
        this->estimated = true;
        this->depthEstimated    = trunc(this->average * 100);
        //this->percentageOfError = trunc(this->standardDeviation * 100);
    } else if ( this->samples.size() > (!this->orientation ? MINUM_SAMPLES : MINUM_SAMPLES/2) * 3 ) this->initParameters();
}