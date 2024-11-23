#ifndef TAPROOT_BUTTERWORTH_HPP_
#define TAPROOT_BUTTERWORTH_HPP_

#include <complex>
#include <vector>

namespace tap
{
namespace algorithms{

class Butterworth{

public:
    /**
     * Initializes a butterworth filter with a given order, cuttoff frequency, and sample time
     *
     * @note make sure to understand what the a given order affects before implementing,
     * mainly the potential phase delays
     * 
     * @param[in] wc the cutoff frequency, roughly the frequency where attenuation begins.
     * @param[in] n the order of the Butterworth, higher orders mean a sharper droppoff in frequency.
     * @param[in] Ts the sample time in seconds, for example at 500hz the value should be 0.02
     */
    Butterworth(float wc, int n, float Ts);

    /**
     * runs the butterworth filter, should be supplied at the provided sample time 
     * @param[in] dat the value to be filtered.
     * @return The current prediction of what the data should be.
    */

    float filterData(float dat);

    /**
     * @return Returns the last filtered data point.
     */
    float getLastFiltered();

private:

    std::vector<float> naturalResponseCoefficients;

    std::vector<float> forcedResponseCoefficients;

    std::vector<float> naturalResponse;

    std::vector<float> naturalResponseResult;
    
    std::vector<float> forcedResponse;

    std::vector<float> forcedResponseResult;

    std::vector<float> sum;
    /**
     * used to transform poles from the laplauce domain to the 
     * Z domain for descrete time using the bilinear transform
     * 
     * @param [in] s a pole or zero from the laplauce domain
     * @param [in] Ts the sample time
     * @return a complex number corresponding to the Z domain location
     */
    std::complex<float> s2z(std::complex<float> s, float Ts); 

    /**
     * used to multiply out a series of zeros to obtain a list of coefficents
     * 
     * @param [in] zeros a vector of complex poles or zeros to multiply out, works for
     * any polynomial
     */
    std::vector<float> expandPolynomial(std::vector<std::complex<float>> zeros);

    void addData(std::vector<float> &Response, float newData);

    float calculateScalar(std::vector<float> numerator, std::vector<float> denominator);
};


} //namespace algorithms

} //namespace tap

#endif  // TAPROOT_BUTTERWORTH_HPP_