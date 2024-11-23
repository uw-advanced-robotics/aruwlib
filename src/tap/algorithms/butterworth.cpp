#include <complex>
#include <vector>
#include <cmath>
#include <numeric>
#include "modm/math/geometry/angle.hpp"
#include "butterworth.hpp"

namespace tap
{
namespace algorithms
{
    Butterworth::Butterworth(float wc, int n, float Ts) : 
    naturalResponseCoefficients(n, 0.0f),
    forcedResponseCoefficients(n, 0.0f),
    naturalResponse(n, 0.0f),
    naturalResponseResult(n, 0.0f),
    forcedResponse(n, 0.0f),
    forcedResponseResult(n, 0.0f),
    sum(n, 0.0f)
    {
        //Warp frequency for bilinear transform
        wc = (2/Ts)*std::atan(wc*(Ts/2));

        //generate poles for butterworth filter
        std::vector<std::complex<float>> poles;
        for (int k = 0; k < n; ++k) {
            float theta = M_PI * (2.0 * k + 1) / (2.0 * n) + M_PI/2;
            std::complex<float> pole = std::polar(wc, theta);
            poles.push_back(pole);
        }

        //Preform the z transform for each pole
        std::vector<std::complex<float>> zPoles(n);
        for (int i = 0; i < n; ++i) {
            zPoles[i] = s2z(poles[i], Ts);
        }

        // zeros calculation, for a butterworth all zeros in the Z domain are -1
        std::complex<float> zZero(-1,0);
        std::vector<std::complex<float>> zZeros(n, zZero);

        // Calculate polynomial coefficients
        auto forcedResponseCoefficients = expandPolynomial(zZeros);
        auto naturalResponseCoefficients = expandPolynomial(zPoles);

        // Calculate and apply the scalar so the DC gain at 1hz is 1
        auto scale_factor = calculateScalar(forcedResponseCoefficients, naturalResponseCoefficients);

        std::transform(forcedResponseCoefficients.begin(), forcedResponseCoefficients.end(), 
        forcedResponseCoefficients.begin(), [&scale_factor](auto& c){return c*scale_factor;});



    }
    
    float Butterworth::filterData(float dat){
        
        addData(forcedResponse, dat);
        std::transform(forcedResponse.begin()+1, forcedResponse.end(),
                forcedResponseCoefficients.begin()+1, forcedResponseResult.begin(),
                std::multiplies<float>() );

        std::transform( naturalResponse.begin()+1, naturalResponse.end(),
                naturalResponseCoefficients.begin()+1, naturalResponseResult.begin(),
                std::multiplies<float>() );

        std::transform(naturalResponseResult.begin(), naturalResponseResult.end(), forcedResponseResult.begin(),
               sum.begin(), std::plus<double>());
    
        addData(naturalResponse, std::reduce(sum.begin(),sum.end()));

        return naturalResponse[0];
    }

    float Butterworth::getLastFiltered(){
        return naturalResponse[0];
    }

    void Butterworth::addData(std::vector<float> &Response, float newData){
        Response.insert(Response.begin(), newData);
        Response.pop_back();
        return;
    }

    std::complex<float> s2z(std::complex<float> s, float Ts){
        return (1.0f + (Ts / 2) * s) / (1.0f - (Ts / 2) * s);
    }

    std::vector<float> Butterworth::expandPolynomial(std::vector<std::complex<float>> zeros){

        std::vector<std::complex<float>> coefficients(1, std::complex<float>(1.0, 0.0));  // Start with 1
        for (const auto& zero : zeros) {
            // Multiply current polynomial by (x - zero)
            std::vector<std::complex<float>> newCoefficients(coefficients.size() + 1, std::complex<float>(0.0, 0.0));
            for (size_t i = 0; i < coefficients.size(); ++i) {
                newCoefficients[i] -= coefficients[i] * zero;      // Multiply current coefficient by zero
                newCoefficients[i + 1] += coefficients[i];         // Shift current coefficient
            }
            coefficients = newCoefficients;  // Update coefficients
        }
        std::vector<float> stripped_coefficients(coefficients.size(), 0.0);
        // Extract the real part of each coefficient, the imaginary appears to be an artifact of float
        for (size_t i = 0; i < coefficients.size(); ++i) {
            stripped_coefficients[i] = coefficients[i].real();
        }

        return stripped_coefficients;
    }

    float Butterworth::calculateScalar(std::vector<float> numeratior, std::vector<float> denominator){
        auto denResult = std::reduce(denominator.begin(), denominator.end());
        auto numResult = std::reduce(numeratior.begin(), numeratior.end());
        //den is over num in the result as it is in the form 1/(num/den) to apply a scalar for the DC gain
        return denResult/numResult;
    }


}// namespace algorithms

}// namespace tap