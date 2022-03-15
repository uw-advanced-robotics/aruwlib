/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Copyright (c) 2019 Sanger_X
 */

#ifndef TAPROOT_EXTENDED_KALMAN_HPP_
#define TAPROOT_EXTENDED_KALMAN_HPP_

namespace tap
{
namespace algorithms
{
/**
 * The kalman filter can be used by instantiating an ExtendedKalman
 * object and calling filterData.
 *
 * Example source:
 *
 * \code
 * float sensorData;
 * float filtered;
 * ExtendedKalman kalman(1.0f, 0.0f);
 *
 * while(1)
 * {
 *     filtered = kalman.filterData(sensorData);
 * }
 * \endcode
 */
class ExtendedKalman
{
public:
    /**
     * Initializes a kalman filter with the given covariances.
     *
     * @note R is fixed. Larger Q means more trust in the
     *      data we are measuring. Conversely, a smaller Q means more
     *      trust in the model's prediction (rather than the measured) value.
     * @param[in] p the kalaman struct.
     * @param[in] tQ the system noise covariance.
     * @param[in] tR the measurement noise covariance.
     */
    ExtendedKalman(float tQ, float tR);

    /**
     * Runs the kalman filter, returning the current prediction.
     *
     * @note description of data:<br>
     * \f$x(k | k)\f$ is the current prediction (filtered output)<br>
     * (and then \f$k - 1\f$ would be the previous output)<br>
     * Corresponding formula:<br>
     * \f{eqnarray*}{
     * x(k | k-1) & = & A \cdot X(k-1 | k-1) + B \cdot U(k) + W(K)\\
     * p(k | k-1) & = & A \cdot p(k-1 | k-1) \cdot A^\prime + Q\\
     * kg(k) & = & p(k | k-1) \cdot \frac{H^\prime}{H \cdot p(k | k-1) \cdot H^\prime + R}\\
     * x(k | k) & = & X(k | k - 1) + kg(k) \cdot (Z(k) - H \cdot X(k | k-1))\\
     * p(k | k) & = & (I - kg(k) \cdot H) \cdot P(k | k-1)
     * \f}
     *
     * @param[in] dat the value to be filtered.
     * @return The current prediction of what the data should be.
     */
    float filterData(float dat);

    /**
     * Returns the last filtered data point.
     */
    float getLastFiltered() const;

    /**
     * Resets the covariances and predictions.
     */
    void reset();

private:
    float xLast;  /// last optimal prediction.
    float xMid;   /// forcast optimal prediction.
    float xNow;   /// current optimal prediction.
    float pMid;   /// predicted covariance.
    float pNow;   /// current covariance.
    float pLast;  /// previous covariance.
    float kg;     /// kalman gain.
    float A;      /// system parameter.
    float B;      /// system parameter.
    float Q;      /// system parameter
    float R;      /// system parameter.
    float H;      /// system parameter.
};                // class ExtendedKalman

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_EXTENDED_KALMAN_HPP_
