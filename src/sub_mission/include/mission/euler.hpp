/** @file euler.hpp
 *  @brief Euler angle conversions. 
 *
 *  @author David Zhang
 */
#ifndef MISSION_EULER_HPP
#define MISSION_EULER_HPP

/** @brief Converts values between body and inertial frame of reference.
 *
 *  @param input Pointer to where the values of the body frame are.
 *  @param angles Current euler angles.
 *  @param output Pointer to where the values of the inertial frame will be
 *         stored.
 */
void bodyToInertial(float *input, float *angles, float *output);

#endif
