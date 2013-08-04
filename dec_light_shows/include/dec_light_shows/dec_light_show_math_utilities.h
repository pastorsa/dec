/*
 * dec_light_show_math_utilities.h
 *
 *  Created on: Jul 30, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_MATH_UTILITIES_H_
#define DEC_LIGHT_SHOW_MATH_UTILITIES_H_

#include <dec_utilities/assert.h>
#include <conversions/tf_to_ros.h>
#include <conversions/ros_to_tf.h>

namespace dec_light_shows
{

class MathUtilities
{

public:
  enum Profile
  {
    eLINEAR = 0,
    eEXP_SIGMA_0,
    eEXP_SIGMA_1,
    eEXP_SIGMA_2,
    eEXP_SIGMA_3,
    eNUM_PROFILES
  };

  static float ramp(const float min,
               const float max,
               const float value,
               const Profile profile = eLINEAR)
  {
    ROS_ASSERT(!(value < min));
    ROS_ASSERT(!(value > max));
    ROS_ASSERT(max > min);
    ROS_ASSERT(max > 1e-3);
    float result = 0.0;
    switch (profile)
    {
      case eLINEAR:
      {
        result = min + (max - min) * ((max - value) / max);
        break;
      }
      case eEXP_SIGMA_0:
      {
        const float SIGMA_SQUARED = 0.04f;
        result = min + (max - min) * exp(-pow(value, 2.0) / SIGMA_SQUARED);
        break;
      }
      case eEXP_SIGMA_1:
      {
        const float SIGMA_SQUARED = 0.08f;
        result = min + (max - min) * exp(-pow(value, 2.0) / SIGMA_SQUARED);
        break;
      }
      case eEXP_SIGMA_2:
      {
        const float SIGMA_SQUARED = 0.14f;
        result = min + (max - min) * exp(-pow(value, 2.0) / SIGMA_SQUARED);
        break;
      }
      case eEXP_SIGMA_3:
      {
        const float SIGMA_SQUARED = 0.22f;
        result = min + (max - min) * exp(-pow(value, 2.0) / SIGMA_SQUARED);
        break;
      }
      default:
      {
        ROS_ERROR("Profile >%i< unknown.", (int)profile);
        return 0.0;
      }
    }
    return result;
  }

  static float computeDistance(const tf::Vector3& point,
                        const tf::Vector3& plane_vector,
                        const tf::Vector3& plane_normal)
  {
    return fabs(-plane_normal.dot(point - plane_vector));
  }


private:
  MathUtilities();
  virtual ~MathUtilities();

};

}

#endif /* DEC_LIGHT_SHOW_MATH_UTILITIES_H_ */
