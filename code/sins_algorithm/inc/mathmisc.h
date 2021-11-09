/**
 ******************************************************************************
 * @addtogroup OpenPilot Math Utilities
 * @{
 * @addtogroup Reuseable math functions
 * @{
 *
 * @file       mathmisc.h
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      Reuseable math functions
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef MATHMISC_H
#define MATHMISC_H

#include <math.h>
#include <cmath>
#include <cstring>
#include <stdint.h>

struct pointf_t
{
    float x;
    float y;
};
struct posef_t
{
    float x;
    float y;
    float th;
};
typedef struct pointf_t pointf;
typedef struct posef_t posef;

/**
 * Ultrafast pow() aproximation needed for expo
 * Based on Algorithm by Martin Ankerl
 */
static inline float signf(float x)
{
    return x > 0.0 ? +1.0 : x < 0.0 ? -1.0 : 0.0;
}
// returns min(boundary1,boundary2) if val<min(boundary1,boundary2)
// returns max(boundary1,boundary2) if val>max(boundary1,boundary2)
// returns val if min(boundary1,boundary2)<=val<=max(boundary1,boundary2)
static inline float boundf(float val, float boundary1, float boundary2)
{
    if (val < boundary1)
    {
        return boundary1;
    }
    else if (val > boundary2)
    {
        return boundary2;
    }
    return val;
}

// returns min(boundary1,boundary2) if val<min(boundary1,boundary2)
// returns max(boundary1,boundary2) if val>max(boundary1,boundary2)
// returns val if min(boundary1,boundary2)<=val<=max(boundary1,boundary2)
static inline float boundf2(float val, float boundary1, float boundary2)
{
    if (fabs(val) < boundary1)
    {
        return signf(val) * boundary1;
    }
    else if (fabs(val) > boundary2)
    {
        return signf(val) * boundary2;
    }
    return val;
}

// returns min(boundary1,boundary2) if val<min(boundary1,boundary2)
// returns max(boundary1,boundary2) if val>max(boundary1,boundary2)
// returns val if min(boundary1,boundary2)<=val<=max(boundary1,boundary2)
static inline int32_t boundi(int32_t val, int32_t boundary1, int32_t boundary2)
{
    if (val < boundary1)
    {
        return boundary1;
    }
    else if (val > boundary2)
    {
        return boundary2;
    }
    return val;
}

// returns min(boundary1,boundary2) if val<min(boundary1,boundary2)
// returns max(boundary1,boundary2) if val>max(boundary1,boundary2)
// returns val if min(boundary1,boundary2)<=val<=max(boundary1,boundary2)
static inline int16_t boundi16(int16_t val, int16_t boundary1, int16_t boundary2)
{
    if (val < boundary1)
    {
        return boundary1;
    }
    else if (val > boundary2)
    {
        return boundary2;
    }
    return val;
}

static inline float squaref(float x)
{
    return x * x;
}

static inline float vector_lengthf(float *vector, const uint8_t dim)
{
    float length = 0.0f;

    for (int t = 0; t < dim; t++)
    {
        length += squaref(vector[t]);
    }
    return sqrtf(length);
}
/*
static inline void vector_normalizef(float *vector, const uint8_t dim)
{
    float length = vector_lengthf(vector, dim);

    if (length <= 0.0f || isnan(length))
    {
        return;
    }
    for (int t = 0; t < dim; t++)
    {
        vector[t] /= length;
    }
}
*/
// Returns the distance between points p0 and p1
static inline float distance(pointf *p0, pointf* p1)
{
    return sqrtf(squaref(p1->x - p0->x) + squaref(p1->y - p0->y));
}

// Returns the distance between points p0 and p1
static inline float distance_pose(posef *p0, posef* p1)
{
    return sqrtf(squaref(p1->x - p0->x) + squaref(p1->y - p0->y));
}
// Returns the y value, given x, on the line passing through the points p0 and p1.
static inline float y_on_line(float x, const pointf *p0, const pointf *p1)
{
    // Setup line y = m * x + b.
    const float dY1 = p1->y - p0->y;
    const float dX1 = p1->x - p0->x;
    const float m   = dY1 / dX1; // == dY0 / dX0 == (p0.y - b) / (p0.x - 0.0f) ==>
    const float b   = p0->y - m * p0->x;

    // Get the y value on the line.
    return m * x + b;
}

// Returns the y value, given x, on the curve defined by the points array.
// The fist and last line of the curve extends beyond the first resp. last points.
static inline float y_on_curve(float x, const pointf points[], int num_points)
{
    // Find the two points x is within.
    // If x is smaller than the first point's x value, use the first line of the curve.
    // If x is larger than the last point's x value, user the last line of the curve.
    int end_point = num_points - 1;

    for (int i = 1; i < num_points; i++)
    {
        if (x < points[i].x)
        {
            end_point = i;
            break;
        }
    }

    // Find the y value on the selected line.
    return y_on_line(x, &points[end_point - 1], &points[end_point]);
}

static inline float invsqrtf(float number)
{
    float y;

    y = 1.0f / sqrtf(number);
    return y;
}

/**
 * Ultrafast pow() aproximation needed for expo
 * Based on Algorithm by Martin Ankerl
 */
static inline float fastPow(float a, float b)
{
    union
    {
        double  d;
        int32_t x[2];
    } u = { (double)a };
    u.x[1] = (int32_t)(b * (u.x[1] - 1072632447) + 1072632447);
    u.x[0] = 0;
    return (float)u.d;
}

/// This subtracts one angle from another and fixes the result to [-180,180]
/**
   @param value1 first angle
   @param value2 second angle, subtracted from first angle
   @return
   @see
   @see
*/
static inline void swapf(float *value1, float *value2)
{
    float t = *value1;
    *value1 = *value2;
    *value2 = t;
}
/// This subtracts one angle from another and fixes the result to [-180,180]
/**
   @param value1 first angle
   @param value2 second angle, subtracted from first angle
   @return
   @see
   @see
*/
static inline void swapi8(int8_t *value1, int8_t *value2)
{
    int8_t t = *value1;
    *value1 = *value2;
    *value2 = t;
}

/// This subtracts one angle from another and fixes the result to [-180,180]
/**
   @param value1 first angle
   @param value2 second angle, subtracted from first angle
   @return
   @see
   @see
*/
static inline void swapu8(uint8_t *value1, uint8_t *value2)
{
    uint8_t t = *value1;
    *value1 = *value2;
    *value2 = t;
}

#include <math.h>
/*!
 * \brief Convert degrees to radians
 */

static inline float from_degrees(float degrees)
{
    return degrees * M_PI / 180.0f;
}

/*!
 * \brief Convert radians to degrees
 */
static inline float to_degrees(float radians)
{
    return radians * 180.0f / M_PI;
}

/*!
 * \brief normalize_angle_positive
 *
 *        Normalizes the angle to be 0 to 2*M_PI
 *        It takes and returns radians.
 */
static inline float normalize_angle_positive(float angle)
{
    return fmod(fmod(angle, 2.0f * M_PI) + 2.0f * M_PI, 2.0f * M_PI);
}

/*!
 * \brief normalize
 *
 * Normalizes the angle to be -M_PI circle to +M_PI circle
 * It takes and returns radians.
 *
 */
static inline float fixAngle(float angle)
{
   // if (angle >= 2 * M_PI)
        angle = angle - 2 * M_PI * ((int)(angle / (2 * M_PI)));
   // if (angle < -2 * M_PI)
      //  angle = angle + 2 * M_PI * ((int)(angle / (2 * M_PI)));
    if (angle <= -M_PI)
        angle = M_PI + (angle + M_PI);
    if (angle > M_PI)
        angle = - M_PI + (angle - M_PI);
    return angle;
}

/// This adds two angles together and fixes the result to [-180, 180]
/**
   @param ang1 first angle
   @param ang2 second angle, added to first
   @return sum of the angles, in range [-180,180]
   @see subAngle
   @see fixAngle */
static inline float addAngle(float ang1, float ang2)
{
    return fixAngle(ang1 + ang2);
}

/// This subtracts one angle from another and fixes the result to [-180,180]
/**
   @param ang1 first angle
   @param ang2 second angle, subtracted from first angle
   @return resulting angle, in range [-180,180]
   @see addAngle
   @see fixAngle
*/
static inline float subAngle(float ang1, float ang2)
{
    return fixAngle(ang1 - ang2);
}

static inline float shortestAngularDistance(float from, float to)
{
    return fixAngle(to - from);
}

/*!
 * \brief normalize
 *
 * Normalizes the angle to be -M_PI circle to +M_PI circle
 * It takes and returns radians.
 *
 */
static inline float normalize_angle(float angle)
{
    float a = normalize_angle_positive(angle);
    if (a > M_PI)
        a -= 2.0f * M_PI;
    return a;
}

/*!
 * \function
 * \brief shortest_angular_distance
 *
 * Given 2 angles, this returns the shortest angular
 * difference.The inputs and ouputs are of course radians.
 *
 * The result
 * would always be -pi <= result <= pi.Adding the result
 * to "from" will always get you an equivelent angle to "to".
 */

static inline float shortest_angular_distance(float from, float to)
{
    return normalize_angle(to - from);
}

/*!
 * \function
 *
 * \brief returns the angle in [-2*M_PI, 2*M_PI]going the other way along the unit circle.
 * \param angle The angle to which you want to turn in the range [-2*M_PI, 2*M_PI]
 * E.g. two_pi_complement(-M_PI/4) returns 7_M_PI/4
 * two_pi_complement(M_PI/4) returns -7*M_PI/4
 *
 */
static inline float two_pi_complement(float angle)
{
    //check input conditions
    if (angle > 2 * M_PI || angle < -2.0f * M_PI)
        angle = fmod(angle, 2.0f * M_PI);
    if (angle < 0)
        return (2 * M_PI + angle);
    else if (angle > 0)
        return (-2 * M_PI + angle);

    return (2 * M_PI);
}

#endif /* MATHMISC_H */
