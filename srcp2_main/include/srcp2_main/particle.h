#ifndef SRCP2_MAIN_PARTICLE_H_
#define SRCP2_MAIN_PARTICLE_H_

// STL
#include <vector>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * @brief      This class describes a particle for Monte Carlo Locilization
 */
typedef struct Particle
{
    typedef Eigen::Affine3d Pose;
    typedef double Weight;

    Pose pose_;
    Weight weight_;
} Particle;

#endif  // SRCP2_MAIN_PARTICLE_H_
