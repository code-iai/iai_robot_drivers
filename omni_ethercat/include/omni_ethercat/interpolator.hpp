

#ifndef OMNI_ETHERCAT_INTERPOLATOR_HPP
#define OMNI_ETHERCAT_INTERPOLATOR_HPP


//Define a class that uses Reflexxes to interpolate in Twist-Space
#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>

#define CYCLE_TIME_IN_SECONDS                   0.001
#define NUMBER_OF_DOFS                          3

class ReflexxesInterpolator {
    int result;
    ReflexxesAPI *RML;
    RMLVelocityInputParameters  *IP;
    RMLVelocityOutputParameters *OP;
    RMLVelocityFlags Flags;

public:
    ReflexxesInterpolator();
    ~ReflexxesInterpolator();

};



#endif //OMNI_ETHERCAT_INTERPOLATOR_HPP
