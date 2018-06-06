

#ifndef OMNI_ETHERCAT_INTERPOLATOR_HPP
#define OMNI_ETHERCAT_INTERPOLATOR_HPP


//Define a class that uses Reflexxes to interpolate in Twist-Space
#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>


class ReflexxesInterpolator {
    int result;
    ReflexxesAPI *RML;
    RMLVelocityInputParameters  *IP;
    RMLVelocityOutputParameters *OP;
    RMLVelocityFlags Flags;

public:
    ReflexxesInterpolator();
    ~ReflexxesInterpolator();

    bool setup();
    void set_current_pose(double x, double y, double theta);
    void set_current_twist(double dx, double dy, double dtheta);
    void set_target_twist(double dx, double dy, double dtheta);
    void get_next_twist(double &dx, double &dy, double &dtheta);

};



#endif //OMNI_ETHERCAT_INTERPOLATOR_HPP
