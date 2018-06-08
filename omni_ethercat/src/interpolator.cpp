//
// Created by amaldo on 6/6/18.
//

#include "omni_ethercat/interpolator.hpp"
#include <iostream>


#define CYCLE_TIME_IN_SECONDS                   0.001
#define NUMBER_OF_DOFS                          3

ReflexxesInterpolator::ReflexxesInterpolator() : RML(NULL), IP(NULL), OP(NULL) {
    std::cout << "ReflexxesInterpolator(): starting up." << std::endl;
    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);

    IP->CurrentPositionVector->VecData[0] = 0.0;
    IP->CurrentPositionVector->VecData[1] = 0.0;
    IP->CurrentPositionVector->VecData[2] = 0.0;

    IP->CurrentVelocityVector->VecData[0] = 0.0;
    IP->CurrentVelocityVector->VecData[1] = 0.0;
    IP->CurrentVelocityVector->VecData[2] = 0.0;

    IP->CurrentAccelerationVector->VecData[0] = 0.0;
    IP->CurrentAccelerationVector->VecData[1] = 0.0;
    IP->CurrentAccelerationVector->VecData[2] = 0.0;

    IP->MaxAccelerationVector->VecData[0] = 2.0;
    IP->MaxAccelerationVector->VecData[1] = 2.0;
    IP->MaxAccelerationVector->VecData[2] = 2.0;

    IP->MaxJerkVector->VecData[0] = 10.0;
    IP->MaxJerkVector->VecData[1] = 10.0;
    IP->MaxJerkVector->VecData[2] = 10.0;

    IP->TargetVelocityVector->VecData[0] = 0.0;
    IP->TargetVelocityVector->VecData[1] = 0.0;
    IP->TargetVelocityVector->VecData[2] = 0.0;

    IP->SelectionVector->VecData[0] = true;
    IP->SelectionVector->VecData[1] = true;
    IP->SelectionVector->VecData[2] = true;

}

ReflexxesInterpolator::~ReflexxesInterpolator() {
    std::cout << "~ReflexxesInterpolator(): closing shop." << std::endl;
    delete RML;
    delete IP;
    delete OP;
}

void ReflexxesInterpolator::set_current_pose(double x, double y, double theta) {
    IP->CurrentPositionVector->VecData[0] = x;
    IP->CurrentPositionVector->VecData[1] = y;
    IP->CurrentPositionVector->VecData[2] = theta;
}

void ReflexxesInterpolator::set_current_twist(double dx, double dy, double dtheta) {
    IP->CurrentVelocityVector->VecData[0] = dx;
    IP->CurrentVelocityVector->VecData[1] = dy;
    IP->CurrentVelocityVector->VecData[2] = dtheta;
}

void ReflexxesInterpolator::set_target_twist(double dx, double dy, double dtheta) {
    IP->TargetVelocityVector->VecData[0] = dx;
    IP->TargetVelocityVector->VecData[1] = dy;
    IP->TargetVelocityVector->VecData[2] = dtheta;

}

void ReflexxesInterpolator::get_next_twist(double &dx, double &dy, double &dtheta) {
    result = RML->RMLVelocity(*IP, OP, Flags);

    if (result < 0) {
        printf("An error occurred (%d).\n", result);
        //break;
    }

    // ****************************************************************
    // Here, the new state of motion, that is
    //
    // - OP->NewPositionVector
    // - OP->NewVelocityVector
    // - OP->NewAccelerationVector
    //
    // can be used as input values for lower level controllers. In the
    // most simple case, a position controller in actuator space is
    // used, but the computed state can be applied to many other
    // controllers (e.g., Cartesian impedance controllers,
    // operational space controllers).
    // ****************************************************************



    // ****************************************************************
    // Feed the output values of the current control cycle back to
    // input values of the next control cycle

    *IP->CurrentPositionVector = *OP->NewPositionVector;
    *IP->CurrentVelocityVector = *OP->NewVelocityVector;
    *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;

    dx = OP->NewVelocityVector->VecData[0];
    dy = OP->NewVelocityVector->VecData[1];
    dtheta = OP->NewVelocityVector->VecData[2];

    //std::cout << "new twist = [" << dx << "," << dy << "," << dtheta << "]" << std::endl;
}

bool ReflexxesInterpolator::setup() {
    return true;
}
