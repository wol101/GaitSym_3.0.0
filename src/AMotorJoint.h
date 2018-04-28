/*
 *  AMotorJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 07/01/2011.
 *  Copyright 2011 Bill Sellers. All rights reserved.
 *
 */

#ifndef AMOTORJOINT_H
#define AMOTORJOINT_H

#include "Joint.h"

class AMotorJoint: public Joint
{
public:

    AMotorJoint(dWorldID worldID);

    void SetStops(dReal low, dReal high);
    void SetAxis(dReal x, dReal y, dReal z, int axisMode);
    void SetAxis(const char *buf);
    void SetTargetVelocity(dReal targetVelocity);
    void SetMaxTorque(dReal maximumTorque);

    dReal GetAngle();
    dReal GetAngleRate();

    virtual void Update();
    virtual void Dump();

#ifdef USE_OPENGL
    virtual void Draw();
#endif

protected:

    void SetAngle();
};


#endif // AMOTORJOINT_H
