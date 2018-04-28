/*
 *  PositionReporter.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/12/2010.
 *  Copyright 2010 Bill Sellers. All rights reserved.
 *
 */

#ifndef POSITIONREPORTER_H
#define POSITIONREPORTER_H

#include "Reporter.h"
#include "PGDMath.h"

class Body;

class PositionReporter : public Reporter
{
public:

    PositionReporter();

    void SetBody(Body *body) { mBody = body; }
    Body *GetBody() { return mBody; }

    // these functions set the geom position relative to its body
    void SetPosition (dReal x, dReal y, dReal z)
    {
        mPosition.x = x; mPosition.y = y; mPosition.z = z;
    }
    void SetQuaternion(dReal q0, dReal q1, dReal q2, dReal q3)
    {
        mQuaternion.n = q0;
        mQuaternion.v.x = q1; mQuaternion.v.y = q2; mQuaternion.v.z = q3;
    }
    void SetPosition (const char *buf);
    void SetQuaternion(const char *buf);

    pgd::Vector GetPosition() { return mPosition; }
    pgd::Quaternion GetQuaternion() { return mQuaternion; }
    pgd::Vector GetWorldPosition();
    pgd::Quaternion GetWorldQuaternion();
    pgd::Vector GetWorldVelocity();

    virtual void Dump();

#ifdef USE_OPENGL
    virtual void Draw();
    void SetRadius(dReal v) { mRadius = v; }
#endif

protected:

    Body *mBody;
    pgd::Vector mPosition;
    pgd::Quaternion mQuaternion;

#ifdef USE_OPENGL
    dReal mRadius;
#endif

};

#endif // POSITIONREPORTER_H
