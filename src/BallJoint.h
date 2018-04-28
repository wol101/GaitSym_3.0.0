/*
 *  BallJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/12/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifndef BallJoint_h
#define BallJoint_h

#include "Joint.h"

class BallJoint: public Joint
{
public:

    BallJoint(dWorldID worldID, int mode = dAMotorEuler);

    void SetBallAnchor (dReal x, dReal y, dReal z);
    void SetBallAnchor(const char *buf);

    void SetStops(dReal a0Low, dReal a0High, dReal a1Low, dReal a1High, dReal a2Low, dReal a2High);
    void SetAxes(dReal x0, dReal y0, dReal z0, dReal x1, dReal y1, dReal z1, dReal x2, dReal y2, dReal z2, int axisMode);
    void SetAngles();
    void SetEulerReferenceVectors(dVector3 reference1, dVector3 reference2);

    void GetBallAnchor(dVector3 result);
    void GetBallAnchor2(dVector3 result);
    void GetEulerReferenceVectors(dVector3 reference1, dVector3 reference2);


    virtual void Update();
    virtual void Dump();

    // overridden method
    void Attach(dBodyID body1, dBodyID body2);

#ifdef USE_OPENGL
    virtual void Draw();
#endif

protected:

    dJointID m_MotorJointID;
    dJointFeedback m_MotorJointFeedback;
    int m_Mode;

};



#endif
