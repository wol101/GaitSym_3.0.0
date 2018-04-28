/*
 *  UniversalJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 21/12/2010.
 *  Copyright 2010 Bill Sellers. All rights reserved.
 *
 */

#ifndef UniversalJoint_h
#define UniversalJoint_h

#include "Joint.h"

class UniversalJoint: public Joint
{
public:

    UniversalJoint(dWorldID worldID);

    void SetUniversalAnchor (dReal x, dReal y, dReal z);
    void SetUniversalAxis1(dReal x, dReal y, dReal z);
    void SetUniversalAxis2(dReal x, dReal y, dReal z);
    void SetUniversalAnchor (const char *buf);
    void SetUniversalAxis1(const char *buf);
    void SetUniversalAxis2(const char *buf);

    void SetStartAngleReference1(dReal startAngleReference);
    void SetStartAngleReference2(dReal startAngleReference);

    void GetUniversalAnchor(dVector3 result);
    void GetUniversalAnchor2(dVector3 result);
    void GetUniversalAxis1(dVector3 result);
    void GetUniversalAxis2(dVector3 result);

    dReal GetUniversalAngle1();
    dReal GetUniversalAngle1Rate();
    dReal GetUniversalAngle2();
    dReal GetUniversalAngle2Rate();

    virtual void Dump();

#ifdef USE_OPENGL
    virtual void Draw();
#endif

protected:

    dReal m_StartAngleReference1;
    dReal m_StartAngleReference2;

};



#endif
