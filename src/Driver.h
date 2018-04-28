/*
 *  Driver.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat May 22 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 * Virtual class that all drivers descend from
 */

#ifndef Driver_h
#define Driver_h

#include "NamedObject.h"

class Drivable;

class Driver: public NamedObject
{
public:
    Driver();
    virtual ~Driver() ;

    void SetTarget(Drivable *target);
    Drivable *GetTarget() { return m_Target; }
    void SetMinMax(dReal minV, dReal maxV) { m_MinValue = minV; m_MaxValue = maxV; }

    virtual dReal GetValue(dReal time) = 0;

    virtual void Dump();

protected:

    Drivable *m_Target;
    dReal m_MinValue;
    dReal m_MaxValue;

};

#endif
