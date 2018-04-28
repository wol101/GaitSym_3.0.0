/*
 *  Drivable.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 6/3/2011.
 *  Copyright 2011 Bill Sellers. All rights reserved.
 *
 */

#ifndef DRIVABLE_H
#define DRIVABLE_H

#include "NamedObject.h"

class Drivable : public NamedObject
{
public:
    Drivable();

    virtual void SetActivation(dReal activation, dReal duration) = 0;

};

#endif // DRIVABLE_H
