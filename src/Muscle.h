/*
 *  Muscle.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Muscle_h
#define Muscle_h

#include "Drivable.h"
#include "Strap.h"
#include "Simulation.h"
#include <vector>

class Muscle:public Drivable
{
public:

    Muscle(Strap *strap);
    virtual ~Muscle();

    dReal GetLength() { return m_Strap->GetLength(); }
    dReal GetVelocity() { return m_Strap->GetVelocity(); }
    dReal GetTension() { return m_Strap->GetTension(); }
    dReal GetPower() { return -m_Strap->GetTension() * m_Strap->GetVelocity(); }

    void CalculateStrap(dReal deltaT) { m_Strap->Calculate(deltaT); }

    virtual dReal GetActivation() = 0;
    virtual dReal GetMetabolicPower() = 0;

    std::vector<PointForce *> *GetPointForceList() { return m_Strap->GetPointForceList(); }

    Strap *GetStrap() { return m_Strap; }

    void SetAllVisible(bool v) { SetVisible(v);  m_Strap->SetVisible(v); }

    virtual int SanityCheck(Muscle *otherMuscle, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight)
    {
        return m_Strap->SanityCheck(otherMuscle->m_Strap, axis, sanityCheckLeft, sanityCheckRight);
    }

#ifdef USE_OPENGL
    virtual void Draw();
    void SetForceColour(Colour &colour) { m_ForceColour = colour; };
    const Colour *GetForceColour() { return &m_ForceColour; };
#endif

protected:

    Strap *m_Strap;

#ifdef USE_OPENGL
    Colour m_ForceColour;
#endif

};

#endif

