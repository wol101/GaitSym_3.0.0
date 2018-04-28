/*
 *  PIDMuscleLength.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 6/3/2011.
 *  Copyright 2011 Bill Sellers. All rights reserved.
 *
 */
#ifndef PIDMUSCLELENGTH_H
#define PIDMUSCLELENGTH_H

#include "Controller.h"

class Muscle;

class PIDMuscleLength : public Controller
{
public:
    PIDMuscleLength();

    void SetMuscle(Muscle *muscle) { m_Muscle = muscle; }
    void SetPID(dReal P, dReal I, dReal D) { Kp = P; Ki = I; Kd = D; }

    void SetNominalLength(dReal length) { nomimal_length = length; }

    virtual void SetActivation(dReal activation, dReal duration);
    virtual dReal GetActivation() { return activation; }

protected:
    Muscle *m_Muscle;
    dReal Kp;
    dReal Ki;
    dReal Kd;
    dReal setpoint;
    dReal previous_error;
    dReal error;
    dReal integral;
    dReal derivative;
    dReal output;
    dReal activation;
    dReal last_activation;
    dReal nomimal_length;

};

#endif // PIDMUSCLELENGTH_H
