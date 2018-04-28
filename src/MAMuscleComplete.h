/*
 *  MAMuscleComplete.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 03/03/2007.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleComplete - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a serial and parallel spring element
// plus length tension stuff too


#ifndef MAMuscleComplete_h
#define MAMuscleComplete_h

#ifdef USE_GSL

#include "Muscle.h"
#include <gsl/gsl_roots.h>

// this struct contains all the paramers required for the CalculateForceError function
struct CalculateForceErrorParams
{
    // input parameters
    double spe; // slack length parallel element (m)
    double epe; // elastic constant parallel element (N/m)
    double sse; // slack length serial element (m)
    double ese; // elastic constant serial element (N/m)
    double k; // shape constant
    double vmax; // maximum shortening velocity (m/s)
    double fmax; // maximum isometric force (N)
    double width; // relative width of length/tension peak
    double alpha; // proportion of muscle activated
    double timeIncrement; // inegration step size for simulation (s)
    double len; // length of the whole element (m)
    double lastlpe; // last calculated lpe value (m)

    // output parameters
        double fce; // contractile force (N)
    double lpe; // contractile and parallel length (m)
    double fpe; // parallel element force (N)
    double lse; // serial length (m)
    double fse; // serial element force (N)
    double vce; // contractile element velocity (m/s)
    double targetFce; // fce calculated from elastic elements (N)
    double f0; // length corrected fmax (N)
};


class Strap;
class MAMuscle;
class DampedSpringMuscle;
class SimpleStrap;
class Filter;

class MAMuscleComplete : public Muscle
{
public:

    MAMuscleComplete(Strap *strap);
    ~MAMuscleComplete();

    void SetSerialElasticProperties(double springConstant, double unloadedLength);
    void SetParallelElasticProperties(double springConstant, double unloadedLength);
    void SetMuscleProperties(double vMax, double Fmax, double K, double Width);
    void SetActivationKinetics(bool activationKinetics) { m_ActivationKinetics = activationKinetics; };

    virtual double GetMetabolicPower();

    virtual void SetActivation(double activation, double timeIncrement);
    virtual double GetActivation() { return m_Stim; };

    double GetFCE() { return m_Params.fce; };
    double GetLPE() { return m_Params.lpe; };
    double GetFPE() { return m_Params.fpe; };
    double GetLSE() { return m_Params.lse; };
    double GetFSE() { return m_Params.fse; };
    double GetVCE() { return m_Params.vce; };
    double GetVPE() { return m_Params.vce; };
    double GetVSE() { return GetVelocity() - m_Params.vce; };
    double GetESE() { if (m_Params.lse > m_Params.sse) return (0.5 * SQUARE(m_Params.lse - m_Params.sse) * m_Params.ese); else return 0; }; // energy serial element
    double GetEPE() { if (m_Params.lpe > m_Params.spe) return (0.5 * SQUARE(m_Params.lpe - m_Params.spe) * m_Params.epe); else return 0; }; // energy parallel element
    double GetPSE() { return GetVSE() * -m_Params.fse; }; // power serial element
    double GetPPE() { return GetVPE() * -m_Params.fpe; }; // power parallel element
    double GetPCE() { return GetVCE() * -m_Params.fce; }; // power contractile element

protected:

    double m_Stim;
    bool m_ActivationKinetics;

    CalculateForceErrorParams m_Params;
    gsl_root_fsolver *m_gsl_root_fsolver;
    double m_Tolerance;
};






#endif // USE_GSL

#endif // MAMuscleComplete_h
