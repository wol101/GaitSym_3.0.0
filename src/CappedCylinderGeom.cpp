/*
 *  CappedCylinderGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "CappedCylinderGeom.h"

#ifdef USE_OPENGL
#include "FacetedCappedCylinder.h"
extern int gDrawGeomsFlag;
extern std::string gOBJName;
#endif

CappedCylinderGeom::CappedCylinderGeom(dSpaceID space, dReal radius, dReal length)
{
    // create the geom
#ifdef OLD_ODE
    m_GeomID = dCreateCCylinder(0, radius, length);
#else
    m_GeomID = dCreateCapsule(space, radius, length);
#endif
    dGeomSetData(m_GeomID, this);
}

#ifdef USE_OPENGL
void CappedCylinderGeom::Draw()
{
    if (m_Visible == false) return;

    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        // get the length and radius
        dReal radius, length;
#ifdef OLD_ODE
        dGeomCCylinderGetParams (m_GeomID, &radius, &length);
#else
        dGeomCapsuleGetParams (m_GeomID, &radius, &length);
#endif

        int capped_cylinder_quality = 32;
        m_PhysRep = new FacetedCappedCylinder(length, radius, capped_cylinder_quality);
        m_PhysRep->SetColour(m_Colour);


    }

    if (gDrawGeomsFlag)
    {
        gOBJName = m_Name;

        const dReal *bodyRotation = dBodyGetRotation(dGeomGetBody(m_GeomID));
        const dReal *cylinderRelPosition = dGeomGetOffsetPosition(m_GeomID);
        const dReal *cylinderRelRotation = dGeomGetOffsetRotation(m_GeomID);

        dVector3 p;
        dMatrix3 r;

        // get the cylinder position in world coordinates
        dBodyGetRelPointPos(dGeomGetBody(m_GeomID), cylinderRelPosition[0],cylinderRelPosition[1], cylinderRelPosition[2], p);

        //combine the body rotation with the cylinder rotation to get combined rotation from world coordinates
        dMultiply0(r, bodyRotation, cylinderRelRotation, 3, 3, 3);

        // and draw the capped cylinder
        m_PhysRep->SetDisplayRotation(r);
        m_PhysRep->SetDisplayPosition(p[0], p[1], p[2]);
        m_PhysRep->Draw();
    }
}
#endif
