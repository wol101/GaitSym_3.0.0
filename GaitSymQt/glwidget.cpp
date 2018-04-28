#include <QtGui>
#include <QtOpenGL>
#include <iostream>
#include <string>
#include <QClipboard>
#include <QWheelEvent>
#include <QElapsedTimer>
#include <QFont>

#include <math.h>
#include <float.h>

#include "glwidget.h"
#include "trackball.h"
#include "Simulation.h"
#include "PGDMath.h"
#include "TIFFWrite.h"
#include "RayGeom.h"
#include "Contact.h"
#include "FacetedSphere.h"
#include "Body.h"
#include "dialoginterface.h"
#include "QTKitHelper.h"
#include "GLUtils.h"

// Simulation global
extern Simulation *gSimulation;

// output globals
extern bool gDestinationOpenGL;
extern bool gDestinationPOVRay;
extern std::ofstream *gPOVRayFile;
extern bool gDestinationOBJFile;
extern std::ofstream *gOBJFile;
extern int gVertexOffset;

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    XYAspect = XYAspect = static_cast<GLfloat>(width()) / static_cast<GLfloat>(height());
    cameraDistance = 50;
    frontClip = 1;
    backClip = 100;
    FOV = 5;
    COIx = COIy = COIz = 0;
    cameraVecX = 0;
    cameraVecY = 1;
    cameraVecZ = 0;
    upX = 0;
    upY = 0;
    upZ = 1;
    overlayFlag = false;
    movieFormat = GLWidget::Quicktime;
    yUp = false;
    frontClip = 0;
    backClip = 1;
    cursorRadius = 0.001;
    m3DCursorNudge = 0.001;
    cursorSphere = new FacetedSphere(cursorRadius, 4);
    cursorSphere->SetColour(1, 1, 1, 1);

    SetWhiteBackground(false);

    trackball = new Trackball();
    mTrackball = false;
    mPan = false;
    mZoom = false;

    qtMovie = 0;

    setCursor(Qt::CrossCursor);

    timer.start();
    displayFrameRate = false;

    bitmap = new QImage(256, 32, QImage::Format_Mono);
    painter = new QPainter(bitmap);
    font = new QFont("Courier", 18);
    openGLBitmap = new GLubyte[256 * 32 / 8];
    font->setStyleStrategy(QFont::StyleStrategy(QFont::PreferBitmap | QFont::NoAntialias | QFont::PreferQuality));
    painter->setFont(*font);
    painter->setPen(QPen(QColor(255, 255, 255), 1));

    setFocusPolicy(Qt::StrongFocus);
}

GLWidget::~GLWidget()
{
    makeCurrent();
    delete trackball;
    ClearLights();
    delete cursorSphere;
    delete [] openGLBitmap;
    delete font;
    delete painter;
    delete bitmap;
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(400, 400);
}

void GLWidget::initializeGL()
{
#if defined(__WIN32__)
    InitWindowsMissingGLFunctions();
#endif
}

void GLWidget::paintGL()
{
    //Set up OpenGL lights
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);

    SetupLights();

    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_FLAT);
    glFrontFace(GL_CCW); // counter clockwise is front facing
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK); // note culling has nothing to do with normals, it is done on winding order
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);

    // antialiasing is very slow
    //glEnable(GL_POINT_SMOOTH);
    //glEnable(GL_LINE_SMOOTH);
    //glEnable(GL_POLYGON_SMOOTH);

    glClearColor( backRed, backGreen, backBlue, backAlpha );
    if (overlayFlag == false)
    {
        glDrawBuffer(GL_BACK);
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    }
    else
    {
        glDrawBuffer(GL_FRONT_AND_BACK);
    }

    // set up GL_PROJECTION matrix to describe the attributes of the camera,
    // such as field of view, focal length, fish eye lens
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective(FOV, XYAspect, frontClip, backClip);
    // std::cerr << "frontClip " << frontClip << " backClip " << backClip << "\n";

    // set up GL_MODELVIEW matrix as where you stand with the camera and the direction you point it.
    glMatrixMode( GL_MODELVIEW );

    glLoadIdentity();
    gluLookAt(COIx - cameraVecX * cameraDistance, COIy - cameraVecY * cameraDistance, COIz - cameraVecZ * cameraDistance,
               COIx, COIy, COIz,
               upX, upY, upZ);

    // draw the lights
    DrawLights();

    // now draw things
    if (gSimulation) gSimulation->Draw();

    // the 3d cursor
    cursorSphere->SetDisplayPosition(m3DCursor.x, m3DCursor.y, m3DCursor.z);
    cursorSphere->Draw();

    // any manipulation feedback
    if (mZoom)
    {
        // line drawing lighting mode
        glDisable(GL_LIGHTING);
        glColor4f(1, 1, 1, 1);

        // raster mode positioning
        // with origin at top left
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width(), height(), 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        double centreX = double(width()) / 2;
        double centreY = double(height()) / 2;
        double xMouse = double(mMouseX) - centreX;
        double yMouse = double(mMouseY) - centreY;
        double radius = sqrt(xMouse * xMouse + yMouse * yMouse);

        glBegin(GL_LINE_LOOP);
        GLfloat x, y;
        for (int angle = 0; angle < 360; angle += 2)
        {
                x = centreX + radius * cos ((double)angle * M_PI / 180);
                y = centreY + radius * sin ((double)angle * M_PI / 180);
                glVertex2f(x, y);
        }
        glEnd();

    }

    if (mTrackball  && trackball->GetOutsideRadius())
    {
        // line drawing lighting mode
        glDisable(GL_LIGHTING);
        glColor4f(1, 1, 1, 1);

        // raster mode positioning
        // with origin at top left
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width(), height(), 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        double centreX = double(width()) / 2;
        double centreY = double(height()) / 2;
        double radius = trackball->GetTrackballRadius();

        glBegin(GL_LINE_LOOP);
        GLfloat x, y;
        for (int angle = 0; angle < 360; angle += 2)
        {
            x = centreX + radius * cos ((double)angle * M_PI / 180);
            y = centreY + radius * sin ((double)angle * M_PI / 180);
            glVertex2f(x, y);
        }
        glEnd();
    }

    // timing information
    qint64 msSinceLastPaint = timer.restart();
    double frameRate;
    if (msSinceLastPaint) frameRate = 1000.0 / double(msSinceLastPaint);
    else frameRate = 0;
    if (displayFrameRate)
    {
        QColor textColour = DialogInterface::getIdealTextColour(QColor(backRed * 255.0, backGreen * 255.0, backBlue * 255.0, backAlpha * 255.0));
        glDisable(GL_LIGHTING);
        glColor4f(textColour.redF(), textColour.greenF(), textColour.blueF(), textColour.alphaF());
        bitmap->fill(0);
        painter->drawText ( 0, 22, QString("Framerate: %1").arg(frameRate, 6, 'f', 3) );
        for (int l = 0; l < 32; l++) memcpy(openGLBitmap + (l * 256 / 8), bitmap->scanLine(31 - l), 256 / 8);
        glWindowPos2i(20, 10);
        glBitmap(256, 32, 0, 0, 0, 0, openGLBitmap);
    }

}

void GLWidget::resizeGL(int width, int height)
{
    glViewport( 0, 0, width, height );
    XYAspect = static_cast<GLfloat>(width) / static_cast<GLfloat>(height);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    if (gSimulation == 0) return;

    mMouseX = event->pos().x();
    mMouseY = event->pos().y();

    mTrackball = false;
    if (event->buttons() & Qt::LeftButton)
    {
        if (event->modifiers() == Qt::NoModifier)
        {
            int trackballRadius;
            if (width() < height()) trackballRadius = width() / 2.2;
            else trackballRadius = height() / 2.2;
            mTrackballStartCameraVec = pgd::Vector(cameraVecX, cameraVecY, cameraVecZ);
            mTrackballStartUp = pgd::Vector(upX, upY, upZ);
            trackball->StartTrackball(event->pos().x(), event->pos().y(), width() / 2, height() / 2, trackballRadius, mTrackballStartUp, -mTrackballStartCameraVec);
            mTrackball = true;
            emit EmitStatusString(tr("Rotate"));
            updateGL();
        }
        else if (event->modifiers() & Qt::ShiftModifier)
        {
            // create the collision ray
            GLdouble objX, objY, objZ;
            GLdouble modelMatrix[16];
            glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
            GLdouble projMatrix[16];
            glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
            int viewport[4];
            glGetIntegerv(GL_VIEWPORT,viewport);

            GLfloat winX, winY, winZ;
            winX = event->pos().x();
            winY = event->pos().y();
            winY = (GLfloat)viewport[3] - winY;

            pgd::Vector cameraPosition = pgd::Vector(COIx, COIy, COIz) - cameraDistance * pgd::Vector(cameraVecX, cameraVecY, cameraVecZ);
            dReal length = backClip;

#ifdef USE_EYE_BASED_PICKING // picking based on the coordinate of the surface under the mouse and the eye position
            glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
            gluUnProject(winX, winY, winZ,
                    modelMatrix, projMatrix, viewport,
                    &objX, &objY, &objZ );

            pgd::Vector rayDirection = pgd::Vector(objX, objY, objZ) - cameraPosition;
            rayDirection.Normalize();

            RayGeom rayGeom(0, length,
                            cameraPosition.x, cameraPosition.y, cameraPosition.z,
                            rayDirection.x, rayDirection.y, rayDirection.z);

#else // picking based on the near and far clipping planes
            winZ = frontClip;
            gluUnProject(winX, winY, winZ,
                    modelMatrix, projMatrix, viewport,
                    &objX, &objY, &objZ );
            pgd::Vector nearPoint(objX, objY, objZ);
            winZ = backClip;
            gluUnProject(winX, winY, winZ,
                    modelMatrix, projMatrix, viewport,
                    &objX, &objY, &objZ );
            pgd::Vector farPoint(objX, objY, objZ);

            pgd::Vector rayDirection = farPoint - nearPoint;
            rayDirection.Normalize();

            RayGeom rayGeom(0, length,
                            nearPoint.x, nearPoint.y, nearPoint.z,
                            rayDirection.x, rayDirection.y, rayDirection.z);
#endif
            rayGeom.SetParams(0, 0, 0); // firstcontact=0, backfacecull=0, closestHit = 0

            // code for collision detection
            pgd::Vector closestContact;
            std::vector<Geom *> *pickGeomList = gSimulation->GetPickGeomList();
            std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
            const int maxContacts = 128;
            dContactGeom contacts[maxContacts];
            int numCollisions;
            dReal distance2;
            pgd::Vector cameraRelVector;
            dReal minDistance2 = DBL_MAX;

            for (unsigned int j = 0; j < pickGeomList->size(); j++)
            {
                if ((*bodyList)[*(*pickGeomList)[j]->GetName()]->GetVisible())
                {

                    // std::cerr << *(*pickGeomList)[j]->GetName() << "\n";
                    numCollisions = dCollide (rayGeom.GetGeomID(), (*pickGeomList)[j]->GetGeomID(), maxContacts, contacts, sizeof(dContactGeom));
                    for (int i = 0; i < numCollisions; i++)
                    {
                        cameraRelVector = pgd::Vector(contacts[i].pos[0], contacts[i].pos[1], contacts[i].pos[2]) - cameraPosition;
                        distance2 = cameraRelVector.Magnitude2();
                        if (distance2 < minDistance2)
                        {
                            minDistance2 = distance2;
                            closestContact = pgd::Vector(contacts[i].pos[0], contacts[i].pos[1], contacts[i].pos[2]);
                        }
                    }
                }
            }

            if (minDistance2 < DBL_MAX)
            {
                Move3DCursor(closestContact.x, closestContact.y, closestContact.z);
                updateGL();
            }

        }
    }
    else if (event->buttons() & Qt::RightButton)
    {
        mPan = true;

        // get pick in world coordinates
        GLdouble objX, objY, objZ;
        // get and store the matrices from the start of panning
        glGetDoublev(GL_MODELVIEW_MATRIX,mPanModelMatrix);
        glGetDoublev(GL_PROJECTION_MATRIX,mPanProjMatrix);
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT,viewport);

        GLfloat winX, winY, winZ;
        winX = event->pos().x();
        winY = event->pos().y();
        winY = (GLfloat)viewport[3] - winY;
        glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
        panStartZ = winZ; // winZ seems to get set to 1 of clicked on blank area
        gluUnProject(winX, winY, winZ,
                     mPanModelMatrix, mPanProjMatrix, viewport,
                     &objX, &objY, &objZ );

        mPanStartVec = pgd::Vector(objX, objY, objZ);
        mPanStartCOI = pgd::Vector(COIx, COIy, COIz);

        emit EmitStatusString(tr("Pan"));
        updateGL();
    }
    else if (event->buttons() & Qt::MidButton)
    {
        mZoom = true;
        // centred -1 to -1 normalised values
        double x = (double)(2 * event->pos().x()) / (double)width() - 1.0;
        double y = (double)(2 * (height() - event->pos().y())) / (double)height() - 1.0;
        mZoomDistance = sqrt(x * x + y * y);
        if (mZoomDistance < 0.05) mZoomDistance = 0.05;
        mZoomStartFOV = FOV;

        emit EmitStatusString(tr("Zoom"));
        updateGL();
    }

}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    mMouseX = event->pos().x();
    mMouseY = event->pos().y();

    if (event->buttons() & Qt::LeftButton)
    {
        if (mTrackball)
        {
            pgd::Quaternion rotation;
            trackball->RollTrackballToClick(event->pos().x(), event->pos().y(), &rotation);
            pgd::Vector newCameraVec = pgd::QVRotate(~rotation, mTrackballStartCameraVec);
            cameraVecX = newCameraVec.x;
            cameraVecY = newCameraVec.y;
            cameraVecZ = newCameraVec.z;
            pgd::Vector newUp = pgd::QVRotate(~rotation, mTrackballStartUp);
            upX = newUp.x;
            upY = newUp.y;
            upZ = newUp.z;
            updateGL();

            emit EmitStatusString(QString("Camera %1 %2 %3 Up %4 %5 %6").arg(cameraVecX).arg(cameraVecY).arg(cameraVecZ).arg(upX).arg(upY).arg(upZ));
        }
    }
    else if (event->buttons() & Qt::RightButton)
    {
        if (mPan)
        {
            // get pick in world coordinates
            GLdouble objX, objY, objZ;
            int viewport[4];
            glGetIntegerv(GL_VIEWPORT,viewport);

            GLfloat winX, winY, winZ;
            winX = event->pos().x();
            winY = event->pos().y();
            winY = (GLfloat)viewport[3] - winY;
            winZ = panStartZ;
            gluUnProject(winX, winY, winZ,
                         mPanModelMatrix, mPanProjMatrix, viewport,
                         &objX, &objY, &objZ );
            // and move centre of interest by move
            //COIx -= (objX - mPanStartVec.x);
            //COIy -= (objY - mPanStartVec.y);
            //COIz -= (objZ - mPanStartVec.z);
            // and reset last position
            //mPanStartVec = pgd::Vector(objX, objY, objZ);
            COIx = mPanStartCOI.x - (objX - mPanStartVec.x);
            COIy = mPanStartCOI.y - (objY - mPanStartVec.y);
            COIz = mPanStartCOI.z - (objZ - mPanStartVec.z);
            updateGL();

            emit EmitStatusString(QString("COI %1 %2 %3").arg(COIx).arg(COIy).arg(COIz));
            emit EmitCOI(COIx, COIy, COIz);
        }
    }
    else if (event->buttons() & Qt::MidButton)
    {
        if (mZoom)
        {
            // centred -1 to -1 normalised values
            double x = (double)(2 * event->pos().x()) / (double)width() - 1.0;
            double y = (double)(2 * (height() - event->pos().y())) / (double)height() - 1.0;
            double zoomDistance = sqrt(x * x + y * y);
            FOV = mZoomStartFOV * mZoomDistance / zoomDistance;
            if (FOV > 170) FOV = 170;
            else if (FOV < 0.001) FOV = 0.001;
            updateGL();

            emit EmitStatusString(QString("FOV %1").arg(FOV));
            emit EmitFoV(FOV);
        }
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    mTrackball = false;
    mPan = false;
    mZoom = false;
    updateGL();
}

void GLWidget::wheelEvent(QWheelEvent * event)
{
    // assume each ratchet of the wheel gives a score of 120 (8 * 15 degrees)
    double sensitivity = 2400;
    double scale = 1.0 + double(event->delta()) / sensitivity;
    FOV *= scale;
    if (FOV > 170) FOV = 170;
    else if (FOV < 0.001) FOV = 0.001;
    updateGL();
    emit EmitStatusString(QString("FOV %1").arg(FOV));
    emit EmitFoV(FOV);
}

void GLWidget::SetCameraRight()
{
    if (yUp)
    {
        cameraVecX = 0;
        cameraVecY = 0;
        cameraVecZ = -1;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
    else
    {
        cameraVecX = 0;
        cameraVecY = 1;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
}

void GLWidget::SetCameraTop()
{
    if (yUp)
    {
        cameraVecX = 0;
        cameraVecY = 1;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
    else
    {
        cameraVecX = 0;
        cameraVecY = 0;
        cameraVecZ = -1;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
}

void GLWidget::SetCameraFront()
{
    if (yUp)
    {
        cameraVecX = -1;
        cameraVecY = 0;
        cameraVecZ = 0;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
    else
    {
        cameraVecX = -1;
        cameraVecY = 0;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
}

void GLWidget::SetWhiteBackground(bool v)
{
    if (v)
    {
        backRed = backGreen = backBlue = backAlpha = 1.0;
    }
    else
    {
        backRed = backGreen = backBlue = 0.0;
        backAlpha = 1.0;
    }
}

void GLWidget::SetCameraVec(double x, double y, double z)
{
    if (yUp)
    {
        cameraVecX = x;
        cameraVecY = z;
        cameraVecZ = -y;
        if (z > 0.999 || z < -0.999)
        {
            upX = 0;
            upY = 0;
            upZ = 1;
        }
        else
        {
            upX = 0;
            upY = 1;
            upZ = 0;
        }
   }
    else
    {
        cameraVecX = x;
        cameraVecY = y;
        cameraVecZ = z;
        if (z > 0.999 || z < -0.999)
        {
            upX = 0;
            upY = 1;
            upZ = 0;
        }
        else
        {
            upX = 0;
            upY = 0;
            upZ = 1;
        }
    }
    updateGL(); // this one is needed because this routine is only called from the ViewControlWidget and it will generally cause a redraw
}

// write the current frame out to a file
int GLWidget::WriteFrame(QString filename)
{
    if (qtMovie)
    {
        unsigned char *rgb = new unsigned char[width() * height() * 3];
        unsigned char *rgb2 = new unsigned char[width() * height() * 3];
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width(), height(), GL_RGB, GL_UNSIGNED_BYTE, rgb);
        // need to invert write order
        for (int i = 0; i < height(); i ++)
            memcpy(rgb2 + ((height() - i - 1) * width() * 3), rgb + (i * width() * 3), width() * 3);
        addImageToMovie(qtMovie, width(), height(), rgb2);
        delete [] rgb;
        delete [] rgb2;
        return 0;
    }

    std::string pathname(filename.toAscii());
    if (movieFormat == PPM)
    {
        unsigned char *rgb = new unsigned char[width() * height() * 3];
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width(), height(), GL_RGB, GL_UNSIGNED_BYTE, rgb);
        std::string ppmpathname = pathname + ".ppm";
        WritePPM(ppmpathname.c_str(), width(), height(), (unsigned char *)rgb);
        delete [] rgb;
    }
    if (movieFormat == TIFF)
    {
        unsigned char *rgb = new unsigned char[width() * height() * 3];
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width(), height(), GL_RGB, GL_UNSIGNED_BYTE, rgb);
        std::string tiffpathname = pathname + ".tif";
        WriteTIFF(tiffpathname.c_str(), width(), height(), (unsigned char *)rgb);
        delete [] rgb;
    }
    if (movieFormat == POVRay)
    {
        gDestinationOpenGL = false;
        gDestinationPOVRay = true;
        gDestinationOBJFile = false;
        std::string povpathname = pathname + ".pov";
        gPOVRayFile = new std::ofstream(povpathname.c_str());
        (*gPOVRayFile) << "#declare gCameraX = " << -cameraVecX * cameraDistance << " ;\n";
        (*gPOVRayFile) << "#declare gCameraY = " << -cameraVecY * cameraDistance << " ;\n";
        (*gPOVRayFile) << "#declare gCameraZ = " << -cameraVecZ * cameraDistance << " ;\n";
        (*gPOVRayFile) << "#declare gCOIx = " << COIx << " ;\n";
        (*gPOVRayFile) << "#declare gCOIy = " << COIy << " ;\n";
        (*gPOVRayFile) << "#declare gCOIz = " << COIz << " ;\n";
        (*gPOVRayFile) << "#declare gUpX = " << upX << " ;\n";
        (*gPOVRayFile) << "#declare gUpY = " << upY << " ;\n";
        (*gPOVRayFile) << "#declare gUpZ = " << upZ << " ;\n";
        (*gPOVRayFile) << "#declare gTime = " << gSimulation->GetTime() << " ;\n";

        (*gPOVRayFile) << "\n#include \"camera.pov\"\n\n";
        gSimulation->Draw();
        delete gPOVRayFile;
        gPOVRayFile = 0;
        gDestinationOpenGL = true;
        gDestinationPOVRay = false;
        gDestinationOBJFile = false;
    }
    if (movieFormat == OBJ)
    {
        gVertexOffset = 0;
        gDestinationOpenGL = false;
        gDestinationPOVRay = false;
        gDestinationOBJFile = true;
        std::string objpathname = pathname + ".obj";
        gOBJFile = new std::ofstream(objpathname.c_str());
        gSimulation->Draw();
        delete gOBJFile;
        gOBJFile = 0;
        gDestinationOpenGL = true;
        gDestinationPOVRay = false;
        gDestinationOBJFile = false;
    }

    return 0;
}

// write a PPM file (need to invert the y axis)
void GLWidget::WritePPM(const char *pathname, int width, int height, unsigned char *rgb)
{
    FILE *out;
    int i;

    out = fopen(pathname, "wb");

    // need to invert write order
    fprintf(out, "P6\n%d %d\n255\n", width, height);
    for (i = height - 1; i >= 0; i--)
        fwrite(rgb + (i * width * 3), width * 3, 1, out);

    fclose(out);
}

// write a TIFF file
void GLWidget::WriteTIFF(const char *pathname, int width, int height, unsigned char *rgb)
{
    TIFFWrite tiff;
    int i;

    tiff.initialiseImage(width, height, 72, 72, 3);
    // need to invert write order
    for (i = 0; i < height; i ++)
        tiff.copyRow(height - i - 1, rgb + (i * width * 3));

    tiff.writeToFile((char *)pathname);
}

// handle key presses
void GLWidget::keyPressEvent( QKeyEvent *e )
{
    switch( e->key() )
    {

        // X, Y and Z move the cursor
    case Qt::Key_X:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.x += m3DCursorNudge;
        else
            m3DCursor.x -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        updateGL();
        break;

    case Qt::Key_Y:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.y += m3DCursorNudge;
        else
            m3DCursor.y -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        updateGL();
        break;

    case Qt::Key_Z:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.z += m3DCursorNudge;
        else
            m3DCursor.z -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        updateGL();
        break;

        // S snaps the cursor to the nearest whole number multiple of the nudge value
    case Qt::Key_S:
        m3DCursor.x = round(m3DCursor.x / m3DCursorNudge) * m3DCursorNudge;
        m3DCursor.y = round(m3DCursor.y / m3DCursorNudge) * m3DCursorNudge;
        m3DCursor.z = round(m3DCursor.z / m3DCursorNudge) * m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        updateGL();
        break;

    default:
        QGLWidget::keyPressEvent( e );
    }
}

// use enter events to grab the keyboard
void GLWidget::enterEvent ( QEvent * event )
{
    grabKeyboard();
}

// use leave events to release the keyboard
void GLWidget::leaveEvent ( QEvent * event )
{
    releaseKeyboard();
}

// set the 3D cursor position
void GLWidget::Move3DCursor(double x, double y, double z)
{
    m3DCursor = pgd::Vector(x, y, z);
    QClipboard *clipboard = QApplication::clipboard();
    clipboard->setText(QString("%1\t%2\t%3").arg(m3DCursor.x).arg(m3DCursor.y).arg(m3DCursor.z), QClipboard::Clipboard);
    emit EmitStatusString(QString("3D Cursor %1\t%2\t%3").arg(m3DCursor.x).arg(m3DCursor.y).arg(m3DCursor.z));
}

// add a new light to the scene
void GLWidget::AddLight(Light *light)
{
    Light *newLight = new Light();
    *newLight = *light;
    mLightList.push_back(newLight);
}

// clear the light list
void GLWidget::ClearLights()
{
    for (unsigned int i = 0; i < mLightList.size(); i++) delete mLightList[i];
    mLightList.clear();
}

// set the lights up
void GLWidget::SetupLights()
{
    for (unsigned int i = 0; i < mLightList.size(); i++)
    {
        switch (i)
        {
        case 0:
            glEnable(GL_LIGHT0);
            glLightfv(GL_LIGHT0, GL_AMBIENT, mLightList[i]->ambient);
            glLightfv(GL_LIGHT0, GL_DIFFUSE, mLightList[i]->diffuse);
            glLightfv(GL_LIGHT0, GL_SPECULAR, mLightList[i]->specular);
            break;

        case 1:
            glEnable(GL_LIGHT1);
            glLightfv(GL_LIGHT1, GL_AMBIENT, mLightList[i]->ambient);
            glLightfv(GL_LIGHT1, GL_DIFFUSE, mLightList[i]->diffuse);
            glLightfv(GL_LIGHT1, GL_SPECULAR, mLightList[i]->specular);
            break;

        case 2:
            glEnable(GL_LIGHT2);
            glLightfv(GL_LIGHT2, GL_AMBIENT, mLightList[i]->ambient);
            glLightfv(GL_LIGHT2, GL_DIFFUSE, mLightList[i]->diffuse);
            glLightfv(GL_LIGHT2, GL_SPECULAR, mLightList[i]->specular);
            break;

        case 3:
            glEnable(GL_LIGHT3);
            glLightfv(GL_LIGHT3, GL_AMBIENT, mLightList[i]->ambient);
            glLightfv(GL_LIGHT3, GL_DIFFUSE, mLightList[i]->diffuse);
            glLightfv(GL_LIGHT3, GL_SPECULAR, mLightList[i]->specular);
            break;

        case 4:
            glEnable(GL_LIGHT4);
            glLightfv(GL_LIGHT4, GL_AMBIENT, mLightList[i]->ambient);
            glLightfv(GL_LIGHT4, GL_DIFFUSE, mLightList[i]->diffuse);
            glLightfv(GL_LIGHT4, GL_SPECULAR, mLightList[i]->specular);
            break;

        case 5:
            glEnable(GL_LIGHT5);
            glLightfv(GL_LIGHT5, GL_AMBIENT, mLightList[i]->ambient);
            glLightfv(GL_LIGHT5, GL_DIFFUSE, mLightList[i]->diffuse);
            glLightfv(GL_LIGHT5, GL_SPECULAR, mLightList[i]->specular);
            break;

        case 6:
            glEnable(GL_LIGHT6);
            glLightfv(GL_LIGHT6, GL_AMBIENT, mLightList[i]->ambient);
            glLightfv(GL_LIGHT6, GL_DIFFUSE, mLightList[i]->diffuse);
            glLightfv(GL_LIGHT6, GL_SPECULAR, mLightList[i]->specular);
            break;

        case 7:
            glEnable(GL_LIGHT7);
            glLightfv(GL_LIGHT7, GL_AMBIENT, mLightList[i]->ambient);
            glLightfv(GL_LIGHT7, GL_DIFFUSE, mLightList[i]->diffuse);
            glLightfv(GL_LIGHT7, GL_SPECULAR, mLightList[i]->specular);
            break;

        }

    }

}

// draw the lights up
void GLWidget::DrawLights()
{
    for (unsigned int i = 0; i < mLightList.size(); i++)
    {
        switch (i)
        {
        case 0:
            glLightfv(GL_LIGHT0, GL_POSITION, mLightList[i]->position);
            break;

        case 1:
            glLightfv(GL_LIGHT1, GL_POSITION, mLightList[i]->position);
            break;

        case 2:
            glLightfv(GL_LIGHT2, GL_POSITION, mLightList[i]->position);
            break;

        case 3:
            glLightfv(GL_LIGHT3, GL_POSITION, mLightList[i]->position);
            break;

        case 4:
            glLightfv(GL_LIGHT4, GL_POSITION, mLightList[i]->position);
            break;

        case 5:
            glLightfv(GL_LIGHT5, GL_POSITION, mLightList[i]->position);
            break;

        case 6:
            glLightfv(GL_LIGHT6, GL_POSITION, mLightList[i]->position);
            break;

        case 7:
            glLightfv(GL_LIGHT7, GL_POSITION, mLightList[i]->position);
            break;

        }

    }

}

void GLWidget::Set3DCursorRadius(double v)
{
    if (v >= 0)
    {
        cursorRadius = v;
        delete cursorSphere;
        cursorSphere = new FacetedSphere(cursorRadius, 4);
        cursorSphere->SetColour(1, 1, 1, 1);
    }
}


