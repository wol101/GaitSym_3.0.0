#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>
#include <QBoxLayout>
#include <QDesktopWidget>
#include <QListWidgetItem>
#include <QSettings>
#include <QLineEdit>
#include <QFile>
#include <QKeyEvent>
#include <QRegExp>
#include <QDir>
#include <QStringList>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dialogpreferences.h"
#include "ui_dialogpreferences.h"
#include "dialoginterface.h"
#include "ui_dialoginterface.h"
#include "dialogoutputselect.h"
#include "ui_dialogoutputselect.h"

#include "glwidget.h"
#include "viewcontrolwidget.h"
#include "ObjectiveMain.h"
#include "Simulation.h"
#include "DataFile.h"
#include "Muscle.h"
#include "Body.h"
#include "Joint.h"
#include "Geom.h"
#include "Driver.h"
#include "DataTarget.h"
#include "FacetedObject.h"
#include "Reporter.h"

#ifdef USE_OPENCL
#include "OpenCLRoutines.h"
#endif

// Simulation global
extern Simulation *gSimulation;

// External display globals
extern int gDrawMuscleForces;
extern int gDrawContactForces;
extern int g_ActivationDisplay;
extern int g_UseOpenCL;

// external settings
extern QSettings *settings;

// external file paths
extern char *gConfigFilenamePtr;
extern char *gGraphicsRoot;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    // initialise some class variables
    skip = 1;
    stepCount = 0;
    stepFlag = false;
    movieFlag = false;
    qtMovie = 0;
    trackingFlag = false;
    allowGLUpdateFlag = true;

    // create the window elements

    ui->setupUi(this);

    // position the elements

    QDesktopWidget *desktop = qApp->desktop();
    QRect available = desktop->availableGeometry(-1);

    // careful about the various Qt size functions
    // Including the window frame:
    // x(), y(), frameGeometry(), pos() and move()
    // Excluding the window frame:
    // geometry(), width(), height(), rect() and size()

#ifdef WINDOWS_NOT_OVERLAPPED
    ui->dockWidgetControls->move(available.left(), available.top());
    ui->dockWidgetVisibility->move(available.right() - ui->dockWidgetVisibility->frameGeometry().width(), available.top());

    move(available.left() + ui->dockWidgetControls->frameGeometry().width(), available.top());
    resize(available.width() - (ui->dockWidgetControls->frameGeometry().width() + ui->dockWidgetVisibility->frameGeometry().width()),
                 available.height() - (frameGeometry().height() - height()));
#else
    move(available.left(), available.top());
    resize(available.width() - (frameGeometry().width() - width()),
                 available.height() - (frameGeometry().height() - height()));

#ifdef __APPLE__
    int verticalSpace = (frameGeometry().height() - height());
#else
    int verticalSpace = (frameGeometry().height() - height() + ui->menuBar->height());
#endif
    ui->dockWidgetControls->setFixedSize(ui->dockWidgetControls->frameGeometry().width(),ui->dockWidgetControls->frameGeometry().height() - 30);
    ui->dockWidgetControls->move(available.left(), available.top() + verticalSpace);
    ui->dockWidgetVisibility->setFixedSize(ui->dockWidgetVisibility->frameGeometry().width(),ui->dockWidgetVisibility->frameGeometry().height() - 30);
    ui->dockWidgetVisibility->move(available.right() - ui->dockWidgetVisibility->frameGeometry().width(), available.top() + verticalSpace);
    ui->dockWidgetView->setFixedSize(ui->dockWidgetView->frameGeometry().width(),ui->dockWidgetView->frameGeometry().height() - 30);
    ui->dockWidgetView->move(available.right() - ui->dockWidgetView->frameGeometry().width(), available.top() + verticalSpace + ui->dockWidgetVisibility->frameGeometry().height());

    ui->dockWidgetLog->move(available.right() - ui->dockWidgetLog->frameGeometry().width(),
                            available.top() + verticalSpace + ui->dockWidgetVisibility->frameGeometry().height() + ui->dockWidgetView->frameGeometry().height());

#endif

    // put GLWidget into centralWidget
    boxLayout = new QBoxLayout(QBoxLayout::LeftToRight, ui->centralWidget);
    boxLayout->setMargin(0);
    glWidget = new GLWidget();
    boxLayout->addWidget(glWidget);

    // put ViewControlWidget into widgetViewFrame
    boxLayout2 = new QBoxLayout(QBoxLayout::LeftToRight, ui->widgetViewFrame);
    boxLayout2->setMargin(0);
    viewControlWidget = new ViewControlWidget();
    boxLayout2->addWidget(viewControlWidget);

    // connect the ViewControlWidget to the GLWidget
    QObject::connect(viewControlWidget, SIGNAL(EmitCameraVec(double, double, double)), glWidget, SLOT(SetCameraVec(double, double, double)));

    // connect the GLWidget to the MainWindow
    QObject::connect(glWidget, SIGNAL(EmitStatusString(QString)), this, SLOT(setStatusString(QString)));
    QObject::connect(glWidget, SIGNAL(EmitCOI(double, double, double)), this, SLOT(setUICOI(double, double, double)));
    QObject::connect(glWidget, SIGNAL(EmitFoV(double)), this, SLOT(setUIFoV(double)));

    // set up the timer
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(processOneThing()));

    // zero the timer
    QString time = QString("%1").arg(double(0), 0, 'f', 5);
    ui->lcdNumberTime->display(time);

    // read the settings
    ReadSettings();
    menuDefaultView();

    statusBar()->showMessage(tr("Ready"));

    // set menu activations for no loaded model
    ui->actionOutput->setEnabled(false);
    ui->actionStart_Quicktime_Save->setEnabled(false);
    ui->actionStop_Quicktime_Save->setEnabled(false);
    ui->action_Restart->setEnabled(false);
    ui->action_Save->setEnabled(false);
    ui->actionSave_as_World->setEnabled(false);

#ifndef __APPLE__
    // quicktime menus only available for Mac version
    ui->actionStart_Quicktime_Save->setVisible(false);
    ui->actionStop_Quicktime_Save->setVisible(false);
#endif

#ifdef USE_OPENCL
    try
    {
        OpenCLRoutines::InitCL();
    }
    catch (...)
    {
        statusBar()->showMessage(tr("Unable to initialise OpenCL - disabled"));
        ui->checkBoxOpenCL->setChecked(false);
        ui->checkBoxOpenCL->setDisabled(true);
        g_UseOpenCL = false;
    }
#else
    ui->checkBoxOpenCL->setChecked(false);
    ui->checkBoxOpenCL->setDisabled(true);
    g_UseOpenCL = false;
#endif
}

MainWindow::~MainWindow()
{
#ifdef USE_OPENCL
    try
    {
        OpenCLRoutines::ReleaseCL();
    }
    catch (...)
    {
        std::cerr << "Error releasing OpenCL\n";
    }
#endif

    settings->sync();
    timer->stop();

    if (gSimulation) delete gSimulation;
    if (qtMovie) closeMovieFile(qtMovie);
    delete timer;
    delete glWidget;
    delete boxLayout;
    delete viewControlWidget;
    delete boxLayout2;
    delete ui;

}

void MainWindow::open()
{
    QFileInfo info = settings->value("LastFileOpened", QString("")).toString();

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Config File"), info.absolutePath(), tr("Config Files (*.xml)"));

    if (fileName.isNull() == false)
    {
        if (gSimulation) delete gSimulation;
        gSimulation = 0;
        stepCount = 0;

        ui->toolButtonPlay->setChecked(false);
        run();

        settings->setValue("LastFileOpened", fileName);
        settings->sync();
        setDefaultLights(settings->value("YUp", 0).toBool());
        char *readFileName = new char[fileName.length() + 1];
        strcpy(readFileName, fileName.toAscii());
        gConfigFilenamePtr = readFileName;

        QString graphicsPath = settings->value("GraphicsPath", QString("")).toString();
        char *graphicsRoot = new char[graphicsPath.length() + 1];
        strcpy(graphicsRoot, graphicsPath.toAscii());
        gGraphicsRoot = graphicsRoot;

        configFile.setFile(fileName);
        QDir::setCurrent ( configFile.absolutePath() );

        int err = ReadModel();
        delete [] graphicsRoot;
        delete [] readFileName;
        if (err)
        {
            statusBar()->showMessage(QString("Error loading ") + fileName);
            return;
        }

        fillVisibitilityLists();
        ui->doubleSpinBoxTimeMax->setValue(gSimulation->GetTimeLimit());
        QString time = QString("%1").arg(double(0), 0, 'f', 5);
        ui->lcdNumberTime->display(time);
        checkboxTracking(trackingFlag);

        updateGL();
        statusBar()->showMessage(fileName + QString(" loaded"));

        // put the filename as a title
        if (fileName.size() <= 256) setWindowTitle(fileName);
        else setWindowTitle(QString("...") + fileName.right(256));

        // set menu activations for loaded model
        ui->actionOutput->setEnabled(true);
        ui->actionStart_Quicktime_Save->setEnabled(true);
        ui->actionStop_Quicktime_Save->setEnabled(false);
        ui->action_Restart->setEnabled(true);
        ui->action_Save->setEnabled(true);
        ui->actionSave_as_World->setEnabled(true);


    }
}

void MainWindow::restart()
{
    if (gSimulation == 0) return;
    delete gSimulation;
    gSimulation = 0;
    stepCount = 0;

    char *readFileName = new char[configFile.absoluteFilePath().length() + 1];
    strcpy(readFileName, configFile.absoluteFilePath().toAscii());
    gConfigFilenamePtr = readFileName;

    QString graphicsPath = settings->value("GraphicsPath", QString("")).toString();
    char *graphicsRoot = new char[graphicsPath.length() + 1];
    strcpy(graphicsRoot, graphicsPath.toAscii());
    gGraphicsRoot = graphicsRoot;

    ReadModel();
    delete [] graphicsRoot;
    delete [] readFileName;

    fillVisibitilityLists();
    ui->doubleSpinBoxTimeMax->setValue(gSimulation->GetTimeLimit());
    QString time = QString("%1").arg(double(0), 0, 'f', 5);
    ui->lcdNumberTime->display(time);
    checkboxTracking(trackingFlag);
    updateGL();
    statusBar()->showMessage(configFile.fileName() + QString(" reloaded"));

}

void MainWindow::saveas()
{
    QFileInfo info = settings->value("LastFileOpened", QString("")).toString();

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Model State File"), info.absolutePath(), tr("Config Files (*.xml)"));

    if (fileName.isNull() == false)
    {
        gSimulation->SetModelStateRelative(true);
        gSimulation->SetOutputModelStateFile(fileName.toAscii());
        gSimulation->OutputProgramState();
    }
}

void MainWindow::saveasworld()
{
    QFileInfo info = settings->value("LastFileOpened", QString("")).toString();

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Model State File"), info.absolutePath(), tr("Config Files (*.xml)"));

    if (fileName.isNull() == false)
    {
        gSimulation->SetModelStateRelative(false);
        gSimulation->SetOutputModelStateFile(fileName.toAscii());
        gSimulation->OutputProgramState();
    }
}


void MainWindow::about()
 {
     QMessageBox::about(this, tr("About Menu"),
             tr("<strong>GaitSymQt<strong> Version 2.2<br>Copyright &copy; 2009-2011 Bill Sellers<br><br>Written using Qt and ODE and released under the GNU Copyleft licence.<br><br>Source code and latest version available from http://www.animalsimulation.org<br><br>Funded by The Leverhulme Trust, NERC, BBSRC"));
 }

void MainWindow::run()
{
    if (ui->toolButtonPlay->isChecked())
    {
        if (gSimulation) timer->start();
        statusBar()->showMessage(tr("Simulation running"));
    }
    else
    {
        timer->stop();
        statusBar()->showMessage(tr("Simulation stopped"));
    }
}

void MainWindow::step()
{
    stepFlag = true;
    if (gSimulation) timer->start();
    statusBar()->showMessage(tr("Simulation stepped"));
}

void MainWindow::processOneThing()
{
    if (gSimulation)
    {
        if (gSimulation->ShouldQuit() || gSimulation->TestForCatastrophy())
        {
            statusBar()->showMessage(tr("Unable to start simulation"));
            ui->toolButtonPlay->setChecked(false);
            run();
            return;
        }

        gSimulation->UpdateSimulation();
        stepCount++;

        if ((stepCount % skip) == 0)
        {
            if (trackingFlag)
            {
                Body *body = gSimulation->GetBody(gSimulation->GetInterface()->TrackBodyID.c_str());
                if (body)
                {
                    const dReal *position = dBodyGetPosition(body->GetBodyID());
                    glWidget->SetCameraCOIX(position[0]);
                    allowGLUpdateFlag = false;
                    ui->doubleSpinBoxCOIX->setValue(position[0]);
                    allowGLUpdateFlag = true;
                }
            }
            if (stepFlag)
            {
                stepFlag = false;
                timer->stop();
            }
            updateGL();
            if (qtMovie)
            {
                glWidget->WriteFrame(QString());
            }
            if (movieFlag)
            {
                QString movieFullPath = QString("%1/%2").arg(configFile.absolutePath()).arg(movieFolder);
                if (QFile::exists(movieFullPath) == false)
                {
                    QDir dir("/");
                    dir.mkpath(movieFullPath);
                }
                QString filename = QString("%1/%2%3").arg(movieFullPath).arg("Frame").arg(gSimulation->GetTime(), 12, 'f', 7, QChar('0'));
                glWidget->WriteFrame(filename);
            }
            QString time = QString("%1").arg(gSimulation->GetTime(), 0, 'f', 5);
            ui->lcdNumberTime->display(time);
        }

        if (gSimulation->ShouldQuit())
        {
            statusBar()->showMessage(tr("Simulation ended normally"));
            ui->textEditLog->append(QString("Fitness = %1\n").arg(gSimulation->CalculateInstantaneousFitness(), 0, 'f', 5));
            updateGL();
            QString time = QString("%1").arg(gSimulation->GetTime(), 0, 'f', 5);
            ui->lcdNumberTime->display(time);
            ui->toolButtonPlay->setChecked(false);
            run();
            return;
        }
        if (gSimulation->TestForCatastrophy())
        {
            statusBar()->showMessage(tr("Simulation aborted"));
            ui->textEditLog->append(QString("Fitness = %1\n").arg(gSimulation->CalculateInstantaneousFitness(), 0, 'f', 5));
            updateGL();
            QString time = QString("%1").arg(gSimulation->GetTime(), 0, 'f', 5);
            ui->lcdNumberTime->display(time);
            ui->toolButtonPlay->setChecked(false);
            run();
            return;
        }
    }
}

void MainWindow::snapshot()
{
    int count = 0;
    QDir dir(configFile.absolutePath());
    QStringList list = dir.entryList(QDir::Files, QDir::Name);
    QStringList matches = list.filter(QRegExp(QString("^Snapshot\\d\\d\\d\\d\\d\\..*")));
    if (matches.size() > 0)
    {
        QString numberString = matches.last().mid(8, 5);
        count = numberString.toInt() + 1;
    }
    QString filename = QString("Snapshot%1").arg(count, 5, 10, QChar('0'));
    glWidget->WriteFrame(QString("%1/%2").arg(configFile.absolutePath()).arg(filename));
    statusBar()->showMessage(tr("Snapshot taken"));
}

void MainWindow::fillVisibitilityLists()
{
    if (gSimulation == 0) return;

    QListWidgetItem *item;
    int count;
    std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
    std::map<std::string, Joint *> *jointList = gSimulation->GetJointList();
    std::map<std::string, Geom *> *geomList = gSimulation->GetGeomList();
    std::map<std::string, Muscle *> *muscleList = gSimulation->GetMuscleList();

    count = 0;
    ui->listWidgetBody->clear();
    std::map<std::string, Body *>::const_iterator bodyIterator;
    for (bodyIterator = bodyList->begin(); bodyIterator != bodyList->end(); bodyIterator++)
    {
        ui->listWidgetBody->addItem(bodyIterator->first.c_str());
        item = ui->listWidgetBody->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetJoint->clear();
    std::map<std::string, Joint *>::const_iterator jointIterator;
    for (jointIterator = jointList->begin(); jointIterator != jointList->end(); jointIterator++)
    {
        ui->listWidgetJoint->addItem(jointIterator->first.c_str());
        item = ui->listWidgetJoint->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetGeom->clear();
    std::map<std::string, Geom *>::const_iterator geomIterator;
    for (geomIterator = geomList->begin(); geomIterator != geomList->end(); geomIterator++)
    {
        ui->listWidgetGeom->addItem(geomIterator->first.c_str());
        item = ui->listWidgetGeom->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetMuscle->clear();
    std::map<std::string, Muscle *>::const_iterator muscleIterator;
    for (muscleIterator = muscleList->begin(); muscleIterator != muscleList->end(); muscleIterator++)
    {
        ui->listWidgetMuscle->addItem(muscleIterator->first.c_str());
        item = ui->listWidgetMuscle->item(count++);
        item->setCheckState(Qt::Checked);
    }


}

void MainWindow::buttonCameraRight()
{
    glWidget->SetCameraRight();
    updateGL();
}


void MainWindow::buttonCameraTop()
{
    glWidget->SetCameraTop();
    updateGL();
}


void MainWindow::buttonCameraFront()
{
    glWidget->SetCameraFront();
    updateGL();
}


void MainWindow::spinboxDistanceChanged(double v)
{
    glWidget->SetCameraDistance(v);
    updateGL();
}


void MainWindow::spinboxFoVChanged(double v)
{
    glWidget->SetCameraFoV(v);
    updateGL();
}


void MainWindow::spinboxCOIXChanged(double v)
{
    glWidget->SetCameraCOIX(v);
    updateGL();
}


void MainWindow::spinboxCOIYChanged(double v)
{
    glWidget->SetCameraCOIY(v);
    updateGL();
}


void MainWindow::spinboxCOIZChanged(double v)
{
    glWidget->SetCameraCOIZ(v);
    updateGL();
}


void MainWindow::checkboxTracking(int v)
{
    trackingFlag = v;
    if (gSimulation)
    {
        if (trackingFlag)
        {
            Body *body = gSimulation->GetBody(gSimulation->GetInterface()->TrackBodyID.c_str());
            if (body)
            {
                const dReal *position = dBodyGetPosition(body->GetBodyID());
                glWidget->SetCameraCOIX(position[0]);
                ui->doubleSpinBoxCOIX->setValue(position[0]);
                updateGL();
            }
        }
    }
}


void MainWindow::checkboxOverlay(int v)
{
    glWidget->SetOverlay(v);
    updateGL();
}


void MainWindow::checkboxContactForce(int v)
{
    gDrawContactForces = v;
    updateGL();
}


void MainWindow::checkboxMuscleForce(int v)
{
    gDrawMuscleForces = v;
    updateGL();
}


void MainWindow::checkboxWhiteBackground(int v)
{
    glWidget->SetWhiteBackground(v);
    updateGL();
}


void MainWindow::checkboxFramerate(int v)
{
    if (v) glWidget->SetDisplayFramerate(true);
    else glWidget->SetDisplayFramerate(false);
    updateGL();
}


void MainWindow::checkboxActivationColours(int v)
{
    g_ActivationDisplay = v;
    updateGL();
}

void MainWindow::checkboxOpenCL(int v)
{
    g_UseOpenCL = v;
}


void MainWindow::lineeditMovieFolder(QString folder)
{
    movieFolder = folder;
}


void MainWindow::checkboxRecordMovie(int v)
{
    movieFlag = v;
}


void MainWindow::radioPPM(bool v)
{
    if (v)
    {
        glWidget->SetMovieFormat(GLWidget::PPM);
    }
}


void MainWindow::radioTIFF(bool v)
{
   if (v)
    {
        glWidget->SetMovieFormat(GLWidget::TIFF);
    }
}


void MainWindow::radioPOVRay(bool v)
{
   if (v)
    {
        glWidget->SetMovieFormat(GLWidget::POVRay);
    }
}


void MainWindow::radioOBJ(bool v)
{
   if (v)
    {
        glWidget->SetMovieFormat(GLWidget::OBJ);
    }
}


void MainWindow::spinboxSkip(int v)
{
    skip = v;
}


void MainWindow::spinboxTimeMax(double v)
{
    gSimulation->SetTimeLimit(v);
}


void MainWindow::listMuscleChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetMuscleList())[std::string(item->text().toAscii())]->SetAllVisible(visible);
    updateGL();
}


void MainWindow::listBodyChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetBodyList())[std::string(item->text().toAscii())]->SetVisible(visible);
    updateGL();
}


void MainWindow::listJointChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetJointList())[std::string(item->text().toAscii())]->SetVisible(visible);
    updateGL();
}


void MainWindow::listGeomChecked(QListWidgetItem* item)
{
   bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetGeomList())[std::string(item->text().toAscii())]->SetVisible(visible);
    updateGL();
}

void MainWindow::menuPreferences()
{
    DialogPreferences dialogPreferences(this);
    Ui::DialogPreferences *dialogUI = dialogPreferences.ui();

    dialogUI->lineEditGraphicPath->setText(settings->value("GraphicsPath", QString("")).toString());
    dialogUI->lineEditMovieFolder->setText(settings->value("MoviePath", QString("movie")).toString());

    dialogUI->doubleSpinBoxFrontClip->setValue(settings->value("CameraFrontClip", 1).toDouble());
    dialogUI->doubleSpinBoxBackClip->setValue(settings->value("CameraBackClip", 100).toDouble());
    dialogUI->doubleSpinBoxCursorRadius->setValue(settings->value("CursorRadius", 0.001).toDouble());
    dialogUI->doubleSpinBoxNudge->setValue(settings->value("Nudge", 0.001).toDouble());

    dialogUI->spinBoxSkip->setValue(settings->value("MovieSkip", 1).toInt());
    dialogUI->doubleSpinBoxFramerate->setValue(settings->value("QuicktimeFramerate", 24).toDouble());

    dialogUI->checkBoxYUp->setChecked(settings->value("YUp", 0).toBool());
    dialogUI->checkBoxTracking->setChecked(settings->value("DisplayTracking", 0).toBool());
    dialogUI->checkBoxOverlay->setChecked(settings->value("DisplayOverlay", 0).toBool());
    dialogUI->checkBoxContactForce->setChecked(settings->value("DisplayContactForces", 0).toBool());
    dialogUI->checkBoxMuscleForce->setChecked(settings->value("DisplayMuscleForces", 0).toBool());
    dialogUI->checkBoxWhiteBackground->setChecked(settings->value("DisplayWhiteBackground", 1).toBool());
    dialogUI->checkBoxFramerate->setChecked(settings->value("DisplayFramerate", 0).toBool());
    dialogUI->checkBoxActivationColours->setChecked(settings->value("DisplayActivation", 0).toBool());
    dialogUI->checkBoxOpenCL->setChecked(settings->value("OpenCLUseOpenCL", 0).toBool());
    dialogUI->checkBoxOpenCLLog->setChecked(settings->value("OpenCLOpenCLLog", 0).toBool());

    dialogUI->radioButtonPPM->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::PPM);
    dialogUI->radioButtonTIFF->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::TIFF);
    dialogUI->radioButtonPOVRay->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::POVRay);
    dialogUI->radioButtonOBJ->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::OBJ);

    dialogUI->spinBoxOpenCLDeviceNumber->setValue(settings->value("OpenCLDeviceDeviceNumber", 0).toInt());
    dialogUI->spinBoxOpenCLTargetPlatform->setValue(settings->value("OpenCLTargetPlatform", 0).toInt());
    dialogUI->comboBoxOpenCLDevice->setCurrentIndex(dialogUI->comboBoxOpenCLDevice->findText(settings->value("OpenCLDeviceType", QString("CPU")).toString()));

    int status = dialogPreferences.exec();

    if (status == QDialog::Accepted) // write the new settings
    {
        settings->setValue("GraphicsPath", dialogUI->lineEditGraphicPath->text());
        settings->setValue("MoviePath", dialogUI->lineEditMovieFolder->text());

        settings->setValue("CameraFrontClip", dialogUI->doubleSpinBoxFrontClip->value());
        settings->setValue("CameraBackClip", dialogUI->doubleSpinBoxBackClip->value());
        settings->setValue("CursorRadius", dialogUI->doubleSpinBoxCursorRadius->value());
        settings->setValue("Nudge", dialogUI->doubleSpinBoxNudge->value());

        settings->setValue("MovieSkip", dialogUI->spinBoxSkip->value());
        settings->setValue("QuicktimeFramerate", dialogUI->doubleSpinBoxFramerate->value());

        settings->setValue("YUp", dialogUI->checkBoxYUp->isChecked());
        settings->setValue("DisplayTracking", dialogUI->checkBoxTracking->isChecked());
        settings->setValue("DisplayOverlay", dialogUI->checkBoxOverlay->isChecked());
        settings->setValue("DisplayContactForces", dialogUI->checkBoxContactForce->isChecked());
        settings->setValue("DisplayMuscleForces", dialogUI->checkBoxMuscleForce->isChecked());
        settings->setValue("DisplayWhiteBackground", dialogUI->checkBoxWhiteBackground->isChecked());
        settings->setValue("DisplayFramerate", dialogUI->checkBoxFramerate->isChecked());
        settings->setValue("DisplayActivation", dialogUI->checkBoxActivationColours->isChecked());
        settings->setValue("OpenCLUseOpenCL", dialogUI->checkBoxOpenCL->isChecked());
        settings->setValue("OpenCLOpenCLLog", dialogUI->checkBoxOpenCLLog->isChecked());

        if (dialogUI->radioButtonPPM->isChecked()) settings->setValue("MovieFormat", static_cast<int>(GLWidget::PPM));
        if (dialogUI->radioButtonTIFF->isChecked()) settings->setValue("MovieFormat", static_cast<int>(GLWidget::TIFF));
        if (dialogUI->radioButtonPOVRay->isChecked()) settings->setValue("MovieFormat", static_cast<int>(GLWidget::POVRay));
        if (dialogUI->radioButtonOBJ->isChecked()) settings->setValue("MovieFormat", static_cast<int>(GLWidget::OBJ));

        settings->setValue("OpenCLDeviceDeviceNumber", dialogUI->spinBoxOpenCLDeviceNumber->value());
        settings->setValue("OpenCLTargetPlatform", dialogUI->spinBoxOpenCLTargetPlatform->value());
        settings->setValue("OpenCLDeviceType", dialogUI->comboBoxOpenCLDevice->currentText());

        settings->sync();

        // these settings have immediate effect
        glWidget->SetYUp(settings->value("YUp", 0).toBool());
        setDefaultLights(settings->value("YUp", 0).toBool());
        glWidget->Set3DCursorRadius(settings->value("CursorRadius", 0).toDouble());
        glWidget->Set3DCursorNudge(settings->value("Nudge", 0).toDouble());
        glWidget->SetCameraFrontClip(settings->value("CameraFrontClip", 0).toDouble());
        glWidget->SetCameraBackClip(settings->value("CameraBackClip", 0).toDouble());

        updateGL();

    }

}

void MainWindow::menuOutputs()
{
    if (gSimulation == 0) return;

    DialogOutputSelect dialogOutputSelect(this);
    Ui::DialogOutputSelect *dialogUI = dialogOutputSelect.ui();

    int status = dialogOutputSelect.exec();
    QListWidgetItem *item;
    int i;
    bool dump;

    if (status == QDialog::Accepted) // write the new settings
    {
        for (i = 0; i < dialogUI->listWidgetBody->count(); i++)
        {
            item = dialogUI->listWidgetBody->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetBodyList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetMuscle->count(); i++)
        {
            item = dialogUI->listWidgetMuscle->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetMuscleList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetGeom->count(); i++)
        {
            item = dialogUI->listWidgetGeom->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetGeomList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetJoint->count(); i++)
        {
            item = dialogUI->listWidgetJoint->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetJointList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetDriver->count(); i++)
        {
            item = dialogUI->listWidgetDriver->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetDriverList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetDataTarget->count(); i++)
        {
            item = dialogUI->listWidgetDataTarget->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetDataTargetList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetReporter->count(); i++)
        {
            item = dialogUI->listWidgetReporter->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetReporterList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

    }
}

void MainWindow::menuInterface()
{
    DialogInterface dialogInterface(this);
    Ui::DialogInterface *dialogUI = dialogInterface.ui();

    // handle default colours
    if (settings->contains("EnvironmentColour") == false) settings->setValue("EnvironmentColour", QColor(0,255,255));
    if (settings->contains("BodyColour") == false) settings->setValue("BodyColour", QColor(0,255,0));
    if (settings->contains("JointColour") == false) settings->setValue("JointColour", QColor(0,0,255));
    if (settings->contains("GeomColour") == false) settings->setValue("GeomColour", QColor(255,255,0));
    if (settings->contains("GeomForceColour") == false) settings->setValue("GeomForceColour", QColor(255,255,0));
    if (settings->contains("StrapColour") == false) settings->setValue("StrapColour", QColor(255,0,0));
    if (settings->contains("StrapForceColour") == false) settings->setValue("StrapForceColour", QColor(255,0,0));
    if (settings->contains("StrapCylinderColour") == false) settings->setValue("StrapCylinderColour", QColor(255,0,255));
    if (settings->contains("ReporterColour") == false) settings->setValue("ReporterColour", QColor(255,128,0));


    dialogInterface.m_EnvironmentColour = settings->value("EnvironmentColour").value<QColor>();
    dialogInterface.m_BodyColour = settings->value("BodyColour").value<QColor>();
    dialogInterface.m_JointColour = settings->value("JointColour").value<QColor>();
    dialogInterface.m_GeomColour = settings->value("GeomColour").value<QColor>();
    dialogInterface.m_GeomForceColour = settings->value("GeomForceColour").value<QColor>();
    dialogInterface.m_StrapColour = settings->value("StrapColour").value<QColor>();
    dialogInterface.m_StrapForceColour = settings->value("StrapForceColour").value<QColor>();
    dialogInterface.m_StrapCylinderColour = settings->value("StrapCylinderColour").value<QColor>();
    dialogInterface.m_ReporterColour = settings->value("ReporterColour").value<QColor>();

    dialogUI->doubleSpinBoxEnvironmentX->setValue(settings->value("EnvironmentX", 1).toDouble());
    dialogUI->doubleSpinBoxEnvironmentY->setValue(settings->value("EnvironmentY", 1).toDouble());
    dialogUI->doubleSpinBoxEnvironmentZ->setValue(settings->value("EnvironmentZ", 1).toDouble());
    dialogUI->doubleSpinBoxBodyX->setValue(settings->value("BodyX", 0.1).toDouble());
    dialogUI->doubleSpinBoxBodyY->setValue(settings->value("BodyY", 0.1).toDouble());
    dialogUI->doubleSpinBoxBodyZ->setValue(settings->value("BodyZ", 0.1).toDouble());
    dialogUI->doubleSpinBoxJointX->setValue(settings->value("JointX", 0.05).toDouble());
    dialogUI->doubleSpinBoxJointY->setValue(settings->value("JointY", 0.05).toDouble());
    dialogUI->doubleSpinBoxJointZ->setValue(settings->value("JointZ", 0.05).toDouble());
    dialogUI->doubleSpinBoxGeomX->setValue(settings->value("GeomX", 0.05).toDouble());
    dialogUI->doubleSpinBoxGeomY->setValue(settings->value("GeomY", 0.05).toDouble());
    dialogUI->doubleSpinBoxGeomZ->setValue(settings->value("GeomZ", 0.05).toDouble());
    dialogUI->doubleSpinBoxGeomForceScale->setValue(settings->value("GeomForceScale", 0.00001).toDouble());
    dialogUI->doubleSpinBoxGeomForceRadius->setValue(settings->value("GeomForceRadius", 0.01).toDouble());
    dialogUI->doubleSpinBoxStrapRadius->setValue(settings->value("StrapRadius", 0.005).toDouble());
    dialogUI->doubleSpinBoxStrapForceRadius->setValue(settings->value("StrapForceRadius", 0.01).toDouble());
    dialogUI->doubleSpinBoxStrapForceScale->setValue(settings->value("StrapForceScale", 0.00001).toDouble());
    dialogUI->doubleSpinBoxStrapCylinderLength->setValue(settings->value("StrapCylinderLength", 0.04).toDouble());
    dialogUI->doubleSpinBoxReporterX->setValue(settings->value("ReporterX", 0.005).toDouble());
    dialogUI->doubleSpinBoxReporterY->setValue(settings->value("ReporterY", 0.005).toDouble());
    dialogUI->doubleSpinBoxReporterZ->setValue(settings->value("ReporterZ", 0.005).toDouble());

    dialogInterface.setColours();

    int status = dialogInterface.exec();

    if (status == QDialog::Accepted) // write the new settings
    {
        settings->setValue("EnvironmentColour", dialogInterface.m_EnvironmentColour);
        settings->setValue("BodyColour", dialogInterface.m_BodyColour);
        settings->setValue("JointColour", dialogInterface.m_JointColour);
        settings->setValue("GeomColour", dialogInterface.m_GeomColour);
        settings->setValue("GeomForceColour", dialogInterface.m_GeomForceColour);
        settings->setValue("StrapColour", dialogInterface.m_StrapColour);
        settings->setValue("StrapForceColour", dialogInterface.m_StrapForceColour);
        settings->setValue("StrapCylinderColour", dialogInterface.m_StrapCylinderColour);
        settings->setValue("ReporterColour", dialogInterface.m_ReporterColour);

        settings->setValue("EnvironmentX", dialogUI->doubleSpinBoxEnvironmentX->value());
        settings->setValue("EnvironmentY", dialogUI->doubleSpinBoxEnvironmentY->value());
        settings->setValue("EnvironmentZ", dialogUI->doubleSpinBoxEnvironmentZ->value());
        settings->setValue("BodyX", dialogUI->doubleSpinBoxBodyX->value());
        settings->setValue("BodyY", dialogUI->doubleSpinBoxBodyY->value());
        settings->setValue("BodyZ", dialogUI->doubleSpinBoxBodyZ->value());
        settings->setValue("JointX", dialogUI->doubleSpinBoxJointX->value());
        settings->setValue("JointY", dialogUI->doubleSpinBoxJointY->value());
        settings->setValue("JointZ", dialogUI->doubleSpinBoxJointZ->value());
        settings->setValue("GeomX", dialogUI->doubleSpinBoxGeomX->value());
        settings->setValue("GeomY", dialogUI->doubleSpinBoxGeomY->value());
        settings->setValue("GeomZ", dialogUI->doubleSpinBoxGeomZ->value());
        settings->setValue("GeomForceScale", dialogUI->doubleSpinBoxGeomForceScale->value());
        settings->setValue("GeomForceRadius", dialogUI->doubleSpinBoxGeomForceRadius->value());
        settings->setValue("StrapRadius", dialogUI->doubleSpinBoxStrapRadius->value());
        settings->setValue("StrapForceRadius", dialogUI->doubleSpinBoxStrapForceRadius->value());
        settings->setValue("StrapForceScale", dialogUI->doubleSpinBoxStrapForceScale->value());
        settings->setValue("StrapCylinderLength", dialogUI->doubleSpinBoxStrapCylinderLength->value());
        settings->setValue("ReporterX", dialogUI->doubleSpinBoxReporterX->value());
        settings->setValue("ReporterY", dialogUI->doubleSpinBoxReporterY->value());
        settings->setValue("ReporterZ", dialogUI->doubleSpinBoxReporterZ->value());

        settings->sync();

        // currently these values only take effect at startup so no need to update
        // updateGL();
    }

}


void MainWindow::ReadSettings()
{
    glWidget->SetCameraDistance(settings->value("CameraDistance", 50).toDouble());
    glWidget->SetCameraFoV(settings->value("CameraFoV", 5).toDouble());
    glWidget->SetCameraCOIX(settings->value("CameraCOIX", 0).toDouble());
    glWidget->SetCameraCOIY(settings->value("CameraCOIY", 0).toDouble());
    glWidget->SetCameraCOIZ(settings->value("CameraCOIZ", 0).toDouble());
    glWidget->SetCameraFrontClip(settings->value("CameraFrontClip", 1).toDouble());
    glWidget->SetCameraBackClip(settings->value("CameraBackClip", 100).toDouble());

    trackingFlag = settings->value("DisplayTracking", 0).toBool();
    glWidget->SetOverlay(settings->value("DisplayOverlay", 0).toBool());
    gDrawContactForces = settings->value("DisplayContactForces", 0).toBool();
    gDrawMuscleForces = settings->value("DisplayMuscleForces", 0).toBool();
    glWidget->SetWhiteBackground(settings->value("DisplayWhiteBackground", 0).toBool());
    glWidget->SetDisplayFramerate(settings->value("DisplayFramerate", 0).toBool());
    g_ActivationDisplay = settings->value("DisplayActivation", 0).toBool();

    movieFolder = settings->value("MoviePath", QString("movie")).toString();
    skip = settings->value("MovieSkip", 1).toInt();
    glWidget->SetMovieFormat(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()));

    glWidget->SetYUp(settings->value("YUp", 0).toBool());

    glWidget->Set3DCursorRadius(settings->value("CursorRadius", 0).toDouble());
    glWidget->Set3DCursorNudge(settings->value("Nudge", 0).toDouble());

#ifdef USE_OPENCL
    g_UseOpenCL = settings->value("OpenCLUseOpenCL", 0).toInt();
#else
    g_UseOpenCL = 0;
#endif

    ui->doubleSpinBoxDistance->setValue(settings->value("CameraDistance", 50).toDouble());
    ui->doubleSpinBoxFoV->setValue(settings->value("CameraFoV", 5).toDouble());
    ui->doubleSpinBoxCOIX->setValue(settings->value("CameraCOIX", 0).toDouble());
    ui->doubleSpinBoxCOIY->setValue(settings->value("CameraCOIY", 0).toDouble());
    ui->doubleSpinBoxCOIZ->setValue(settings->value("CameraCOIZ", 0).toDouble());

    ui->checkBoxTracking->setChecked(settings->value("DisplayTracking", 0).toBool());
    ui->checkBoxOverlay->setChecked(settings->value("DisplayOverlay", 0).toBool());
    ui->checkBoxContactForce->setChecked(settings->value("DisplayContactForces", 0).toBool());
    ui->checkBoxMuscleForce->setChecked(settings->value("DisplayMuscleForces", 0).toBool());
    ui->checkBoxWhiteBackground->setChecked(settings->value("DisplayWhiteBackground", 0).toBool());
    ui->checkBoxFramerate->setChecked(settings->value("DisplayFramerate", 0).toBool());
    ui->checkBoxActivationColours->setChecked(settings->value("DisplayActivation", 0).toBool());
    ui->checkBoxOpenCL->setChecked(settings->value("OpenCLUseOpenCL", 0).toBool());

    ui->lineEditMovieFolder->setText(settings->value("MoviePath", QString("movie")).toString());
    ui->spinBoxSkip->setValue(settings->value("MovieSkip", 1).toInt());
    ui->radioButtonPPM->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::PPM);
    ui->radioButtonTIFF->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::TIFF);
    ui->radioButtonPOVRay->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::POVRay);
    ui->radioButtonOBJ->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::OBJ);
}

void MainWindow::menuRequestMuscle(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetMuscle->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetMuscle->count(); i++)
        {
            item = ui->listWidgetMuscle->item(i);
            item->setCheckState(state);
            (*gSimulation->GetMuscleList())[std::string(item->text().toAscii())]->SetAllVisible(visible);
        }
        updateGL();
    }
}

void MainWindow::menuRequestBody(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetBody->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetBody->count(); i++)
        {
            item = ui->listWidgetBody->item(i);
            item->setCheckState(state);
            (*gSimulation->GetBodyList())[std::string(item->text().toAscii())]->SetVisible(visible);
        }
        updateGL();
    }
}

void MainWindow::menuRequestJoint(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetJoint->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetJoint->count(); i++)
        {
            item = ui->listWidgetJoint->item(i);
            item->setCheckState(state);
            (*gSimulation->GetJointList())[std::string(item->text().toAscii())]->SetVisible(visible);
        }
        updateGL();
    }
}

void MainWindow::menuRequestGeom(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetGeom->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetGeom->count(); i++)
        {
            item = ui->listWidgetGeom->item(i);
            item->setCheckState(state);
            (*gSimulation->GetGeomList())[std::string(item->text().toAscii())]->SetVisible(visible);
        }
        updateGL();
    }
}

void MainWindow::menuDefaultView()
{
    glWidget->SetCameraDistance(settings->value("CameraDistance", 50).toDouble());
    glWidget->SetCameraFoV(settings->value("CameraFoV", 5).toDouble());
    glWidget->SetCameraCOIX(settings->value("CameraCOIX", 0).toDouble());
    glWidget->SetCameraCOIY(settings->value("CameraCOIY", 0).toDouble());
    glWidget->SetCameraCOIZ(settings->value("CameraCOIZ", 0).toDouble());

    glWidget->SetCameraVec(settings->value("CameraVecX", 0).toDouble(), settings->value("CameraVecY", 1).toDouble(), settings->value("CameraVecZ", 0).toDouble());
    glWidget->SetCameraUp(settings->value("CameraUpX", 0).toDouble(), settings->value("CameraUpY", 0).toDouble(), settings->value("CameraUpZ", 1).toDouble());

    ui->doubleSpinBoxDistance->setValue(settings->value("CameraDistance", 50).toDouble());
    ui->doubleSpinBoxFoV->setValue(settings->value("CameraFoV", 5).toDouble());
    ui->doubleSpinBoxCOIX->setValue(settings->value("CameraCOIX", 0).toDouble());
    ui->doubleSpinBoxCOIY->setValue(settings->value("CameraCOIY", 0).toDouble());
    ui->doubleSpinBoxCOIZ->setValue(settings->value("CameraCOIZ", 0).toDouble());

    updateGL();
}

void MainWindow::menuSaveDefaultView()
{
    settings->setValue("CameraDistance", ui->doubleSpinBoxDistance->value());
    settings->setValue("CameraFoV", ui->doubleSpinBoxFoV->value());
    settings->setValue("CameraCOIX", ui->doubleSpinBoxCOIX->value());
    settings->setValue("CameraCOIY", ui->doubleSpinBoxCOIY->value());
    settings->setValue("CameraCOIZ", ui->doubleSpinBoxCOIZ->value());

    GLfloat x, y, z;
    glWidget->GetCameraVec(&x, &y, &z);
    settings->setValue("CameraVecX", double(x));
    settings->setValue("CameraVecY", double(y));
    settings->setValue("CameraVecZ", double(z));
    glWidget->GetCameraUp(&x, &y, &z);
    settings->setValue("CameraUpX", double(x));
    settings->setValue("CameraUpY", double(y));
    settings->setValue("CameraUpZ", double(z));
}

void MainWindow::setStatusString(QString s)
{
    statusBar()->showMessage(s);
}

void MainWindow::setUICOI(double x, double y, double z)
{
    ui->doubleSpinBoxCOIX->setValue(x);
    ui->doubleSpinBoxCOIY->setValue(y);
    ui->doubleSpinBoxCOIZ->setValue(z);
}

void MainWindow::setUIFoV(double v)
{
    ui->doubleSpinBoxFoV->setValue(v);
}

void MainWindow::setDefaultLights(bool yUp)
{
    // lights
    Light l0, l1, l2, l3;

    // Green front right
    l0.SetAmbient (0.2, 0.2, 0.2, 1);
    l0.SetDiffuse (0.3, 0.6, 0.3, 1);
    l0.SetSpecular(0.4, 0.6, 0.4, 1);
    if (yUp) l0.SetPosition(0.5, 1, 1, 0);
    else l0.SetPosition(0.5, -1, 1, 0);

    // Blue front left
    l1.SetAmbient (0.2, 0.2, 0.2, 1);
    l1.SetDiffuse (0.3, 0.3, 0.6, 1);
    l1.SetSpecular(0.4, 0.4, 0.6, 1);
    if (yUp) l1.SetPosition(0.5, 1, -1, 0);
    else l1.SetPosition(0.5, 1, 1, 0);

    // Red back centre
    l2.SetAmbient (0.2, 0.2, 0.2, 1);
    l2.SetDiffuse (0.6, 0.3, 0.3, 1);
    l2.SetSpecular(0.6, 0.4, 0.4, 1);
    if (yUp) l2.SetPosition(-1, 0, 0, 0);
    else l2.SetPosition(-1, 0, 0, 0);

    // White underneath
    l3.SetAmbient (0.1, 0.1, 0.1, 1);
    l3.SetDiffuse (0.2, 0.2, 0.2, 1);
    l3.SetSpecular(0.3, 0.3, 0.3, 1);
    if (yUp) l3.SetPosition(0, -1, 0, 0);
    else l3.SetPosition(0, 0, -1, 0);

    glWidget->ClearLights();
    glWidget->AddLight(&l0);
    glWidget->AddLight(&l1);
    glWidget->AddLight(&l2);
    glWidget->AddLight(&l3);
}

void MainWindow::menu640x480()
{
    resizeAndCentre(640, 480);
}

void MainWindow::menu800x600()
{
    resizeAndCentre(800, 600);
}

void MainWindow::menu1280x720()
{
    resizeAndCentre(1280, 720);
}

void MainWindow::menu1920x1080()
{
    resizeAndCentre(1920, 1080);
}

void MainWindow::resizeAndCentre(int w, int h)
{
    QDesktopWidget *desktop = qApp->desktop();
    QRect available = desktop->availableGeometry(-1);

    // Need to find how big the central widget is compared to the window
    int heightDiff = height() - glWidget->height();
    int widthDiff = width() - glWidget->width();
    int newWidth = w + widthDiff;
    int newHeight = h + heightDiff;

    // centre window
    int topLeftX = available.left() + (available.width() / 2) - (newWidth / 2);
    int topLeftY = available.top() + (available.height() / 2) - (newHeight / 2);
    // but don't start off screen
    if (topLeftX < available.left()) topLeftX = available.left();
    if (topLeftY < available.top()) topLeftY = available.top();

    move(topLeftX, topLeftY);
    resize(newWidth, newHeight);
    updateGL();
}

void MainWindow::menuStartQuicktimeSave()
{
    QFileInfo info = settings->value("LastFileOpened", QString("")).toString();

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save output as Quicktime file"), info.absolutePath(), tr("Quicktime Files (*.mov)"));

    if (fileName.isNull() == false)
    {
        qtMovie = initialiseMovieFile(fileName.toUtf8(), settings->value("QuicktimeFramerate", 24).toInt());
        if (qtMovie)
        {
            ui->actionStart_Quicktime_Save->setEnabled(false);
            ui->actionStop_Quicktime_Save->setEnabled(true);
            glWidget->SetQTMovie(qtMovie);
        }
    }
}

void MainWindow::menuStopQuicktimeSave()
{
    ui->actionStart_Quicktime_Save->setEnabled(true);
    ui->actionStop_Quicktime_Save->setEnabled(false);
    if (qtMovie) closeMovieFile(qtMovie);
    qtMovie = 0;
    glWidget->SetQTMovie(qtMovie);
}

void MainWindow::log(const char *text)
{
    ui->textEditLog->append(text);
}

void MainWindow::copy()
{
    QWidget* focused = QApplication::focusWidget();
    if( focused != 0 )
    {
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyPress, Qt::Key_C, Qt::ControlModifier ));
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyRelease, Qt::Key_C, Qt::ControlModifier ));
    }

}

void MainWindow::cut()
{
    QWidget* focused = QApplication::focusWidget();
    if( focused != 0 )
    {
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyPress, Qt::Key_X, Qt::ControlModifier ));
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyRelease, Qt::Key_X, Qt::ControlModifier ));
    }

}

void MainWindow::paste()
{
    QWidget* focused = QApplication::focusWidget();
    if( focused != 0 )
    {
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyPress, Qt::Key_V, Qt::ControlModifier ));
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyRelease, Qt::Key_V, Qt::ControlModifier ));
    }

}

void MainWindow::selectAll()
{
    QWidget* focused = QApplication::focusWidget();
    if( focused != 0 )
    {
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyPress, Qt::Key_A, Qt::ControlModifier ));
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyRelease, Qt::Key_A, Qt::ControlModifier ));
    }

}

void MainWindow::updateGL()
{
    if (allowGLUpdateFlag) glWidget->updateGL();
}
