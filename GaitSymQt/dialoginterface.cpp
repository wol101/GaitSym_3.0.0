#include "dialoginterface.h"
#include "ui_dialoginterface.h"

#include <QColorDialog>

DialogInterface::DialogInterface(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::DialogInterface)
{
    m_ui->setupUi(this);
}

DialogInterface::~DialogInterface()
{
    delete m_ui;
}

void DialogInterface::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void DialogInterface::setColours()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    m_ui->pushButtonEnvironmentColour->setStyleSheet(COLOUR_STYLE.arg(m_EnvironmentColour.name()).arg(getIdealTextColour(m_EnvironmentColour).name()).arg(getAlphaColourHint(m_EnvironmentColour).name()));
    m_ui->pushButtonBodyColour->setStyleSheet(COLOUR_STYLE.arg(m_BodyColour.name()).arg(getIdealTextColour(m_BodyColour).name()).arg(getAlphaColourHint(m_BodyColour).name()));
    m_ui->pushButtonJointColour->setStyleSheet(COLOUR_STYLE.arg(m_JointColour.name()).arg(getIdealTextColour(m_JointColour).name()).arg(getAlphaColourHint(m_JointColour).name()));
    m_ui->pushButtonGeomColour->setStyleSheet(COLOUR_STYLE.arg(m_GeomColour.name()).arg(getIdealTextColour(m_GeomColour).name()).arg(getAlphaColourHint(m_GeomColour).name()));
    m_ui->pushButtonGeomForceColour->setStyleSheet(COLOUR_STYLE.arg(m_GeomForceColour.name()).arg(getIdealTextColour(m_GeomForceColour).name()).arg(getAlphaColourHint(m_GeomForceColour).name()));
    m_ui->pushButtonStrapColour->setStyleSheet(COLOUR_STYLE.arg(m_StrapColour.name()).arg(getIdealTextColour(m_StrapColour).name()).arg(getAlphaColourHint(m_StrapColour).name()));
    m_ui->pushButtonStrapForceColour->setStyleSheet(COLOUR_STYLE.arg(m_StrapForceColour.name()).arg(getIdealTextColour(m_StrapForceColour).name()).arg(getAlphaColourHint(m_StrapForceColour).name()));
    m_ui->pushButtonStrapCylinderColour->setStyleSheet(COLOUR_STYLE.arg(m_StrapCylinderColour.name()).arg(getIdealTextColour(m_StrapCylinderColour).name()).arg(getAlphaColourHint(m_StrapCylinderColour).name()));
    m_ui->pushButtonReporterColour->setStyleSheet(COLOUR_STYLE.arg(m_ReporterColour.name()).arg(getIdealTextColour(m_ReporterColour).name()).arg(getAlphaColourHint(m_ReporterColour).name()));

}

void DialogInterface::colourEnvironment()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_EnvironmentColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonEnvironmentColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_EnvironmentColour = colour;
    }
}


void DialogInterface::colourBody()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_BodyColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonBodyColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_BodyColour = colour;
    }
}


void DialogInterface::colourJoint()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_JointColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonJointColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_JointColour = colour;
    }
}


void DialogInterface::colourGeom()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_GeomColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonGeomColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_GeomColour = colour;
    }
}


void DialogInterface::colourGeomForce()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_GeomForceColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonGeomForceColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_GeomForceColour = colour;
    }
}


void DialogInterface::colourStrap()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_StrapColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonStrapColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_StrapColour = colour;
    }
}


void DialogInterface::colourStrapForce()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_StrapForceColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonStrapForceColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_StrapForceColour = colour;
    }
}


void DialogInterface::colourStrapCylinder()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_StrapCylinderColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonStrapCylinderColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_StrapCylinderColour = colour;
    }
}

void DialogInterface::colourReporter()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_ReporterColour, this, "Select Color", QColorDialog::ShowAlphaChannel);
    if (colour.isValid())
    {
        m_ui->pushButtonReporterColour->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_ReporterColour = colour;
    }
}

// return an ideal label colour, based on the given background colour.
// Based on http://www.codeproject.com/cs/media/IdealTextColor.asp
QColor DialogInterface::getIdealTextColour(const QColor& rBackgroundColour)
{
    const int THRESHOLD = 105;
    int BackgroundDelta = (rBackgroundColour.red() * 0.299) + (rBackgroundColour.green() * 0.587) + (rBackgroundColour.blue() * 0.114);
    return QColor((255- BackgroundDelta < THRESHOLD) ? Qt::black : Qt::white);
}

QColor DialogInterface::getAlphaColourHint(const QColor& colour)
{
    // (source × Blend.SourceAlpha) + (background × Blend.InvSourceAlpha)
    QColor background;
    background.setRgbF(1.0, 1.0, 1.0);
    QColor hint;
    hint.setRedF((colour.redF() * colour.alphaF()) + (background.redF() * (1 - colour.alphaF())));
    hint.setGreenF((colour.greenF() * colour.alphaF()) + (background.greenF() * (1 - colour.alphaF())));
    hint.setBlueF((colour.blueF() * colour.alphaF()) + (background.blueF() * (1 - colour.alphaF())));
    return hint;
}

