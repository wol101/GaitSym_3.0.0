#ifndef DIALOGINTERFACE_H
#define DIALOGINTERFACE_H

#include <QDialog>

namespace Ui {
    class DialogInterface;
}

class DialogInterface : public QDialog
{
    Q_OBJECT

public:
    explicit DialogInterface(QWidget *parent = 0);
    ~DialogInterface();

    Ui::DialogInterface *ui() { return m_ui; }

    static QColor getIdealTextColour(const QColor& rBackgroundColour);
    static QColor getAlphaColourHint(const QColor& colour);

    void setColours();

    QColor m_EnvironmentColour;
    QColor m_BodyColour;
    QColor m_JointColour;
    QColor m_GeomColour;
    QColor m_GeomForceColour;
    QColor m_StrapColour;
    QColor m_StrapForceColour;
    QColor m_StrapCylinderColour;
    QColor m_ReporterColour;


public slots:

    void colourEnvironment();
    void colourBody();
    void colourJoint();
    void colourGeom();
    void colourGeomForce();
    void colourStrap();
    void colourStrapForce();
    void colourStrapCylinder();
    void colourReporter();

protected:
    void changeEvent(QEvent *e);


private:
    Ui::DialogInterface *m_ui;
};

#endif // DIALOGINTERFACE_H
