#ifndef RVIZTOPICS_HPP
#define RVIZTOPICS_HPP

#include <QWidget>
#include <QDebug>
#include <QTreeWidgetItem>
#include <QCheckBox>
namespace Ui
{
class RvizTopics;
}
class RvizTopics : public QWidget
{
    Q_OBJECT

public:
    explicit RvizTopics(QWidget * parent = nullptr);
    QTreeWidgetItem * choose;
    ~RvizTopics();
signals:
    void TopicChoose(QTreeWidgetItem * choose, QString name);
private slots:
    void onPushButtonCancelClicked();

private slots:
    void onPushButtonOkClicked();

private slots:
    void slotCurritemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);

private:
    Ui::RvizTopics * ui;
    void initUi();
    QCheckBox * checkbox;
    QTreeWidgetItem * Navigate=nullptr;
    QTreeWidgetItem * Build_Map=nullptr;
    QTreeWidgetItem * Axes=nullptr;
    QTreeWidgetItem * Camera=nullptr;
    QTreeWidgetItem * DepthCloud=nullptr;
    QTreeWidgetItem * Effort=nullptr;
    QTreeWidgetItem * FluidPressure=nullptr;
    QTreeWidgetItem * Grid=nullptr;
    QTreeWidgetItem * GridCells=nullptr;
    QTreeWidgetItem * Group=nullptr;
    QTreeWidgetItem * Illuminance=nullptr;
    QTreeWidgetItem * Image=nullptr;
    QTreeWidgetItem * InteractiveMarkers=nullptr;
    QTreeWidgetItem * LaserScan=nullptr;
    QTreeWidgetItem * Map=nullptr;
    QTreeWidgetItem * Marker=nullptr;
    QTreeWidgetItem * MarkerArray=nullptr;
    QTreeWidgetItem * Odometry=nullptr;
    QTreeWidgetItem * Path=nullptr;
    QTreeWidgetItem * PointCloud=nullptr;
    QTreeWidgetItem * PointCloud2=nullptr;
    QTreeWidgetItem * PointStamped=nullptr;
    QTreeWidgetItem * Polygon=nullptr;
    QTreeWidgetItem * Pose=nullptr;
    QTreeWidgetItem * PoseArray=nullptr;
    QTreeWidgetItem * PoseWithCovariance=nullptr;
    QTreeWidgetItem * Range=nullptr;
    QTreeWidgetItem * RelativeHumidity=nullptr;
    QTreeWidgetItem * RobotModel=nullptr;
    QTreeWidgetItem * TF=nullptr;
    QTreeWidgetItem * Temperature=nullptr;
    QTreeWidgetItem * WrenchStamped=nullptr;
    QTreeWidgetItem * Imu=nullptr;
};

#endif // RVIZTOPICS_HPP
