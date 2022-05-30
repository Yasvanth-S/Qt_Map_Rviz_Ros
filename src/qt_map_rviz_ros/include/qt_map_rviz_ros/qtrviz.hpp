#ifndef QTRVIZ_HPP
#define QTRVIZ_HPP

#include <QDebug>
#include <QThread>
#include <QException>
#include <QVBoxLayout>
#include <QAbstractItemModel>

#include <rviz/tool.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/tool_manager.h>
#include <rviz/display_group.h>
#include <rviz/yaml_config_reader.h>
#include <rviz/yaml_config_writer.h>
#include <rviz/visualization_manager.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz/properties/property_tree_model.h>

#define QT_RVIZ_DISPLAY_TF                      "rviz/TF"
#define QT_RVIZ_DISPLAY_MAP                     "rviz/Map"
#define QT_RVIZ_DISPLAY_AXES                    "rviz/Axes"
#define QT_RVIZ_DISPLAY_GRID                    "rviz/Grid"
#define QT_RVIZ_DISPLAY_PATH                    "rviz/Path"
#define QT_RVIZ_DISPLAY_POSE                    "rviz/Pose"
#define QT_RVIZ_DISPLAY_GROUP                   "rviz/Group"
#define QT_RVIZ_DISPLAY_IMAGE                   "rviz/Image"
#define QT_RVIZ_DISPLAY_RANGE                   "rviz/Range"
#define QT_RVIZ_DISPLAY_CAMERA                  "rviz/Camera"
#define QT_RVIZ_DISPLAY_EFFORT                  "rviz/Effort"
#define QT_RVIZ_DISPLAY_MARKER                  "rviz/Marker"
#define QT_RVIZ_DISPLAY_POLYGON                 "rviz/Polygon"
#define QT_RVIZ_DISPLAY_ODOMETRY                "rviz/Odometry"
#define QT_RVIZ_DISPLAY_LASERSCAN               "rviz/LaserScan"
#define QT_RVIZ_DISPLAY_GRIDCELLS               "rviz/GridCells"
#define QT_RVIZ_DISPLAY_POSEARRAY               "rviz/PoseArray"
#define QT_RVIZ_DISPLAY_DEPTHCLOUD              "rviz/DepthCloud"
#define QT_RVIZ_DISPLAY_ROBOTMODEL              "rviz/RobotModel"
#define QT_RVIZ_DISPLAY_POINTCLOUD              "rviz/PointCloud"
#define QT_RVIZ_DISPLAY_POINTCLOUD2             "rviz/PointCloud2"
#define QT_RVIZ_DISPLAY_ILLUMINANCE             "rviz/Illuminance"
#define QT_RVIZ_DISPLAY_MARKERARRAY             "rviz/MarkerArray"
#define QT_RVIZ_DISPLAY_TEMPERATURE             "rviz/Temperature"
#define QT_RVIZ_DISPLAY_FLUIDPRESSURE           "rviz/FluidPressure"
#define QT_RVIZ_DISPLAY_WRENCHSTAMPED           "rviz/WrenchStamped"
#define QT_RVIZ_DISPLAY_RELATIVEHUMIDITY        "rviz/RelativeHumidity"
#define QT_RVIZ_DISPLAY_INTERACTIVEMARKER       "rviz/InteractiveMarker"
#define QT_RVIZ_DISPLAY_POSEWITHCOVARIANCE      "rviz/PoseWithCovariance"

class QtRviz:public QThread
{
  Q_OBJECT
  public:
      QtRviz(QVBoxLayout * layout, QString nodeName);
      ~QtRviz();

      void DisplayInit(QString classID, bool enabled, QMap<QString, QVariant>nameValue);
      void DisplayInit(QString classID, bool enabled, QString name, QMap<QString, QVariant>nameValue);
      void RemoveDisp(QString name);
      void RemoveDisp(QString classID, QString name);
      int RenameDisp(QString oldName, QString newName);
      void OutDispSet(QString path);
      void ReadDispSet(QString path);
      void SetGlobalOptions(QString frameName, QColor backColor, int frameRate);

      void SetPos();
      void SetGoal();
      void Select();
      void MoveCamera();
      void SendGoalTopic();
      void GetDispTreeModel();

  signals:
      void ReturnModelSignal(QAbstractItemModel * model);

  private:
      int GetDispNum(QString classID);
      int GetDispNum(QString classID, QString name);
      int GetDispNumName(QString name);

      rviz::RenderPanel * renderPanel;
      rviz::VisualizationManager * manager;
      rviz::DisplayGroup * dispGroup;
      rviz::Tool * currentTool;
      rviz::ToolManager * toolManager;
      QVBoxLayout * Layout;
      QString nodename;
      QMap<QString, QVariant>nullMap;

};

#endif // QTRVIZ_HPP
