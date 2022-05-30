#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QProcess>
#include <QSpinBox>
#include <QVariant>
#include <iostream>
#include <QSettings>
#include <QComboBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMainWindow>
#include <QMessageBox>
#include <QInputDialog>
#include <QTreeWidgetItem>
#include <QStandardItemModel>

#include "qtnode.hpp"
#include "qtrviz.hpp"
#include "rviztopics.hpp"
#include "ui_mainwindow.h"

namespace qt_map_rviz_ros
{
  class MainWindow : public QMainWindow
  {
    Q_OBJECT

    public:
      explicit MainWindow(int argc, char ** argv, QWidget *parent = nullptr);
      ~MainWindow();
      void readSettings();
      void writeSettings();
      void initRviz();
      void initUi();

    public slots:
      void onCheckboxEnvironmet(int state);
      void slotRosShutdown();
      void rvizGetModel(QAbstractItemModel * model);
      void slotChooseTopic(QTreeWidgetItem *choose, QString name);
      void onSliderAngValueChanged(int value);
      void onSliderLinearValueChanged(int value);
      void slotCmdControl();
      void slotSet2DGoal();
      void slotSet2DPos();
      void slotSetSelect();
      void slotReturn();
      void slotMoveCameraBtn();
      

  private slots:
      void onTreeViewRvizDispTreeClicked(const QModelIndex & index);
      void onPushButtonAddTopicClicked();
      void onPushButtonRemoveTopicClicked();
      void onPushButtonRenameTopicClicked();
      void on_button_disconnect_clicked();
      void onBtnRvizReadDispSetClicked();
      void onBtnRvizSaveDispSetClicked();
      void on_button_connect_clicked();

    private:
      Ui::MainWindow *ui;
      void initData();
      void connections();
      void inform(QString);
      bool askInform(QString);
      QString judgeDispNewName(QString name);
      QString getUserName();
      QtNode qtNode;
      QProcess * baseCmd = nullptr;
      QtRviz * mapRviz = nullptr;
      RvizTopics * rvizTopic = nullptr;
      QStandardItemModel * treeViewRvizModel = nullptr;
      QMap < QWidget * , QTreeWidgetItem * > widgetToParentItemMap;
      QMap < QString, QTreeWidgetItem * > treeRvizStatus;
      QMap < QTreeWidgetItem *, QMap < QString, QString >> treeRvizValues;
      QAbstractItemModel * mModelRvizDisp;
      QMap < QString, QString > mMapRvizDisp;
      QString mRvizDispChooseName;
};
}
#endif // MAINWINDOW_H
