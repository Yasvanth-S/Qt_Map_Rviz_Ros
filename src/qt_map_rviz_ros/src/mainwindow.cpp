#include "../include/qt_map_rviz_ros/mainwindow.h"

namespace qt_map_rviz_ros {

MainWindow::MainWindow(int argc, char ** argv, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  qtNode(argc,argv)
{
  setWindowIcon(QIcon("://images/icon.png"));
  ui->setupUi(this);
  initUi();
  initData();
  readSettings();
  rvizTopic = new RvizTopics();
  connect(rvizTopic, SIGNAL(TopicChoose(QTreeWidgetItem *, QString)), this, SLOT(slotChooseTopic(QTreeWidgetItem *, QString)));
  if(ui->checkbox_remember_settings->isChecked())
  {
    on_button_connect_clicked();
  }
  connections();
}

MainWindow::~MainWindow()
{
  if(baseCmd)
  {
    delete baseCmd;
    baseCmd = nullptr;
  }
  delete ui;
  writeSettings();
}

void MainWindow::initUi()
{
  ui->gridLayout_7->setEnabled(false);
  ui->tabWidget->setCurrentIndex(0);
  ui->remove_topic->setEnabled(true);
  ui->rename_topic->setEnabled(true);
  ui->button_disconnect->setEnabled(false);
  ui->move_camera_btn->setIcon(QIcon("://images/classes/MoveCamera.png"));
  ui->select_btn->setIcon(QIcon("://images/classes/Select.png"));
  ui->return_btn->setIcon(QIcon("://images/return.png"));
  ui->set_pos_btn->setIcon(QIcon("://images/classes/Pose.png"));
  ui->set_nav_btn->setIcon(QIcon("://images/classes/SetInitialPose.png"));
  ui->button_connect->setIcon(QIcon("://images/line.png"));
  QPixmap pix ("://images/classes/Displays.svg");
  ui->label_12->setPixmap(pix);
}

void MainWindow::initData()
{
  mMapRvizDisp.insert("TF",QT_RVIZ_DISPLAY_TF);
  mMapRvizDisp.insert("Map",QT_RVIZ_DISPLAY_MAP);
  mMapRvizDisp.insert("Axes",QT_RVIZ_DISPLAY_AXES);
  mMapRvizDisp.insert("Grid",QT_RVIZ_DISPLAY_GRID);
  mMapRvizDisp.insert("Path",QT_RVIZ_DISPLAY_PATH);
  mMapRvizDisp.insert("Pose",QT_RVIZ_DISPLAY_POSE);
  mMapRvizDisp.insert("Group",QT_RVIZ_DISPLAY_GROUP);
  mMapRvizDisp.insert("Image",QT_RVIZ_DISPLAY_IMAGE);
  mMapRvizDisp.insert("Range",QT_RVIZ_DISPLAY_RANGE);
  mMapRvizDisp.insert("Camera",QT_RVIZ_DISPLAY_CAMERA);
  mMapRvizDisp.insert("Effort",QT_RVIZ_DISPLAY_EFFORT);
  mMapRvizDisp.insert("Marker",QT_RVIZ_DISPLAY_MARKER);
  mMapRvizDisp.insert("Polygon",QT_RVIZ_DISPLAY_POLYGON);
  mMapRvizDisp.insert("Odometry",QT_RVIZ_DISPLAY_ODOMETRY);
  mMapRvizDisp.insert("GridCells",QT_RVIZ_DISPLAY_GRIDCELLS);
  mMapRvizDisp.insert("LaserScan",QT_RVIZ_DISPLAY_LASERSCAN);
  mMapRvizDisp.insert("PoseArray",QT_RVIZ_DISPLAY_POSEARRAY);
  mMapRvizDisp.insert("DepthCloud",QT_RVIZ_DISPLAY_DEPTHCLOUD);
  mMapRvizDisp.insert("PointCloud",QT_RVIZ_DISPLAY_POINTCLOUD);
  mMapRvizDisp.insert("RobotModel",QT_RVIZ_DISPLAY_ROBOTMODEL);
  mMapRvizDisp.insert("Illuminance",QT_RVIZ_DISPLAY_ILLUMINANCE);
  mMapRvizDisp.insert("PointCloud2",QT_RVIZ_DISPLAY_POINTCLOUD2);
  mMapRvizDisp.insert("MarkerArray",QT_RVIZ_DISPLAY_MARKERARRAY);
  mMapRvizDisp.insert("Temperature",QT_RVIZ_DISPLAY_TEMPERATURE);
  mMapRvizDisp.insert("FluidPressure",QT_RVIZ_DISPLAY_FLUIDPRESSURE);
  mMapRvizDisp.insert("WrenchStamped",QT_RVIZ_DISPLAY_WRENCHSTAMPED);
  mMapRvizDisp.insert("RelativeHumidity",QT_RVIZ_DISPLAY_RELATIVEHUMIDITY);
  mMapRvizDisp.insert("InterActiveMarker",QT_RVIZ_DISPLAY_INTERACTIVEMARKER);
  mMapRvizDisp.insert("PoseWithCovariance",QT_RVIZ_DISPLAY_POSEWITHCOVARIANCE);
}

void MainWindow::initRviz()
{
  ui->rviz_map->hide();
  mapRviz = new QtRviz(ui->verticalLayout_5,"qtrviz");
  connect(mapRviz, &QtRviz::ReturnModelSignal, this, &MainWindow::rvizGetModel);
  mapRviz->GetDispTreeModel();
  QMap<QString, QVariant>nameValue;
  nameValue.insert("Line Style", "Billboards");
  nameValue.insert("Color", QColor(160,160,160));
  nameValue.insert("Plane Cell Count", 10);
  mapRviz->DisplayInit(QT_RVIZ_DISPLAY_GRID, true, "Grid", nameValue);
  ui->add_topic->setEnabled(true);
  ui->rviz_import->setEnabled(true);
  ui->rviz_export->setEnabled(true);
}

void MainWindow::rvizGetModel(QAbstractItemModel * model)
{
  mModelRvizDisp = model;
  ui->rviz_treeview->setModel(model);
}

void MainWindow::connections()
{
  QObject::connect(&qtNode, SIGNAL(rosShutdown()),this,SLOT(slotRosShutdown()));
  connect(ui->horizontalSlider_ang,SIGNAL(valueChanged(int)),this,SLOT(onSliderAngValueChanged(int)));
  connect(ui->horizontalSlider_lin,SIGNAL(valueChanged(int)),this,SLOT(onSliderLinearValueChanged(int)));
  connect(ui->pushButton_i, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->pushButton_j, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->pushButton_l, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->pushButton_m, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->pushButton_o, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->pushButton_u, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->pushButton_k, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->pushButton_back, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->pushButton_backr, SIGNAL(clicked()), this, SLOT(slotCmdControl()));
  connect(ui->set_pos_btn, SIGNAL(clicked()), this, SLOT(slotSet2DPos()));
  connect(ui->set_nav_btn, SIGNAL(clicked()), this, SLOT(slotSet2DGoal()));
  connect(ui->select_btn, SIGNAL(clicked()), this, SLOT(slotSetSelect()));
  connect(ui->move_camera_btn, SIGNAL(clicked()), this, SLOT(slotMoveCameraBtn()));
  connect(ui->return_btn, SIGNAL(clicked()), this, SLOT(slotReturn()));
  connect(ui->add_topic, SIGNAL(clicked()), this, SLOT(onPushButtonAddTopicClicked()));
  connect(ui->remove_topic, SIGNAL(clicked()), this, SLOT(onPushButtonRemoveTopicClicked()));
  connect(ui->rename_topic, SIGNAL(clicked()), this, SLOT(onPushButtonRenameTopicClicked()));
  connect(ui->rviz_import, SIGNAL(clicked()), this, SLOT(onBtnRvizReadDispSetClicked()));
  connect(ui->rviz_export, SIGNAL(clicked()), this, SLOT(onBtnRvizSaveDispSetClicked()));
}

void MainWindow::slotSet2DPos()
{
  mapRviz->SetPos();
}

void MainWindow::slotSet2DGoal()
{
  mapRviz->SetGoal();
}

void MainWindow::slotSetSelect()
{
  mapRviz->Select();
}

void MainWindow::slotMoveCameraBtn()
{
  mapRviz->MoveCamera();
  qDebug()<<"move camera";
}

void MainWindow::slotReturn()
{
  qtNode.setGoal("map",0,0,0,0);
}

QString MainWindow::judgeDispNewName(QString name)
{
  if(mModelRvizDisp != nullptr)
  {
    bool same = true;
    while(same)
    {
      same = false;
      for(int i = 0; i < mModelRvizDisp->rowCount(); i++)
      {
        if(mModelRvizDisp->index(i, 0).data().value<QString>() == name)
        {
          if(name.indexOf("_") != -1)
          {
            int num = name.section("_", -1, -1).toInt();
            name = name.left(name.length() - name.section("_", -1, -1).length() - 1);
            if(num <= 1)
            {
              num = 2;
            }
            else
            {
              num++;
            }
            name = name + "_" + QString::number(num);
          }
          else
          {
            name = name + "_2";
          }
          same = true;
          break;
        }
      }
    }
  }
  return name;
}

void MainWindow::slotCmdControl()
{

    QPushButton * btn = qobject_cast<QPushButton*>(sender());
    char key=btn->text().toStdString()[0];
    float liner=ui->horizontalSlider_lin->value()*0.01f;
    float turn=ui->horizontalSlider_ang->value()*0.01f;
    bool is_all=ui->checkBox_use_all->isChecked();
    switch (key) {
        case 'u':
            qtNode.moveBase(is_all?'U':'u',liner,turn);
        break;
        case 'i':
            qtNode.moveBase(is_all?'I':'i',liner,turn);
        break;
        case 'o':
            qtNode.moveBase(is_all?'O':'o',liner,turn);
        break;
        case 'j':
            qtNode.moveBase(is_all?'J':'j',liner,turn);
        break;
        case 'l':
            qtNode.moveBase(is_all?'L':'l',liner,turn);
        break;
        case 'm':
            qtNode.moveBase(is_all?'M':'m',liner,turn);
        break;
        case ',':
            qtNode.moveBase(is_all?'<':',',liner,turn);
        break;
        case '.':
            qtNode.moveBase(is_all?'>':'.',liner,turn);
        break;
        case 'k':
            qtNode.moveBase(is_all?'K':'k',liner,turn);
        break;
    }
}

void MainWindow::onSliderAngValueChanged(int v)
{
  ui->label_ang->setText(QString::number(v));
}

void MainWindow::onSliderLinearValueChanged(int v)
{
  ui->label_lin->setText(QString::number(v));
}

void MainWindow::inform(QString strData)
{
  QMessageBox mR;
  mR.setWindowTitle("Hint");
  mR.setText(strData);
  mR.exec();
}

bool MainWindow::askInform(QString strData)
{
  QMessageBox mR;
  mR.setWindowTitle("Hint");
  mR.setText(strData);
  mR.addButton(tr("Confirm"), QMessageBox::AcceptRole);
  mR.addButton(tr("Cancel"), QMessageBox::AcceptRole);
  int isOk = mR.exec();
  if(isOk == 1)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void MainWindow::onCheckboxEnvironmet(int state)
{
  bool enabled;
  if(state == 0)
  {
    enabled = true;
  }
  else
  {
    enabled = false;
  }
  ui->line_edit_master->setEnabled(enabled);
  ui->line_edit_host->setEnabled(enabled);
}

void MainWindow::readSettings()
{
  QSettings settings("Qt-Rviz-Map-Ros-Package", "qt_map_rviz_ros");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString masterUrl = settings.value("master_url", QString("https://192.168.0.1:11311/")).toString();
  QString hostUrl = settings.value("host_url", QString("192.168.0.1")).toString();
  ui->line_edit_master->setText(masterUrl);
  ui->line_edit_host->setText(hostUrl);
  bool remember = settings.value("remember_settings", false).toBool();
  ui->checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environmental_variable", false).toBool();
  ui->checkbox_use_environment->setChecked(checked);
  if(checked)
  {
    ui->line_edit_master->setEnabled(false);
    ui->line_edit_host->setEnabled(false);
  }
}

void MainWindow::writeSettings()
{
  QSettings settings("Qt-Rviz-Map-Ros-Package", "qt_map_rviz_ros");
  settings.setValue("master_url",ui->line_edit_master->text());
  settings.setValue("host_url",ui->line_edit_host->text());
  settings.setValue("use_environment_variables",QVariant(ui->checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("remember_settings",QVariant(ui->checkbox_remember_settings->isChecked()));
}

void MainWindow::onPushButtonAddTopicClicked()
{
  rvizTopic->show();
}

void MainWindow::onPushButtonRemoveTopicClicked()
{
  if(ui->rviz_treeview->currentIndex().row() >= 0)
  {
    mRvizDispChooseName = ui->rviz_treeview->currentIndex().data().value<QString>();
    mapRviz->RemoveDisp(mRvizDispChooseName);
    if(ui->rviz_treeview->currentIndex().row() >= 0)
    {
      onTreeViewRvizDispTreeClicked(ui->rviz_treeview->currentIndex());
    }
    else
    {
      mRvizDispChooseName.clear();
    }
  }
  else
  {
    inform("Please Select the Display before performing this operation");
  }
}

void MainWindow::onPushButtonRenameTopicClicked()
{
    if (ui->rviz_treeview->currentIndex().row() < 0)
    {
        inform("Please select Display before performing this operation");
        return ;
    }
    QString dlgTitle = "Rename";
    QString txtlabel = "Please enter first name:";
    QString defaultInupt = ui->rviz_treeview->currentIndex().data().value<QString>();
    QLineEdit::EchoMode echoMode = QLineEdit::Normal;
    bool ok = false;
    QString newname = QInputDialog::getText(this, dlgTitle, txtlabel, echoMode, defaultInupt, &ok);
    if (ok && !newname.isEmpty())
    {
        if (newname != defaultInupt)
        {
            QString nosamename = judgeDispNewName(newname);
            mapRviz->RenameDisp(defaultInupt, nosamename);
            mRvizDispChooseName = nosamename;
            if (nosamename != newname)
            {
                inform("Duplicate names! Naming has been automatically changed to" + nosamename);
            }
        }
    }
    else if (ok)
    {
        inform("Input is empty, rename failed");
    }
}

void MainWindow::onTreeViewRvizDispTreeClicked(const QModelIndex &index)
{
    mRvizDispChooseName = index.data().value<QString>();
    if (index.parent().row() == -1)
        if (index.row() > 1)
        {
            ui->remove_topic->setEnabled(true);
            ui->rename_topic->setEnabled(true);
        }
        else
        {
            ui->remove_topic->setEnabled(false);
            ui->rename_topic->setEnabled(false);
        }

    else
    {
        ui->remove_topic->setEnabled(false);
        ui->rename_topic->setEnabled(false);
    }
}

void MainWindow::on_button_connect_clicked()
{
    if ( ui->checkbox_use_environment->isChecked() )
    {
        if ( !qtNode.init() )
        {
            QMessageBox::warning(nullptr, "Failed", "Failed to connect to ROS Master!", QMessageBox::Yes, QMessageBox::Yes);
            ui->robot_status_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
            ui->robot_status_ind->setStyleSheet("color:red;");
            ui->robot_status_ind->setText("offline");
            ui->tabWidget->setTabEnabled(1,false);
            ui->gridLayout_7->setEnabled(false);
            return ;
        }
    }
    else
    {
        if ( !qtNode.init(ui->line_edit_master->text().toStdString(), ui->line_edit_host->text().toStdString()) )
        {
            QMessageBox::warning(nullptr, "fail", "Connect ROS Master failed!", QMessageBox::Yes, QMessageBox::Yes);
            ui->robot_status_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
            ui->robot_status_ind->setStyleSheet("color:red;");
            ui->robot_status_ind->setText("offline");
            ui->tabWidget->setTabEnabled(1,false);
            ui->gridLayout_7->setEnabled(false);
            return ;
        }
        else
        {
            ui->line_edit_master->setReadOnly(true);
            ui->line_edit_host->setReadOnly(true);
        }

    }
    ui->tabWidget->setTabEnabled(1,true);
    ui->gridLayout_7->setEnabled(true);
    initRviz();
    ui->button_connect->setEnabled(false);
    ui->robot_status_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
    ui->robot_status_ind->setStyleSheet("color:green;");
    ui->robot_status_ind->setText("online");
    ui->button_disconnect->setEnabled(true);
}

void MainWindow::on_button_disconnect_clicked()
{
    qDebug()<<"Gui exited";
    this->close();
}

void MainWindow::onBtnRvizReadDispSetClicked()
{
    if (mapRviz == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getOpenFileName(this, "import RVIZ Display configure", "/home/" + getUserName() + "/", "YAML(*.yaml);;ALL(*.*)");
    if (!path.isEmpty())
    {
        mapRviz->ReadDispSet(path);
    }
}

void MainWindow::onBtnRvizSaveDispSetClicked()
{
    if (mapRviz == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getSaveFileName(this, "export RVIZ Display configure", "/home/" + getUserName() + "/", "YAML(*.yaml)");

    if (!path.isEmpty())
    {
        if (path.section('/', -1, -1).indexOf('.') < 0)
        {
            path = path + ".yaml";
        }
        mapRviz->OutDispSet(path);
    }
}
QString MainWindow::getUserName()
{
    QString userPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    QString userName = userPath.section("/", -1, -1);
    return userName;
}

void MainWindow::slotRosShutdown()
{
    ui->robot_status_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
    ui->robot_status_ind->setStyleSheet("color:red;");
    ui->robot_status_ind->setText("offline");
    ui->button_connect->setEnabled(true);
    ui->line_edit_master->setReadOnly(false);
    ui->line_edit_host->setReadOnly(false);
}

void MainWindow::slotChooseTopic(QTreeWidgetItem *choose, QString name)
{
    QString ClassID = choose->text(0);
    name = judgeDispNewName(name);
    qDebug() << "choose topic ClassID:" << ClassID << ", name:" << name;
    QMap<QString, QVariant> namevalue;
    namevalue.clear();
    mapRviz->DisplayInit(mMapRvizDisp[ClassID], true, name, namevalue);
}

}
