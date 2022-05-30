#include "../include/qt_map_rviz_ros/qtrviz.hpp"

QtRviz::QtRviz(QVBoxLayout * layout, QString nodeName)
{
  nullMap.clear();
  this->Layout=layout;
  this->nodename=nodeName;
  renderPanel = new rviz::RenderPanel;
  Layout->addWidget(renderPanel);
  manager = new rviz::VisualizationManager(renderPanel);
  dispGroup = manager->getRootDisplayGroup();
  toolManager = manager->getToolManager();
  renderPanel->initialize(manager->getSceneManager(),manager);
  manager->initialize();
  toolManager->initialize();
  manager->removeAllDisplays();
}

QtRviz::~QtRviz()
{
  if(Layout != nullptr && renderPanel != nullptr)
    Layout->removeWidget(renderPanel);
  if(renderPanel != nullptr)
    delete renderPanel;
  if(manager != nullptr)
    delete manager;
  if(currentTool != nullptr)
    delete currentTool;
  if(toolManager != nullptr)
    delete toolManager;
  ROS_INFO("Rviz is shutdown");
}

void QtRviz::GetDispTreeModel()
{
  rviz::PropertyTreeModel * rvizModel = manager->getDisplayTreeModel();
  QAbstractItemModel * model = rvizModel;
  emit ReturnModelSignal(model);
}

void QtRviz::DisplayInit(QString classID, bool enabled, QMap<QString, QVariant> nameValue)
{
  int num = GetDispNumName(classID);
  if(num == -1)
  {
    rviz::Display * rvizDisp = manager->createDisplay(classID,classID,true);
    ROS_ASSERT(rvizDisp != nullptr);
    num = GetDispNum(classID);
  }
  if(!nameValue.empty())
  {
    QMap<QString, QVariant>::iterator it;
    for(it = nameValue.begin(); it != nameValue.end(); it++)
    {
      dispGroup->getDisplayAt(num)->subProp(it.key())->setValue(it.value());
    }
  }
  dispGroup->getGroupAt(num)->setEnabled(enabled);
  manager->startUpdate();
}

void QtRviz::DisplayInit(QString classID, bool enabled, QString name, QMap<QString, QVariant> nameValue)
{
  int num = GetDispNum(classID, name);
  if(num == -1)
  {
    rviz::Display * rvizDisp = manager->createDisplay(classID, name, true);
    ROS_ASSERT(rvizDisp != nullptr);
    num = GetDispNum(classID, name);
  }
  if(!nameValue.empty())
  {
    QMap<QString, QVariant>::iterator it;
    for(it = nameValue.begin(); it != nameValue.end(); it++)
    {
      dispGroup->getDisplayAt(num)->subProp(it.key())->setValue(it.value());
    }
    dispGroup->getDisplayAt(num)->setEnabled(enabled);
    manager->startUpdate();
  }
}

void QtRviz::RemoveDisp(QString name)
{
  int num = GetDispNumName(name);
  if(num == -1)
  {
    return ;
  }
  delete dispGroup->getDisplayAt(num);
}

void QtRviz::RemoveDisp(QString classID, QString name)
{
  int num = GetDispNum(classID,name);
  if(num == -1)
  {
    return;
  }
  delete dispGroup->getDisplayAt(num);
}

void QtRviz::OutDispSet(QString path)
{
  if(!path.isEmpty())
  {
    if(manager == nullptr)
    {
      return ;
    }
    rviz::Config con;
    manager->save(con);
    rviz::YamlConfigWriter yamlConfigWrite;
    yamlConfigWrite.writeFile(con, path);
  }
}

void QtRviz::ReadDispSet(QString path)
{
  if(!path.isEmpty())
  {
    return ;
  }
  rviz::YamlConfigReader yamlConfigRead;
  rviz::Config con;
  yamlConfigRead.readFile(con, path);
  manager->load(con);
}

int QtRviz::GetDispNum(QString classID)
{
  int num = -1;
  for(int i = 0; i < dispGroup->numDisplays(); i++)
  {
    if(dispGroup->getDisplayAt(i) != nullptr)
    {
      if(classID == dispGroup->getDisplayAt(i)->getClassId())
      {
        num = i;
        break;
      }
    }
  }
  return  num;
}

int QtRviz::GetDispNum(QString classID, QString name)
{
  int num = -1;
  for(int i = 0; i < dispGroup->numDisplays(); i++)
  {
    if(dispGroup->getDisplayAt(i) != nullptr)
    {
      if(classID == dispGroup->getDisplayAt(i)->getClassId() && name == dispGroup->getDisplayAt(i)->getName())
      {
        num = i;
        break;
      }
    }
  }
  return num;
}

int QtRviz::GetDispNumName(QString name)
{
  int num = -1;
  for(int i = 0; i < dispGroup->numDisplays(); i++)
  {
    if(dispGroup->getDisplayAt(i) != nullptr)
    {
      if(name == dispGroup->getDisplayAt(i)->getName())
      {
        num = i;
        break;
      }
    }
  }
  return num;
}

void QtRviz::SetGlobalOptions(QString frameName, QColor backColor, int frameRate)
{
  manager->setFixedFrame(frameName);
  manager->setProperty("Background Color",backColor);
  manager->setProperty("Frame Rate",frameRate);
  manager->startUpdate();
}

void QtRviz::SetPos()
{
  currentTool = toolManager->addTool("rviz/SetInitialPose");
  toolManager->setCurrentTool(currentTool);
  manager->startUpdate();
}

void QtRviz::SetGoal()
{
  currentTool = toolManager->addTool("rviz/SetGoal");
  rviz::Property * prop = currentTool->getPropertyContainer();
  prop->subProp("Topic")->setValue("/move_base_simple/goal");
  manager->setFixedFrame("map");
  toolManager->setCurrentTool(currentTool);
  manager->stopUpdate();
}

void QtRviz::MoveCamera()
{
  currentTool = toolManager->addTool("rviz/MoveCamera");
  toolManager->setCurrentTool(currentTool);
  manager->startUpdate();
}

void QtRviz::Select()
{
  currentTool = toolManager->addTool("rviz/Select");
  toolManager->setCurrentTool(currentTool);
  manager->startUpdate();
}
int QtRviz::RenameDisp(QString oldname, QString newname)
{
    int num = GetDispNumName(oldname);
    if (num == -1)
    {
        return 0;
    }
    dispGroup->getDisplayAt(num)->setName(newname);
    return 1;
}
