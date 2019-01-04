/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include <bica_gui/HFSMScene.h>

#include <string>
#include <vector>

namespace bica_gui
{
HfsmScene::HfsmScene(QObject* parent)
  : QGraphicsScene(parent), state_(IDLE), id_(""), path_(""), need_path_(false), active_state_("")
{
  char* ros_env = getenv("ROS_PACKAGE_PATH");
  std::string ros_env_str(ros_env);

  boost::char_separator<char> sep(":");
  boost::tokenizer<boost::char_separator<char> > tokens(ros_env_str, sep);

  path_ = (*tokens.begin());

  state_sub_ = nh_.subscribe<std_msgs::String>(id_ + "/state", 1, &HfsmScene::stateCB, this);
}

HfsmScene::~HfsmScene()
{
}

void HfsmScene::resizeEvent(QResizeEvent* ev)
{
}

void showEvent(QShowEvent* ev)
{
}

void HfsmScene::fitView()
{
}

void HfsmScene::dependencyContextMenu(QGraphicsSceneMouseEvent* mouseEvent, Dependency* dependency)
{
  QMenu depMenu;

  depMenu.addAction("Remove");
  QPoint globalPos = mouseEvent->screenPos();

  QAction* selectedItem = depMenu.exec(globalPos);

  if (selectedItem)
  {
    removeDependency(dependency);
    state_ = IDLE;
  }
  else
  {
    state_ = IDLE;
  }
}

void HfsmScene::transitionContextMenu(QGraphicsSceneMouseEvent* mouseEvent, Transition* transition)
{
  QMenu depMenu;

  depMenu.addAction("Remove");
  QPoint globalPos = mouseEvent->screenPos();

  QAction* selectedItem = depMenu.exec(globalPos);

  if (selectedItem)
  {
    removeTransition(transition);
    state_ = IDLE;
  }
  else
  {
    state_ = IDLE;
  }
}

void HfsmScene::stateContextMenu(QGraphicsSceneMouseEvent* mouseEvent, State* state)
{
  QMenu stateMenu;

  stateMenu.addAction("Rename");
  stateMenu.addAction("Remove");
  stateMenu.addAction("Add Transition");
  stateMenu.addAction("Add Dependency");
  stateMenu.addAction("Set initial");

  QPoint globalPos = mouseEvent->screenPos();

  QAction* selectedItem = stateMenu.exec(globalPos);

  if (selectedItem)
  {
    if (selectedItem->text().toStdString() == "Rename")
    {
      QInputDialog getIdDialog;
      getIdDialog.setWindowTitle("Rename state");
      getIdDialog.setLabelText("Select Id");
      getIdDialog.move(mouseEvent->screenPos());
      int result = getIdDialog.exec();

      if (result == QDialog::Accepted)
      {
        std::string id = getIdDialog.textValue().toUtf8().constData();

        if (!existState(id))
          state->setId(id);
        else
        {
          QMessageBox dialog;
          dialog.setWindowTitle("State exist");
          dialog.setText("State already exist");
          dialog.exec();
        }
      }
      state_ = IDLE;
    }
    else if (selectedItem->text().toStdString() == "Remove")
    {
      removeState(state);

      state_ = IDLE;
    }
    else if (selectedItem->text().toStdString() == "Add Dependency")
    {
      std::string path = ros::package::getPath("bica_gui");
      this->views().first()->setCursor(QCursor(QPixmap(QString::fromStdString(path + "/resources/Connection.png"))));

      transOrigin = state;
      state_ = ADD_DEPENDENCY;
    }
    else if (selectedItem->text().toStdString() == "Add Transition")
    {
      std::string path = ros::package::getPath("bica_gui");
      this->views().first()->setCursor(
          QCursor(QPixmap(QString::fromStdString(path + "/resources/NodeTransitionSmall.png"))));

      transOrigin = state;
      state_ = ADD_TRANSITION;
    }
    else if (selectedItem->text().toStdString() == "Set initial")
    {
      foreach(State* stateit, states_)
      {
        stateit->setInitial(false);
      }
      state->setInitial(true);

      state_ = IDLE;
    }
    else
      state_ = IDLE;
  }
  else
  {
    state_ = IDLE;
  }
}

void HfsmScene::componentContextMenu(QGraphicsSceneMouseEvent* mouseEvent, Component* component)
{
  QMenu stateMenu;

  stateMenu.addAction("Remove");
  stateMenu.addAction("Load");

  QPoint globalPos = mouseEvent->screenPos();

  QAction* selectedItem = stateMenu.exec(globalPos);

  if (selectedItem)
  {
    if (selectedItem->text().toStdString() == "Remove")
    {
      removeComponent(component);
      state_ = IDLE;
    }
    else if (selectedItem->text().toStdString() == "Load")
    {
      char* ros_env = getenv("ROS_PACKAGE_PATH");
      std::string ros_env_str(ros_env);

      boost::char_separator<char> sep(":");
      boost::tokenizer<boost::char_separator<char> > tokens(ros_env_str, sep);

      path_ = (*tokens.begin());

      // fprintf(stderr, "Looking for [%s] in [%s]\n", component->getId().c_str(), path_.c_str());

      boost::filesystem::path path(path_);

      std::string fullPath = getFullPath(path_, component->getId() + ".bica");

      // fprintf(stderr, "Found [%s] in [%s]\n", component->getId().c_str(), fullPath.c_str());

      if (fullPath != "")
      {
        clearAll();
        need_path_ = true;
        load(fullPath);
      }
      state_ = IDLE;
    }
    else
      state_ = IDLE;
  }
  else
  {
    state_ = IDLE;
  }
}

bool HfsmScene::removeState(State* state)
{
  this->removeItem(state);
  states_.remove(state);

  if (state->isInitial() && states_.size() > 0)
    (*states_.begin())->setInitial(true);

  foreach(Dependency* dep, state->getActivations())
  {
    this->removeItem(dep);
    delete dep;
  }

  foreach(Transition* tran, state->getTransition())
  {
    this->removeItem(tran);
    delete tran;
  }

  if (state != NULL)
    delete state;
}

bool HfsmScene::removeDependency(Dependency* dependency)
{
  this->removeItem(dependency);
  dependecies_.remove(dependency);
  delete dependency;
}

bool HfsmScene::removeTransition(Transition* transition)
{
  this->removeItem(transition);
  transitions_.remove(transition);

  delete transition;
}

bool HfsmScene::removeComponent(Component* component)
{
  this->removeItem(component);
  components_.remove(component);

  foreach(Dependency* dep, component->getActivations())
  {
    this->removeItem(dep);
    delete dep;
  }

  if (component != NULL)
    delete component;
}

void HfsmScene::voidContextMenu(QGraphicsSceneMouseEvent* mouseEvent)
{
  QMenu voidMenu;
  voidMenu.addAction("Add state");
  voidMenu.addAction("Add component");
  voidMenu.addAction("---");
  voidMenu.addAction("New component");
  voidMenu.addAction("Load...");
  voidMenu.addAction("Save...");
  voidMenu.addAction("Quick Save");

  std::string path = ros::package::getPath("bica_gui");

  QPoint globalPos = mouseEvent->screenPos();  // this->views().first()->mapToGlobal(mouseEvent->scenePos());
  QAction* selectedItem = voidMenu.exec(globalPos);

  if (selectedItem)
  {
    if (selectedItem->text().toStdString() == "Add state")
    {
      this->views().first()->setCursor(QCursor(QPixmap(QString::fromStdString(path + "/resources/RegularState.png"))));
      state_ = ADD_STATE;
    }
    else if (selectedItem->text().toStdString() == "Add component")
    {
      this->views().first()->setCursor(QCursor(QPixmap(QString::fromStdString(path + "/resources/Component.png"))));
      state_ = ADD_COMPONENT;
    }
    else if (selectedItem->text().toStdString() == "New component")
    {
      clearAll();
      state_ = IDLE;
    }
    else if (selectedItem->text().toStdString() == "Save...")
    {
      saveDialog();

      state_ = IDLE;
    }
    else if (selectedItem->text().toStdString() == "Quick Save")
    {
      if (id_ != "" && !need_path_)
        save(path_.c_str(), id_);
      else
      {
        QMessageBox dialog;
        dialog.setWindowTitle("No Id");
        dialog.setText("Save first for selecting Component Id");
        dialog.exec();
      }
      state_ = IDLE;
    }
    else if (selectedItem->text().toStdString() == "Load...")
    {
      loadDialog();

      state_ = IDLE;
    }
    else
      state_ = IDLE;
  }
  else
  {
    state_ = IDLE;
  }
}

void HfsmScene::addStateMenu(QGraphicsSceneMouseEvent* mouseEvent)
{
  QInputDialog getIdDialog;
  getIdDialog.setWindowTitle("add State");
  getIdDialog.setLabelText("Select Id");
  getIdDialog.move(mouseEvent->screenPos());
  int result = getIdDialog.exec();

  if (result == QDialog::Accepted)
  {
    std::string id = getIdDialog.textValue().toUtf8().constData();

    if (!existState(id))
      addState(id, mouseEvent->scenePos().x(), mouseEvent->scenePos().y());
    else
    {
      QMessageBox dialog;
      dialog.setWindowTitle("State exist");
      dialog.setText("State already exist");
      dialog.exec();
    }
  }
}

bool HfsmScene::existState(std::string id)
{
  QSet<State*>::iterator it;
  for (it = states_.begin(); it != states_.end(); ++it)
    if ((*it)->getId() == id)
      return true;

  return false;
}

bool HfsmScene::existComponent(std::string id)
{
  QSet<Component*>::iterator it;
  for (it = components_.begin(); it != components_.end(); ++it)
    if ((*it)->getId() == id)
      return true;

  return false;
}

void HfsmScene::addComponentMenu(QGraphicsSceneMouseEvent* mouseEvent)
{
  QMenu compsMenu;

  /*char* ros_env = getenv("ROS_PACKAGE_PATH");
std::string ros_env_str(ros_env);

boost::char_separator<char> sep(":");
boost::tokenizer<boost::char_separator<char> > tokens(ros_env_str, sep);


BOOST_FOREACH(std::string const& token, tokens)
{
  boost::filesystem::path path(token);

  std::vector<boost::filesystem::path> found;

  get_all(path, ".bica", found);

  for(std::vector<boost::filesystem::path>::iterator it=found.begin(); it!=found.end(); ++it)
  {
    compsMenu.addAction((it->string().substr(0, it->string().length()-5)).c_str());
  }
}
  */
  compsMenu.addAction("Add manual...");

  QPoint globalPos = mouseEvent->screenPos();
  QAction* selectedItem = compsMenu.exec(globalPos);

  if (selectedItem)
  {
    std::string id;
    if (selectedItem->text().toStdString() == "Add manual...")
    {
      QInputDialog getIdDialog;
      getIdDialog.setWindowTitle("Add manual");
      getIdDialog.setLabelText("Select Id");
      getIdDialog.move(mouseEvent->screenPos());
      int result = getIdDialog.exec();

      if (result == QDialog::Accepted)
      {
        std::string id = getIdDialog.textValue().toUtf8().constData();
        if (!existComponent(id))
          addComponent(id, mouseEvent->scenePos().x(), mouseEvent->scenePos().y());
        else
        {
          QMessageBox dialog;
          dialog.setWindowTitle("Component exist");
          dialog.setText("Component already exist");
          dialog.exec();
        }
      }
    }
  }
}

void HfsmScene::mousePressEvent(QGraphicsSceneMouseEvent* mouseEvent)
{
  QList<QGraphicsItem*> selected_items =
      items(mouseEvent->scenePos(), Qt::IntersectsItemBoundingRect, Qt::DescendingOrder);

  State* currentState = 0;
  Component* currentComp = 0;
  Dependency* currentDep = 0;
  Transition* currentTran = 0;

  if (selected_items.size() > 0)
  {
    currentState = dynamic_cast<State*>(*(selected_items.begin()));
    currentComp = dynamic_cast<Component*>(*(selected_items.begin()));
    currentDep = dynamic_cast<Dependency*>(*(selected_items.begin()));
    currentTran = dynamic_cast<Transition*>(*(selected_items.begin()));
  }

  switch (state_)
  {
    case IDLE:
      if ((mouseEvent->button() == Qt::RightButton) && (currentState != 0))
        stateContextMenu(mouseEvent, currentState);
      else if ((mouseEvent->button() == Qt::RightButton) && (currentComp != 0))
        componentContextMenu(mouseEvent, currentComp);
      else if ((mouseEvent->button() == Qt::RightButton) && (currentDep != 0))
        dependencyContextMenu(mouseEvent, currentDep);
      else if ((mouseEvent->button() == Qt::RightButton) && (currentTran != 0))
        transitionContextMenu(mouseEvent, currentTran);
      else if ((mouseEvent->button() == Qt::RightButton))
        voidContextMenu(mouseEvent);
      else
        state_ = IDLE;

      break;

    case ADD_STATE:

      if ((mouseEvent->button() == Qt::LeftButton) && (currentState == 0))
      {
        addStateMenu(mouseEvent);

        this->views().first()->setCursor(Qt::ArrowCursor);
        state_ = IDLE;
      }
      else
      {
        this->views().first()->setCursor(Qt::ArrowCursor);
        state_ = IDLE;
      }
      break;

    case ADD_COMPONENT:

      if ((mouseEvent->button() == Qt::LeftButton) && (currentState == 0))
      {
        addComponentMenu(mouseEvent);

        this->views().first()->setCursor(Qt::ArrowCursor);
        state_ = IDLE;
      }
      else
      {
        this->views().first()->setCursor(Qt::ArrowCursor);
        state_ = IDLE;
      }
      break;

    case ADD_DEPENDENCY:

      if ((mouseEvent->button() == Qt::LeftButton) && (currentComp != 0))
      {
        addDependency(transOrigin, currentComp);

        this->views().first()->setCursor(Qt::ArrowCursor);
        state_ = IDLE;
      }
      else
      {
        this->views().first()->setCursor(Qt::ArrowCursor);
        state_ = IDLE;
      }

      break;

    case ADD_TRANSITION:

      if ((mouseEvent->button() == Qt::LeftButton) && (currentState != 0))
      {
        addTransition(transOrigin, currentState);

        this->views().first()->setCursor(Qt::ArrowCursor);
        state_ = IDLE;
      }
      else
      {
        this->views().first()->setCursor(Qt::ArrowCursor);
        state_ = IDLE;
      }

      break;
  }

  QGraphicsScene::mousePressEvent(mouseEvent);
}

State* HfsmScene::addState(std::string id, int x, int y)
{
  State* newState = new State(id);

  newState->setX(x);
  newState->setY(y);

  if (states_.empty())
    newState->setInitial(true);

  states_.insert(newState);

  addItem(newState);

  return newState;
}

void HfsmScene::addComponent(std::string id, int x, int y)
{
  Component* newComp = new Component(id);

  newComp->setX(x);
  newComp->setY(y);

  components_.insert(newComp);
  addItem(newComp);
}

void HfsmScene::addDependency(State* state, Component* comp)
{
  QSet<Dependency*>::iterator it;
  for (it = dependecies_.begin(); it != dependecies_.end(); ++it)
  {
    if (((*it)->getState() == state) && ((*it)->getComponent() == comp))
    {
      QMessageBox dialog;
      dialog.setWindowTitle("Dependency exist");
      dialog.setText("Dependency already exist");
      dialog.exec();

      return;
    }
  }

  Dependency* newDep = new Dependency(state, comp);
  dependecies_.insert(newDep);
  state->addActivation(newDep);
  comp->addActivation(newDep);
  addItem(newDep);
}

void HfsmScene::addTransition(State* src, State* dst)
{
  QSet<Transition*>::iterator it;
  for (it = transitions_.begin(); it != transitions_.end(); ++it)
  {
    if (((*it)->getSrcState() == src) && ((*it)->getDestinyState() == dst))
    {
      QMessageBox dialog;
      dialog.setWindowTitle("Transition exist");
      dialog.setText("Transition already exist");
      dialog.exec();

      return;
    }
  }

  if (src == dst)
  {
    QMessageBox dialog;
    dialog.setWindowTitle("Same states");
    dialog.setText("Source and Destiny are the same");
    dialog.exec();

    return;
  }

  Transition* newTran = new Transition(src, dst);

  transitions_.insert(newTran);
  src->addTransition(newTran);
  dst->addTransition(newTran);
  addItem(newTran);
}

void HfsmScene::get_all(const boost::filesystem::path& root, const std::string& ext,
                        std::vector<boost::filesystem::path>& ret)
{
  if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
    return;

  boost::filesystem::recursive_directory_iterator it(root, boost::filesystem::symlink_option::recurse);
  boost::filesystem::recursive_directory_iterator endit;

  while (it != endit)
  {
    if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
      ret.push_back(it->path().filename());
    ++it;
  }
}

std::string HfsmScene::getFullPath(const boost::filesystem::path& root, const std::string& filename)
{
  if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
    return "";

  boost::filesystem::recursive_directory_iterator it(root, boost::filesystem::symlink_option::recurse);
  boost::filesystem::recursive_directory_iterator endit;

  while (it != endit)
  {
    if (it->path().filename() == "ir_planning" ||
        it->path().filename() == "pepper_navigation_bringup" ||
        it->path().filename() == "ROSPlan")
    {
      it.no_push();  // don't recurse into this directory.
    }
    if (boost::filesystem::is_regular_file(*it) && it->path().filename() == filename){
      fprintf(stderr, "\tit->path().string() [%s]\n", it->path().string().c_str());
      return it->path().string();
    }
    ++it;
  }

  return "";
}

void HfsmScene::clearAll()
{
  foreach(Dependency* dep, dependecies_)
  {
    delete dep;
  }
  dependecies_.clear();

  foreach(Transition* tran, transitions_)
  {
    delete tran;
  }
  transitions_.clear();

  foreach(State* state, states_)
  {
    delete state;
  }
  states_.clear();

  foreach(Component* comp, components_)
  {
    delete comp;
  }
  components_.clear();

  this->clear();
}

void HfsmScene::save(const std::string& path, const std::string& id)
{
  // fprintf(stderr, "Saving \n\tpath: [%s]\n\tid: [%s]\n", path.c_str(), id.c_str());

  boost::char_separator<char> sep("/");
  boost::tokenizer<boost::char_separator<char> > tokens(path, sep);

  std::string pkgName;
  BOOST_FOREACH(std::string const& token, tokens)
  pkgName = token;

  // fprintf(stderr, "\tpkg: [%s]\n", pkgName.c_str());

  generateBica(path + "/" + id + ".bica");
  generateHeader(path + "/" + id + ".h");
  generateSource(pkgName, path + "/" + id + ".cpp");
}

void HfsmScene::load(const std::string& filename)
{
  clearAll();

  QFile file(filename.c_str());

  if (!file.open(QIODevice::ReadOnly))
  {
    QMessageBox dialog;
    dialog.setWindowTitle("Fail to open");
    dialog.setText("Failing opening file");
    dialog.exec();
    return;
  }

  QByteArray content = file.readAll();
  QDomDocument doc;

  QString errorMessage;
  int line, col;

  if (!doc.setContent(content, &errorMessage, &line, &col))
  {
    QMessageBox dialog;
    dialog.setWindowTitle("Fail to open");
    dialog.setText("Error openning. More info in stderr");

    std::cerr << "Error in Line " << line << ", column " << col << ":" << qPrintable(errorMessage) << endl;

    dialog.exec();
    return;
  }

  QDomElement root = doc.documentElement();

  if (root.hasAttribute("id"))
  {
    QDomAttr attr = root.attributeNode("id");
    id_ = attr.value().toStdString();
    state_sub_ = nh_.subscribe<std_msgs::String>(id_ + "/state", 1, &HfsmScene::stateCB, this);
  }

  // 1st States and components only
  QDomNode node = root.firstChild();
  while (!node.isNull())
  {
    if (node.isElement())
    {
      QDomElement elem = node.toElement();

      if (elem.tagName().toStdString() == "state")
      {
        int x, y;
        std::string id;
        bool initial;

        x = atoi(elem.attribute("x").toStdString().c_str());
        y = atoi(elem.attribute("y").toStdString().c_str());
        id = elem.attribute("id").toStdString();
        initial = (elem.attribute("initial").toStdString() == "true") ? true : false;
        // fprintf(stderr, "State [%s] at (%d, %d) initial: %s\n", id.c_str(), x, y, initial?"true":"false");

        State* newState = addState(id, x, y);
        newState->setInitial(initial);
      }

      if (elem.tagName().toStdString() == "component")
      {
        int x, y;
        std::string id;

        x = atoi(elem.attribute("x").toStdString().c_str());
        y = atoi(elem.attribute("y").toStdString().c_str());
        id = elem.attribute("id").toStdString();

        // fprintf(stderr, "Component [%s] at (%d, %d)\n", id.c_str(), x, y);

        addComponent(id, x, y);
      }
    }
    node = node.nextSibling();
  }

  // 2nd Transitions and dependencies
  node = root.firstChild();
  while (!node.isNull())
  {
    if (node.isElement())
    {
      QDomElement elem = node.toElement();

      if (elem.tagName().toStdString() == "state")
      {
        int x, y;
        std::string id;

        x = atoi(elem.attribute("x").toStdString().c_str());
        y = atoi(elem.attribute("y").toStdString().c_str());
        id = elem.attribute("id").toStdString();

        // fprintf(stderr, "State [%s] at (%d, %d)\n", id.c_str(), x, y);

        State* currstate = getState(id);

        if (elem.hasChildNodes())
        {
          QDomNode activations = elem.firstChild();
          while (!activations.isNull())
          {
            if (activations.isElement())
            {
              QDomElement elemact = activations.toElement();
              if (elemact.tagName().toStdString() == "activation")
              {
                std::string actid;
                actid = elemact.attribute("id").toStdString();

                addDependency(currstate, getComponent(actid));

                // fprintf(stderr, "\tActivation [%s]\n", actid.c_str());
              }
            }

            activations = activations.nextSibling();
          }
        }
      }

      if (elem.tagName().toStdString() == "transition")
      {
        std::string src, dst;
        src = elem.attribute("src").toStdString();
        dst = elem.attribute("dst").toStdString();

        addTransition(getState(src), getState(dst));
      }
    }
    node = node.nextSibling();
  }
}

State* HfsmScene::getState(const std::string& id)
{
  foreach(State* state, states_)
  {
    if (state->getId() == id)
      return state;
  }

  fprintf(stderr, "No se encontró estado [%s]", id.c_str());

  return NULL;
}

Component* HfsmScene::getComponent(const std::string& id)
{
  foreach(Component* component, components_)
  {
    if (component->getId() == id)
      return component;
  }

  fprintf(stderr, "No se encontró component [%s]", id.c_str());

  return NULL;
}

void HfsmScene::loadDialog()
{
  char* ros_env = getenv("ROS_PACKAGE_PATH");
  std::string ros_env_str(ros_env);

  boost::char_separator<char> sep(":");
  boost::tokenizer<boost::char_separator<char> > tokens(ros_env_str, sep);

  QFileDialog fileDialog;
  fileDialog.setNameFilter(tr("Images (*.bica)"));
  fileDialog.setDirectory((*tokens.begin()).c_str());

  int fileResult = fileDialog.exec();

  if (fileResult != QDialog::Accepted)
    return;

  need_path_ = true;

  load((*fileDialog.selectedFiles().begin()).toStdString());
}

void HfsmScene::saveDialog()
{
  QInputDialog getIdDialog;
  getIdDialog.setWindowTitle("Component Id");
  getIdDialog.setLabelText("Select Id for this component");
  getIdDialog.setTextValue(id_.c_str());

  int result = getIdDialog.exec();

  if (result == QDialog::Accepted)
  {
    id_ = getIdDialog.textValue().toStdString();
    state_sub_ = nh_.subscribe<std_msgs::String>(id_ + "/state", 1, &HfsmScene::stateCB, this);
  }
  else
    return;

  char* ros_env = getenv("ROS_PACKAGE_PATH");
  std::string ros_env_str(ros_env);

  boost::char_separator<char> sep(":");
  boost::tokenizer<boost::char_separator<char> > tokens(ros_env_str, sep);

  QFileDialog dirDialog;
  dirDialog.setFileMode(QFileDialog::DirectoryOnly);
  dirDialog.setDirectory((*tokens.begin()).c_str());

  int dirResult = dirDialog.exec();

  if (dirResult != QDialog::Accepted)
    return;

  path_ = (*dirDialog.selectedFiles().begin()).toStdString();

  need_path_ = false;

  save(path_.c_str(), id_);
}

void HfsmScene::generateBica(const std::string& file)
{
  QDomDocument doc;
  QDomElement componentRoot = doc.createElement("component");
  componentRoot.setAttribute("id", id_.c_str());

  foreach(State* state, states_)
  {
    QDomElement stateElem = doc.createElement("state");
    stateElem.setAttribute("id", state->getId().c_str());
    stateElem.setAttribute("x", state->x());
    stateElem.setAttribute("y", state->y());
    stateElem.setAttribute("initial", state->isInitial() ? "true" : "false");

    foreach(Dependency* dep, state->getActivations())
    {
      QDomElement compElem = doc.createElement("activation");
      compElem.setAttribute("id", dep->getComponent()->getId().c_str());
      stateElem.appendChild(compElem);
    }
    componentRoot.appendChild(stateElem);
  }

  foreach(Component* component, components_)
  {
    QDomElement compElem = doc.createElement("component");
    compElem.setAttribute("id", component->getId().c_str());
    compElem.setAttribute("x", component->x());
    compElem.setAttribute("y", component->y());
    componentRoot.appendChild(compElem);
  }

  foreach(Transition* tran, transitions_)
  {
    QDomElement transElem = doc.createElement("transition");
    transElem.setAttribute("src", tran->getSrcState()->getId().c_str());
    transElem.setAttribute("dst", tran->getDestinyState()->getId().c_str());
    componentRoot.appendChild(transElem);
  }

  doc.appendChild(componentRoot);

  QFile data(file.c_str());
  if (data.open(QFile::WriteOnly | QFile::Truncate))
  {
    QTextStream out(&data);
    out << doc.toString();
  }
}

std::string HfsmScene::getFileContent(const std::string& file)
{
  char* ros_env = getenv("ROS_PACKAGE_PATH");
  std::string ros_env_str(ros_env);

  boost::char_separator<char> sep(":");
  boost::tokenizer<boost::char_separator<char> > tokens(ros_env_str, sep);

  boost::filesystem::path path((*tokens.begin()));
  std::string fullPath = getFullPath((*tokens.begin()), file);

  // fprintf(stderr, "reading [%s]\n", fullPath.c_str());

  std::ifstream t(fullPath.c_str());
  std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

  fprintf(stderr, "reading fullPath [%s]\n", fullPath.c_str());
  fprintf(stderr, "reading str [%s]\n", str.c_str());

  return str;
}

void HfsmScene::replaceAll(std::string& str, const std::string& from, const std::string& to)
{
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos)
  {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();  // In case 'to' contains 'from', like replacing 'x' with 'yx'
  }
}

std::string HfsmScene::getFunctionsFromStates()
{
  std::string ret;

  foreach(State* state, states_)
  {
    ret += "\tvirtual void " + state->getId() + "_code_iterative() {};\n";
    ret += "\tvirtual void " + state->getId() + "_code_once() {};\n";
  }

  return ret;
}

std::string HfsmScene::getFunctionsFromTransitions()
{
  std::string ret;

  foreach(Transition* tran, transitions_)
  {
    ret += "\tvirtual bool " + tran->getSrcState()->getId() + "_2_" + tran->getDestinyState()->getId() + "() {return "
                                                                                                         "false;};\n";
  }

  return ret;
}

std::string HfsmScene::getStates()
{
  std::string ret;
  int counter = 0;

  foreach(State* state, states_)
  {
    ret += "\tstatic const int " + boost::to_upper_copy<std::string>(state->getId()) + " = " +
           std::to_string(counter++) + ";\n";
  }

  return ret;
}

std::string HfsmScene::getActivFunctionsFromStates()
{
  std::string ret;

  ret += "\tvoid deactivateAllDeps();\n";

  foreach(State* state, states_)
  {
    // ret += "\tvoid "+state->getId()+"_DeactivateDeps();\n";
    ret += "\tvoid " + state->getId() + "_activateDeps();\n";
  }

  return ret;
}

void HfsmScene::generateHeader(const std::string& file)
{
  std::string path = ros::package::getPath("bica_gui");
  std::string pattern = getFileContent("patternSrc.h");

  fprintf(stderr, "%s\n", pattern.c_str());

  replaceAll(pattern, "IDM", boost::to_upper_copy<std::string>(id_));
  replaceAll(pattern, "ID", id_);
  replaceAll(pattern, "FUNCTIONS_STATES", getFunctionsFromStates());
  replaceAll(pattern, "FUNCTIONS_TRANSITIONS", getFunctionsFromTransitions());
  replaceAll(pattern, "FUNCTIONS_ACTS", getActivFunctionsFromStates());
  replaceAll(pattern, "STATES", getStates());

  std::ofstream out(file);
  out << pattern;
  out.close();
}

State* HfsmScene::getInitial()
{
  foreach(State* state, states_)
  {
    if (state->isInitial())
      return state;
  }

  fprintf(stderr, "No hay initial\n");

  return NULL;
}

std::string HfsmScene::getStateIteration(State* state)
{
  std::string ret;
  ret += "\tcase " + boost::to_upper_copy<std::string>(state->getId()) + ":\n\n";
  ret += "\t" + state->getId() + "_code_iterative();\n\n";

  ret += "\tmsg.data = \"" + state->getId() + "\";\n";

  foreach(Transition* tran, state->getTransition())
  {
    if (state == tran->getSrcState())
    {
      ret += "\tif(" + tran->getSrcState()->getId() + "_2_" + tran->getDestinyState()->getId() + "())\n\t{\n";
      ret += "\n" + getTransitionCodeTo(tran->getDestinyState()) + "\t}\n";
    }
  }

  ret += "\tstate_pub_.publish(msg);\n";
  ret += "\tbreak;\n\n";

  return ret;
}

std::string HfsmScene::getTransitionCodeTo(State* state)
{
  std::string ret;

  ret += "\tdeactivateAllDeps();\n\n";
  ret += "\tstate_ = " + boost::to_upper_copy<std::string>(state->getId()) + ";\n";
  ret += "\tstate_ts_ = ros::Time::now();\n\n";
  ret += "\t" + state->getId() + "_activateDeps();\n";
  ret += "\t" + state->getId() + "_code_once();\n";

  return ret;
}

std::string HfsmScene::getAllStateIterations()
{
  std::string ret;
  foreach(State* state, states_)
  {
    ret += getStateIteration(state);
  }

  return ret;
}

std::string HfsmScene::getAllActivations()
{
  std::string ret;

  ret += "void\n" + id_ + "::deactivateAllDeps()\n{\n";

  foreach(Component* component, components_)
  {
    ret += "\tremoveDependency(\"" + component->getId() + "\");\n";
  }

  ret += "};\n\n";
  foreach(State* state, states_)
  {
    ret += "void\n" + id_ + "::" + state->getId() + "_activateDeps()\n{\n";

    foreach(Dependency* dependency, state->getActivations())
    {
      ret += "\taddDependency(\"" + dependency->getComponent()->getId() + "\");\n";
    }

    ret += "}\n\n";
  }
  return ret;
}

void HfsmScene::generateSource(const std::string& pkg, const std::string& file)
{
  std::string pattern = getFileContent("patternSrc.cpp");

  fprintf(stderr, "%s\n", pattern.c_str());

  replaceAll(pattern, "ID", id_);
  replaceAll(pattern, "PKG", pkg);
  replaceAll(pattern, "INITIAL_STATE", boost::to_upper_copy<std::string>(getInitial()->getId()));
  replaceAll(pattern, "INITIAL_ITERATION", getTransitionCodeTo(getInitial()));
  replaceAll(pattern, "STATES_ITERATIONS", getAllStateIterations());
  replaceAll(pattern, "STATE_ACTIVATIONS", getAllActivations());

  fprintf(stderr, "Saving ccp in [%s]", file.c_str());

  std::ofstream out(file);
  out << pattern;
  out.close();
}

void HfsmScene::stateCB(const std_msgs::String::ConstPtr& msg)
{
  State* selected = getState(msg->data);

  if (selected != NULL)
  {
    foreach(State* state, states_)
    {
      if (state == selected)
        state->debugIsActive(true);
      else
        state->debugIsActive(false);
    }
  }
}

}  // namespace bica_gui
