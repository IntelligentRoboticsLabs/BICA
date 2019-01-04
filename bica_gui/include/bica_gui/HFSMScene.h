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
#ifndef BICA_GUI_HFSMSCENE_H
#define BICA_GUI_HFSMSCENE_H

#include <iostream>
#include <QGraphicsScene>
#include <QPen>
#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QGraphicsSceneDragDropEvent>
#include <QPointF>
#include <QMessageBox>
#include <QInputDialog>
#include <QGraphicsView>
#include <QMenu>
#include <QFileDialog>

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <bica_gui/State.h>
#include <bica_gui/Component.h>
#include <bica_gui/Dependency.h>
#include <bica_gui/Transition.h>

#include <string>
#include <fstream>
#include <streambuf>

#include <QtXml/QtXml>
#include <QTextStream>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <vector>

namespace bica_gui
{
class HfsmScene : public QGraphicsScene
{
public:
  explicit HfsmScene(QObject* parent = 0);
  virtual ~HfsmScene();

  void mousePressEvent(QGraphicsSceneMouseEvent* mouseEvent);

  State* addState(std::string id, int x, int y);
  void addComponent(std::string id, int x, int y);
  void addDependency(State* state, Component* comp);
  void addTransition(State* src, State* dst);

  /*	void addTansition(std::string id, State *src, State *dst);
    void addDependency(State *state, Component *comp);
  */

  void resizeEvent(QResizeEvent* ev);
  void showEvent(QShowEvent* ev);
  void fitView();

  void stateCB(const std_msgs::String::ConstPtr& msg);

private:
  void get_all(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret);
  std::string getFullPath(const boost::filesystem::path& root, const std::string& filename);

  void voidContextMenu(QGraphicsSceneMouseEvent* mouseEvent);
  void stateContextMenu(QGraphicsSceneMouseEvent* mouseEvent, State* state);
  void componentContextMenu(QGraphicsSceneMouseEvent* mouseEvent, Component* component);
  void dependencyContextMenu(QGraphicsSceneMouseEvent* mouseEvent, Dependency* dependency);
  void transitionContextMenu(QGraphicsSceneMouseEvent* mouseEvent, Transition* transition);

  void addStateMenu(QGraphicsSceneMouseEvent* mouseEvent);
  void addComponentMenu(QGraphicsSceneMouseEvent* mouseEvent);

  bool existState(std::string id);
  bool existComponent(std::string id);

  bool removeState(State* state);
  bool removeComponent(Component* component);
  bool removeDependency(Dependency* dependency);
  bool removeTransition(Transition* transition);

  void clearAll();
  void save(const std::string& path, const std::string& id);
  void load(const std::string& filename);

  void saveDialog();
  void loadDialog();

  void generateBica(const std::string& file);

  std::string getFileContent(const std::string& file);
  void replaceAll(std::string& str, const std::string& from, const std::string& to);
  std::string getFunctionsFromStates();
  std::string getFunctionsFromTransitions();
  std::string getActivFunctionsFromStates();
  std::string getStates();
  void generateHeader(const std::string& file);

  State* getInitial();
  std::string getStateIteration(State* state);
  std::string getAllStateIterations();
  std::string getTransitionCodeTo(State* state);
  std::string getAllActivations();
  void generateSource(const std::string& pkg, const std::string& file);

  State* getState(const std::string& id);
  Component* getComponent(const std::string& id);

  static const int IDLE = 0;
  static const int ADD_STATE = 1;
  static const int ADD_COMPONENT = 2;
  static const int ADD_DEPENDENCY = 3;
  static const int ADD_TRANSITION = 4;

  int state_;

  State* transOrigin;

  QSet<State*> states_;
  QSet<Component*> components_;
  QSet<Dependency*> dependecies_;
  QSet<Transition*> transitions_;

  std::string id_;
  std::string path_;

  bool need_path_;

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Time state_ts_;
  std::string active_state_;
};

}  // namespace bica_gui
#endif  // BICA_GUI_HFSMSCENE_H
