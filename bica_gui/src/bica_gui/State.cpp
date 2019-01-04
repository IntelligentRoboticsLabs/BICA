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
#include <bica_gui/State.h>

#include <string>

namespace bica_gui
{
State::State(std::string id) : QGraphicsItem(), id_(id), initial_(false), debugActive_(false)
{
  setFlags(ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges);

  std::string path = ros::package::getPath("bica_gui");
  statePxmp_ = new QPixmap(QString::fromStdString(path + "/resources/RegularState.png"));
  stateActivePxmp_ = new QPixmap(QString::fromStdString(path + "/resources/RegularStateDbg.png"));

  last_active_ = ros::Time::now();
}

State::~State()
{
  delete statePxmp_;
}

void State::setInitial(bool initial)
{
  if (initial == initial_)
    return;
  initial_ = initial;

  delete statePxmp_;
  std::string path = ros::package::getPath("bica_gui");

  if (initial_)
    statePxmp_ = new QPixmap(QString::fromStdString(path + "/resources/InitialState.png"));
  else
    statePxmp_ = new QPixmap(QString::fromStdString(path + "/resources/RegularState.png"));

  update();
}

QRectF State::boundingRect() const
{
  return QRectF(-statePxmp_->width() / 2, -statePxmp_->height() / 2, statePxmp_->width(), statePxmp_->height());
}

void State::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  QPointF pos(-statePxmp_->width() / 2, -statePxmp_->height() / 2);

  if (debugActive_ && (ros::Time::now() - last_active_).toSec() < 1.0)
    painter->drawPixmap(pos, *stateActivePxmp_);
  else
    painter->drawPixmap(pos, *statePxmp_);

  QRectF myRect(-statePxmp_->width() / 2, -statePxmp_->height() / 2, statePxmp_->width(), statePxmp_->height());
  painter->drawText(myRect, Qt::AlignCenter | Qt::TextWrapAnywhere, QString::fromStdString(id_));
}

QVariant State::itemChange(GraphicsItemChange change, const QVariant& value)
{
  foreach(Dependency* dep, activations_)
    dep->trackNodes();
  foreach(Transition* tran, transitions_)
    tran->trackNodes();

  return QGraphicsItem::itemChange(change, value);
}

void State::addActivation(Dependency* dep)
{
  activations_.insert(dep);
}

void State::removeActivation(Dependency* dep)
{
  activations_.remove(dep);
}

void State::addTransition(Transition* tran)
{
  transitions_.insert(tran);
}

void State::removeTransition(Transition* tran)
{
  transitions_.remove(tran);
}

void State::debugIsActive(bool active)
{
  debugActive_ = active;
  last_active_ = ros::Time::now();

  QGraphicsItem::update(boundingRect());
}

}  // namespace bica_gui
