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
/*
 * Dependency.cpp
 *
 *  Created on: 18/05/2016
 *      Author: paco
 */

#include <bica_gui/Dependency.h>

namespace bica_gui
{
Dependency::Dependency(State* state, Component* comp)
  : QGraphicsLineItem(comp->scenePos().x(), comp->scenePos().y(), state->scenePos().x(), state->scenePos().y())
  , comp_(comp)
  , state_(state)
{
  setFlags(ItemIsSelectable);
  setZValue(-1);
  trackNodes();
}

Dependency::Dependency(const Dependency& other)
  : QGraphicsLineItem(other.comp_->scenePos().x(), other.comp_->scenePos().y(), other.state_->scenePos().x(),
                      other.state_->scenePos().y())
  , comp_(other.comp_)
  , state_(other.state_)
{
  setFlags(ItemIsSelectable);

  trackNodes();
}

Dependency::~Dependency()
{
  comp_->removeActivation(this);
  state_->removeActivation(this);
}

void Dependency::trackNodes()
{
  QPen pen;
  pen.setColor(QColor(0, 0, 255));
  pen.setStyle(Qt::DashLine);
  pen.setWidth(5);

  setPen(pen);

  setLine(QLineF(state_->pos(), comp_->pos()));
}

QRectF Dependency::boundingRect() const
{
  qreal extra = (pen().width() + 20) / 2.0;

  return QRectF(line().p1(), QSizeF(line().p2().x() - line().p1().x(), line().p2().y() - line().p1().y()))
      .normalized()
      .adjusted(-extra, -extra, extra, extra);
}

QPainterPath Dependency::shape() const
{
  QPainterPath path = QGraphicsLineItem::shape();
  return path;
}
};  // namespace bica_gui
