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
#include <bica_gui/Component.h>

#include <string>

namespace bica_gui
{
Component::Component(std::string id) : QGraphicsItem(), id_(id)
{
  setFlags(ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges);

  std::string path = ros::package::getPath("bica_gui");
  ComponentPxmp_ = new QPixmap(QString::fromStdString(path + "/resources/Component.png"));
  ComponentActPxmp_ = new QPixmap(QString::fromStdString(path + "/resources/ComponentAct.png"));
}

Component::~Component()
{
  delete ComponentPxmp_;
  delete ComponentActPxmp_;
}

QRectF Component::boundingRect() const
{
  return QRectF(-ComponentPxmp_->width() / 2, -ComponentPxmp_->height() / 2, ComponentPxmp_->width(),
                ComponentPxmp_->height());
}

void Component::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  QPointF pos(-ComponentPxmp_->width() / 2, -ComponentPxmp_->height() / 2);

  painter->drawPixmap(pos, *ComponentPxmp_);

  QRectF myRect(-ComponentPxmp_->width() / 2, -ComponentPxmp_->height() / 2, ComponentPxmp_->width(),
                ComponentPxmp_->height());
  painter->drawText(myRect, Qt::AlignCenter | Qt::TextWrapAnywhere, QString::fromStdString(id_));
}

QVariant Component::itemChange(GraphicsItemChange change, const QVariant& value)
{
  foreach(Dependency* dep, activations_)
    dep->trackNodes();

  return QGraphicsItem::itemChange(change, value);
}

void Component::addActivation(Dependency* dep)
{
  activations_.insert(dep);
}

void Component::removeActivation(Dependency* dep)
{
  activations_.remove(dep);
}

}  // namespace bica_gui
