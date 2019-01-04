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
#include <bica_gui/Transition.h>

namespace bica_gui
{
Transition::Transition(State* src, State* dst) : QGraphicsPolygonItem(), src_(src), dst_(dst)
{
  QGraphicsPolygonItem::setFlags(ItemIsSelectable);
  QGraphicsPolygonItem::setZValue(-1);
  trackNodes();
}

Transition::Transition(const Transition& other) : QGraphicsPolygonItem(), src_(other.src_), dst_(other.dst_)
{
  QGraphicsPolygonItem::setFlags(ItemIsSelectable);
  QGraphicsPolygonItem::setZValue(-1);
  trackNodes();
}

Transition::~Transition()
{
  src_->removeTransition(this);
  dst_->removeTransition(this);
}

void Transition::trackNodes()
{
  QPen pen;
  pen.setColor(Qt::red);
  pen.setStyle(Qt::SolidLine);
  pen.setWidth(5);

  setPen(pen);

  QPointF intersectPoint;
  intersectPoint = (src_->pos() + dst_->pos()) / 2;

  QLineF centerLine(src_->pos(), dst_->pos());

  qreal arrowSize = 20;
  double angle = ::acos(centerLine.dx() / centerLine.length());
  if (centerLine.dy() >= 0)
    angle = (M_PI * 2) - angle;

  QPointF arrowP1 = intersectPoint - QPointF(sin(angle + M_PI / 3) * arrowSize, cos(angle + M_PI / 3) * arrowSize);
  QPointF arrowP2 =
      intersectPoint - QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize, cos(angle + M_PI - M_PI / 3) * arrowSize);

  arrowHead.clear();

  arrowHead << centerLine.p1() << dst_->pos() << centerLine.p1();
  arrowHead << intersectPoint << arrowP1 << arrowP2 << intersectPoint;

  setPolygon(arrowHead);
}

/*
void
Transition::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{

  if (src_->collidesWithItem(dst_))
    return;


  QPen myPen = pen();
  myPen.setColor(Qt::red);
  myPen.setWidth(5);
  qreal arrowSize = 20;
  painter->setPen(myPen);
  painter->setBrush(Qt::red);

  QPointF intersectPoint;
  intersectPoint = (src_->pos()+dst_->pos())/2;

  QLineF centerLine(src_->pos(), dst_->pos());

  setLine(QLineF(intersectPoint, src_->pos()));

  double angle = ::acos(line().dx() / line().length());
  if (line().dy() >= 0)
    angle = (M_PI * 2) - angle;

  QPointF arrowP1 = line().p1() + QPointF(sin(angle + M_PI / 3) * arrowSize,
      cos(angle + M_PI / 3) * arrowSize);
  QPointF arrowP2 = line().p1() + QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize,
      cos(angle + M_PI - M_PI / 3) * arrowSize);

  arrowHead.clear();
  arrowHead << line().p1() << arrowP1 << arrowP2 <<line().p1()<<dst_->pos();
  painter->drawLine(line());
  painter->drawPolygon(arrowHead);

  QLineF mainline(src_->pos(), dst_->pos());
  setLine(mainline);


}
*/
/*
QRectF Transition::boundingRect() const
{
  qreal extra = (pen().width() + 20) / 2.0;

  return QRectF(line().p1(), QSizeF(line().p2().x() - line().p1().x(),
      line().p2().y() - line().p1().y()))
      .normalized()
      .adjusted(-extra, -extra, extra, extra);
}

QPainterPath Transition::shape() const
{
  QPainterPath path = QGraphicsLineItem::shape();
  return path;
}*/

}  // namespace bica_gui
