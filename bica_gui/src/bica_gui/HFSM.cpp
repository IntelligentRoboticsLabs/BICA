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
#include <bica_gui/HFSM.h>

#include <string>
#include <list>

namespace bica_gui
{
HFSM::HFSM()
{
}

HFSM::~HFSM()
{
}

bool HFSM::isElement(int x, int y)
{
  return isState(x, y) || isTransition(x, y);
}

bool HFSM::isState(int x, int y)
{
  for (std::list<State>::iterator it = states_.begin(); it != states_.end(); ++it)
    if (it->isInPos(x, y))
      return true;
  return false;
}

bool HFSM::isTransition(int x, int y)
{
  for (std::list<Transition>::iterator it = transitions_.begin(); it != transitions_.end(); ++it)
    if (it->isInPos(x, y))
      return true;
  return false;
}

bool HFSM::existState(const std::string& id)
{
  for (std::list<State>::iterator it = states_.begin(); it != states_.end(); ++it)
    if (it->getId() == id)
      return true;
  return false;
}

void HFSM::addState(std::string id, int x, int y)
{
  std::cerr << "Adding State [" << id << "]" << std::endl;
  State newState(id, x, y);
  states_.push_back(newState);
}

void HFSM::draw(QGraphicsScene& scene)
{
  for (std::list<State>::iterator it = states_.begin(); it != states_.end(); ++it)
  {
    fprintf(stderr, "Drawing State [%s]", it->getId().c_str());
    it->draw(scene);
  }
  for (std::list<Transition>::iterator it = transitions_.begin(); it != transitions_.end(); ++it)
  {
    fprintf(stderr, "Drawing Transition [%s]", it->getId().c_str());
    it->draw(scene);
  }
}

}  // namespace bica_gui
