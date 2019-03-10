#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Intelligent Robotics Core S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Francisco Martín Rico - fmrico@gmail.com


from __future__ import division
import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget
from python_qt_binding.QtSvg import QSvgGenerator

from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_dotgraph.pydotfactory import PydotFactory
from rqt_gui_py.plugin import Plugin


from .dotcode import BicaGraphDotcodeGenerator
from .interactive_graphics_view import InteractiveGraphicsView
from .bica_graph_impl import BicaGraphImpl

from PyQt5 import QtGui, QtCore

try:
    unicode
    # we're on python2, or the "unicode" function has already been defined elsewhere
except NameError:
    unicode = str
    # we're on python3


class BicaGraph(Plugin):

    _deferred_fit_in_view = Signal()

    def __init__(self, context):
        super(BicaGraph, self).__init__(context)
        self.initialized = False
        self.setObjectName('Bica Graph')

        self._graph = BicaGraphImpl()
        self._current_dotcode = None

        self._widget = QWidget()

        self.dotcode_factory = PydotFactory()
        self.dotcode_generator = BicaGraphDotcodeGenerator()
        self.dot_to_qt = DotToQtGenerator()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bica_rqt_graph'), 'resource', 'BicaGraph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('BicaGraphUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_graph_push_button.pressed.connect(self._update_bicagraph)

        self._widget.save_as_svg_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.save_as_svg_push_button.pressed.connect(self._save_svg)
        self._widget.save_as_image_push_button.setIcon(QIcon.fromTheme('image'))
        self._widget.save_as_image_push_button.pressed.connect(self._save_image)

        self._update_bicagraph()
        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        context.add_widget(self._widget)

        self._updateTimer = QtCore.QTimer()
        self._updateTimer.timeout.connect(self.do_update)
        self._updateTimer.start(1000)

    def do_update(self):
        self._update_bicagraph()

        self._updateTimer = QtCore.QTimer()
        self._updateTimer.timeout.connect(self.do_update)
        self._updateTimer.start(1000)


    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        self.initialized = True
        #self._refresh_bicagraph()

    def _update_bicagraph(self):
        #self._graph = rosgraph.impl.graph.Graph()
        #self._graph.update()

        if self._graph.ready:
            self._refresh_bicagraph()

    def _refresh_bicagraph(self):
        if not self.initialized:
            return
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        return self.dotcode_generator.generate_dotcode(
            bicagraphinst=self._graph,
            dotcode_factory=self.dotcode_factory)

    def _update_graph_view(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()

    def _generate_tool_tip(self, url):
        return url

    def _redraw_graph_view(self):
        self._scene.clear()

        # layout graph and create qt items
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level=3,
                                                            same_label_siblings=True,
                                                            scene=self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())
        self._fit_in_view()

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_svg(self):
        file_name, _ = QFileDialog.getSaveFileName(
            self._widget, self.tr('Save as SVG'), 'bicagraph.svg', self.tr('Scalable Vector Graphic (*.svg)'))
        if file_name is None or file_name == '':
            return

        generator = QSvgGenerator()
        generator.setFileName(file_name)
        generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

        painter = QPainter(generator)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()

    def _save_image(self):
        file_name, _ = QFileDialog.getSaveFileName(
            self._widget, self.tr('Save as image'), 'bicagraph.png', self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0)
                     .toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)
