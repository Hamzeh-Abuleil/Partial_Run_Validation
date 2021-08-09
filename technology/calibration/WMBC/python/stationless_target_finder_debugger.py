#!/usr/bin/python

from PyQt4 import QtGui, QtCore

from base.utils import current_context as cc, warn
from plugins.abstract_mest_plugin import AbstractMestPlugin, remote

import mest.types


class stationless_target_finder_debugger(AbstractMestPlugin):
    
    _pen = QtGui.QPen(QtCore.Qt.red, 1)
    
    def get_registered_categories(self):
        return [('WMBC', 'SLC_TFinder')]
    
    def get_objects_for_frame(self, frame_descriptor, frame_data, current_width, current_height):        
        '''
        This method is called when a frame was changed or mouse/keyboard
        events was handled by plugins.
        Its role is to provide list of QGraphicsItem to the relevant view
        so they will be shown above the view.
        '''

        items = []
        
        #self_frame = frame_descriptor.relative_root
        
        frame_data = self._reader.get_frame_data(frame_descriptor, 'WMBC', 'SLC_TFinder')
        print frame_data
        
        if frame_data:
            x = frame_data.get('x')
            
            an_int_rect = frame_data.get('an_int_rect')
            origin_coord = frame_data.get('origin_coord')
            size = frame_data.get('size')
            image = frame_data.get('image')
            
            if image:
                qimg = image[-1][0].qimage
                pixmap_item = QtGui.QGraphicsPixmapItem()
                pixmap_item.setPixmap(QtGui.QPixmap.fromImage(qimg))
                
                # Put the image below other items (in case 
                pixmap_item.setZValue(id(self)-1)
                items.append(pixmap_item)
    
            if x:
                x = x[-1][0]
                self.__class__._x_text_line.setText(x)
                
                
        return items
    
    
    # def _call_mest(self):
    #     #self.call_mest_func.emit('cackle1')
    #     self.call_mest_func.emit('cackle2')
    
    
    # def get_widget(self):
    #     # demo - create some widgets, pass to main...

    #     widget = QtGui.QWidget()
    #     layout = QtGui.QVBoxLayout()
    #     widget.setLayout(layout)

    #     gogo = self.__class__._gogo_text_line = QtGui.QTextEdit("", widget)
    #     gogo.setReadOnly(True)
    #     layout.addWidget(gogo)

    #     btn = self.__class__._btn = QtGui.QPushButton('&Run method in MEST', widget)
    #     layout.addWidget(btn)
    #     btn.clicked.connect(self._call_mest)
        
    #     return widget
    

