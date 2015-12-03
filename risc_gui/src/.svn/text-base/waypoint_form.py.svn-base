from PyQt4.QtCore import *
from PyQt4.QtGui import *

import sys

import waypoint_form_ui


class WaypointsDialog(QDialog, waypoint_form_ui.Ui_waypointDialog):
    def __init__(self, parent=None):
        super(WaypointsDialog, self).__init__(parent)
        self.setupUi(self)
        self.connect(self.graphicsView, SIGNAL("graphicsview_doubleClick"), self.graphicsView_doubleClick)
        self.scene = QGraphicsScene()
        self.scene.width = self.graphicsView.width()
        self.scene.height = self.graphicsView.height()
        self.graphicsView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.graphicsView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.graphicsView.setScene(self.scene)
        self.deleteButton.clicked.connect(self.deleteButton_click)
        self.scene.selectionChanged.connect(self.scene_selectionChanged)
        self.altitudeSlider.valueChanged.connect(self.altitudeSlider_valueChanged)
        self.quad1RadioButton.clicked.connect(self.quad1RadioButton_clicked)
        self.quad2RadioButton.clicked.connect(self.quad2RadioButton_clicked)
        self.quad3RadioButton.clicked.connect(self.quad3RadioButton_clicked)
        self.quad4RadioButton.clicked.connect(self.quad4RadioButton_clicked)

        self.scene.setItemIndexMethod(-1)
        self.waypoints = {}
        self.waypoint_index = 0

# Limit viewing area so that graphics view doesn't move around to center on the scene automoatically
        viewRect = QRectF(-100,-100,200,200)
        self.graphicsView.setSceneRect(viewRect)

        redbrush = QBrush(Qt.red)
        pen = QPen(Qt.black)

#        e = QGraphicsEllipseItem(None, self.scene)
#        e.setBrush(redbrush)
#        e.setPen(pen)
#        e.setRect(0,0,100,100)
#        e.setFlag(QGraphicsItem.ItemIsMovable)

    def quad1RadioButton_clicked(self):
        for item in self.scene.selectedItems():
            self.waypoints[item] = [self.waypoints[item][0], 'quad1', self.waypoints[item][2]]

    def quad2RadioButton_clicked(self):
        for item in self.scene.selectedItems():
            self.waypoints[item] = [self.waypoints[item][0], 'quad2', self.waypoints[item][2]]

    def quad3RadioButton_clicked(self):
        for item in self.scene.selectedItems():
            self.waypoints[item] = [self.waypoints[item][0], 'quad3', self.waypoints[item][2]]

    def quad4RadioButton_clicked(self):
        for item in self.scene.selectedItems():
            self.waypoints[item] = [self.waypoints[item][0], 'quad4', self.waypoints[item][2]]

    def altitudeSlider_valueChanged(self):
        for item in self.scene.selectedItems():
            self.waypoints[item] = [self.waypoints[item][0], self.waypoints[item][1], self.altitudeSlider.value()]

    def scene_selectionChanged(self):
        if self.scene.selectedItems() is not None:
            self.altitudeSlider.setValue(self.waypoints[self.scene.selectedItems()[0]][2])

            if self.waypoints[self.scene.selectedItems()[0]][1] == 'quad1':
                self.quad1RadioButton.setChecked(True)
            elif self.waypoints[self.scene.selectedItems()[0]][1] == 'quad2':
                self.quad2RadioButton.setChecked(True)
            elif self.waypoints[self.scene.selectedItems()[0]][1] == 'quad3':
                self.quad3RadioButton.setChecked(True)
            elif self.waypoints[self.scene.selectedItems()[0]][1] == 'quad4':
                self.quad4RadioButton.setChecked(True)

            #print self.waypoints[self.scene.selectedItems()[0]]
            #print self.waypoints[self.scene.selectedItems()[0]][1]

    def deleteButton_click(self):
        for marker in self.scene.items():
            self.scene.removeItem(marker)

    def update_marker_numberings(self):
        i = 0
        items = self.scene.items()
        for i in range(len(items)):
            if items[i].parentItem() != None:
                items[i].setPlainText("[Marker %s]" % i)


#        for marker in self.scene.items():
#            if marker.parentItem() != None:
#                marker.setPlainText("[Marker %s]" % i)
#                i += 1

    def graphicsView_doubleClick(self, e):
        print e.x(), e.y(), e.button()

        pt = self.graphicsView.mapToScene(e.pos())

        if e.button() == 1: # Left click
           # if self.scene.itemAt(pt) == None:
            rad = 10

            waypointMarker = QGraphicsEllipseItem()
            waypointMarker.setRect(pt.x()-rad, pt.y()-rad, rad*2.0, rad*2.0)

            pen = QPen(Qt.black)
            pen.setWidth(4)
            brush = QBrush(Qt.red)

            waypointMarker.setPen(pen)
            waypointMarker.setBrush(brush)

            self.scene.addItem(waypointMarker)

            waypointMarker.setFlag(QGraphicsItem.ItemIsMovable)
            waypointMarker.setFlag(QGraphicsItem.ItemIsSelectable)

            if self.quad1RadioButton.isChecked():
                self.waypoints[waypointMarker] = [self.waypoint_index, 'quad1', self.altitudeSlider.value()]
            elif self.quad2RadioButton.isChecked():
                self.waypoints[waypointMarker] = [self.waypoint_index, 'quad2', self.altitudeSlider.value()]
            elif self.quad3RadioButton.isChecked():
                self.waypoints[waypointMarker] = [self.waypoint_index, 'quad3', self.altitudeSlider.value()]
            elif self.quad4RadioButton.isChecked():
                self.waypoints[waypointMarker] = [self.waypoint_index, 'quad4', self.altitudeSlider.value()]

            self.waypoint_index += 1


            font = QFont('Courier')
 #           markerText = '[Marker %s]' % (len(self.scene.items()) / 2)
 #           textItem = QGraphicsTextItem(markerText, waypointMarker, self.scene)
 #           textItem.setPos(pt)
 #           textItem.setFont(font)

        elif e.button() == 2: # Right clik..delete marker
            waypointMarker = self.scene.itemAt(pt)

            if waypointMarker != None and waypointMarker.parentItem() == None:
                self.scene.removeItem(waypointMarker)
                self.update_marker_numberings()


    def mousePressEvent(self, event):
        #print "bwaha"
        #print self.waypoints
        pass

#app = QApplication(sys.argv)
#form = WaypointsDialog()
#form.show()
#app.exec_()
