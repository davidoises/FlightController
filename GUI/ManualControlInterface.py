from PyQt4.QtGui import *
from PyQt4.QtCore import *
import sys
from enum import Enum

class Direction(Enum):
    Left = 0
    Right = 1
    Up = 2
    Down = 3

class Joystick(QWidget):
    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(120, 120)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 50

    def paintEvent(self, event):
        painter = QPainter(self)
        bounds = QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(Qt.black)
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())
    
    def _center(self):
        return QPointF(self.width()/2, self.height()/2)


    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if (limitLine.length() > self.__maxDistance):
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def joystickDirection(self):
        if not self.grabCenter:
            return 0
        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)
        if 45 <= angle < 135:
            return (Direction.Up, distance)
        elif 135 <= angle < 225:
            return (Direction.Left, distance)
        elif 225 <= angle < 315:
            return (Direction.Down, distance)
        return (Direction.Right, distance)


    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0)
        self.update()

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            #print("Moving")
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
        #print(self.joystickDirection())

class vertical_slider(QWidget):
   def __init__(self, parent = None):
      super(vertical_slider, self).__init__(parent)

      layout = QVBoxLayout()
      self.l1 = QLabel("YAW")
      self.l1.setAlignment(Qt.AlignCenter)
      layout.addWidget(self.l1)
		
      self.sl = QSlider(Qt.Vertical)
      self.sl.setMinimum(10)
      self.sl.setMaximum(30)
      self.sl.setValue(10)
      self.sl.setTickPosition(QSlider.TicksBelow)
      self.sl.setTickInterval(5)
		
      layout.addWidget(self.sl)
      self.sl.valueChanged.connect(self.valuechange)
      self.setLayout(layout)
      #self.setWindowTitle("SpinBox demo")

   def valuechange(self):
      size = self.sl.value()
      #self.l1.setFont(QFont("Arial",size))

class horizontal_slider(QWidget):
   def __init__(self, parent = None):
      super(horizontal_slider, self).__init__(parent)

      layout = QVBoxLayout()
      self.l1 = QLabel("Thrust")
      self.l1.setAlignment(Qt.AlignCenter)
      layout.addWidget(self.l1)
		
      self.sl = QSlider(Qt.Horizontal)
      self.sl.setMinimum(10)
      self.sl.setMaximum(30)
      self.sl.setValue(10)
      self.sl.setTickPosition(QSlider.TicksBelow)
      self.sl.setTickInterval(5)
		
      layout.addWidget(self.sl)
      self.sl.valueChanged.connect(self.valuechange)
      self.setLayout(layout)
      #self.setWindowTitle("SpinBox demo")

   def valuechange(self):
      size = self.sl.value()
      #self.l1.setFont(QFont("Arial",size))

class on_button(QDialog):
    def __init__(self, parent = None):
        super(on_button, self).__init__(parent)

        layout = QVBoxLayout()
        self.b1 = QPushButton("On")
        #self.b1.setCheckable(True)
        self.b1.setDefault(True)
        self.b1.clicked.connect(self.btnstate)
        
        layout.addWidget(self.b1)
        self.setLayout(layout)
        
    def btnstate(self):
        print("on")

class off_button(QDialog):
    def __init__(self, parent = None):
        super(off_button, self).__init__(parent)

        layout = QVBoxLayout()
        self.b1 = QPushButton("Off")
        self.b1.setCheckable(True)
        self.b1.clicked.connect(self.btnstate)

        self.b1.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding)
        
        layout.addWidget(self.b1,5)
        self.setLayout(layout)
        
    def btnstate(self):
        if self.b1.isChecked():
            print("Paro Activado")
        else:
            print("Paro desactivado")

if __name__ == '__main__':
    # Create main application window
    app = QApplication([])
    app.setStyle(QStyleFactory.create("Cleanlooks"))
    mw = QMainWindow()
    mw.setWindowTitle('GUI Controller')

    # Create and set widget layout
    # Main widget container
    cw = QWidget()
    #cw.setGeometry(1000,100,2000,100)
    ml = QGridLayout()
    cw.setLayout(ml)
    mw.setCentralWidget(cw)

    # Create joystick 
    joystick = Joystick()
    yaw = vertical_slider()
    on = on_button()
    off = off_button()
    thrust = horizontal_slider()
    
    #ml.addLayout(joystick.get_joystick_layout(),0,0)
    ml.addWidget(on,0,0)
    ml.addWidget(yaw,1,0)
    ml.addWidget(thrust,0,1)
    ml.addWidget(joystick,1,1)
    ml.addWidget(off,0,3,3,3)
    
    mw.show()

    ## Start Qt event loop unless running in interactive mode or using pyside.
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QApplication.instance().exec_()
