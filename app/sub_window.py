import cv2
from PyQt5 import QtGui, QtCore, QtWidgets
import threading


class ImageWidget(QtWidgets.QWidget):
    """
    Rewrite Qwidget to display movies/image.
    """
    def __init__(self, parent=None):
        super(ImageWidget, self).__init__(parent)
        Vlayout = QtWidgets.QVBoxLayout()
        self.movie_dir = ""
        self.img_dir = ""
        self.label = QtWidgets.QLabel('', self)
        self.label.setGeometry(QtCore.QRect(0, 0, 984, 783))
        self.label.setMinimumSize(640,400)
        self.label.setScaledContents(True)
        self.videoflag = -1
        self.stop_flag = 1
        Vlayout.addWidget(self.label)
        self.setLayout(Vlayout)
    
    def SetFrameFromAddr(self, img_dir):
        self.frame = cv2.imread(self.img_dir, 1)
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
    
    def SetFrame(self, img):
        self.frame = img

    def ShowImg(self):
        try:
            img = QtGui.QImage(self.frame.data, self.frame.shape[1], self.frame.shape[0], QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(img)
            self.label.setPixmap(pixmap)
        except Exception as e:
            print(e)

    def ShowMovie(self):
        try:
            self.cap = cv2.VideoCapture(self.movie_dir)
            self.frameRate = self.cap.get(cv2.CAP_PROP_FPS)
            self.videoflag = 1
            thu = threading.Thread(target=self.Display)
            thu.start()
        except Exception as e:
            print(e)

    def Display(self):
        while self.videoflag == 1:
            if self.cap.isOpened():
                self.stop_flag = 0
                success, frame = self.cap.read()
                self.picture = frame
                # RGB转BGR
                if success:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    frame = cv2.resize(frame, (self.label.width(),self.label.height()))
                    img = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
                    pixmap = QtGui.QPixmap.fromImage(img)
                    self.label.setPixmap(pixmap)
                    time.sleep(1/self.frameRate)
                else:
                    #print("read failed, no frame data")
                     pass
            else:
                print("open file or capturing device error, init again")
                self.reset()
        self.stop_flag = 1
        self.cap.release()


    def stop(self):
        """ Slot function to stop the movie. """
        self.videoflag = -1
