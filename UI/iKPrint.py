# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ikPrint.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
import sys
sys.path.insert(0, '../')
from robot import ik

class Ui_ikPrint(object):
    def __init__(self,joints,dhMatrix,poseMatrix):
        ikx=poseMatrix[0]
        ikx=ikx.toPlainText()
        iky=poseMatrix[1]
        iky=iky.toPlainText()
        ikz=poseMatrix[2]
        ikz=ikz.toPlainText()
        ikRoll=poseMatrix[3]
        ikPitch=poseMatrix[4]
        ikYaw=poseMatrix[5]
        self.solution = ik(dhMatrix,joints, x=float(ikx), y=float(iky), z=float (ikz))
#         self.solution = ik(dhMatrix,joints, x=ikx.toPlainText(), y=iky.toPlainText(), z=ikz.toPlainText(),roll=ikRoll.toPlainText(), pitch=ikPitch.toPlainText(), yaw=ikYaw.toPlainText())
        print(self.solution)
    def backToStart(self,mainWindow):
        self.window=QtWidgets.QMainWindow()
        # self.ui=Ui_choiceScreen()
        self.ui.setupUi(self.window)
        self.window.show()
        mainWindow.hide()

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(580, 504)
        MainWindow.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.back = QtWidgets.QPushButton(self.centralwidget,clicked= lambda:self.backToStart(MainWindow))
        self.back.setGeometry(QtCore.QRect(10, 420, 91, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.back.setFont(font)
        self.back.setStyleSheet("background-color: rgb(222, 201, 254);")
        self.back.setObjectName("back")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 599, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Robotics Project"))
        self.back.setText(_translate("MainWindow", "Back"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_ikPrint()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
