# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'startScreen.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from choiceScreen import Ui_choiceScreen


class Ui_Window1(object):

    def openWindow(self):
        self.window=QtWidgets.QMainWindow()
        self.ui=Ui_choiceScreen()
        self.ui.setupUi(self.window)
        self.window.show()
        Window1.hide()

    def setupUi(self, Window1):
        Window1.setObjectName("Window1")
        Window1.resize(575, 466)
        Window1.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(Window1)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(180, 90, 231, 91))
        font = QtGui.QFont()
        font.setFamily("Sitka")
        font.setPointSize(20)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.startButton = QtWidgets.QPushButton(self.centralwidget,clicked= lambda:self.openWindow())
        self.startButton.setGeometry(QtCore.QRect(220, 250, 131, 51))
        font = QtGui.QFont()
        font.setFamily("Sitka")
        font.setPointSize(16)
        font.setBold(False)
        font.setWeight(50)
        self.startButton.setFont(font)
        self.startButton.setStyleSheet("background-color: rgb(215, 228, 253);")
        self.startButton.setObjectName("startButton")
        Window1.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(Window1)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 575, 21))
        self.menubar.setObjectName("menubar")
        Window1.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(Window1)
        self.statusbar.setObjectName("statusbar")
        Window1.setStatusBar(self.statusbar)

        self.retranslateUi(Window1)
        QtCore.QMetaObject.connectSlotsByName(Window1)

    def retranslateUi(self, Window1):
        _translate = QtCore.QCoreApplication.translate
        Window1.setWindowTitle(_translate("Window1", "Robotics Project"))
        self.label.setText(_translate("Window1", "Roboitcs Project"))
        self.startButton.setText(_translate("Window1", "Start"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Window1 = QtWidgets.QMainWindow()
    ui = Ui_Window1()
    ui.setupUi(Window1)
    Window1.show()
    sys.exit(app.exec_())