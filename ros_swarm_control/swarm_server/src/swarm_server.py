#!/usr/bin/env python
# coding: utf-8

import sys
import rospy

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import QObject, pyqtSignal,  QRunnable, QThread, QThreadPool, pyqtSlot, Qt, QTimer
import DroneUI, window
from DroneClient import DroneConnect
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, String
import json
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

mode_list = ("OFFBOARD", "STABILIZED", "AUTO.LAND", "POSCTL")
common_ip = "127.0.0.1"
common_port = 9090
robotsList = list()

tag = "drone_"

origin_topic = '/geo/set_origin'
origin_pose = NavSatFix()
abortFlag = False


def send_receive_signal():
    "Sending a empty message"
    msgs = Empty()
    for drone in robotsList:
        if drone.is_active():
            drone.send_receive_signal("drone/receive", String())

class WindowApp(QtWidgets.QMainWindow, window.Ui_Form):
    def __init__(self):
        # Это здесь нужно для доступа к переменным, методам
        # и т.д. в файле design.py
        super(WindowApp, self).__init__()

        self.setupUi(self)  # Это нужно для инициализации нашего дизайна

        # init signals
        self.addButton.clicked.connect(self.addItems)
        self.dellButton.clicked.connect(self.delItems)
        self.ArmAllButton.clicked.connect(self.ArmAll)
        self.DisarmAllButton.clicked.connect(self.DisarmAll)
        self.ConnectAllButton.clicked.connect(self.ConnectAll)
        self.DisconnectAllButton.clicked.connect(self.DisconnectAll)

        self.ModeButton_OFFBOARD.clicked.connect(self.setModeOffboard)
        self.ModeButton_STAB.clicked.connect(self.setModeStab)
        self.ModeButton_LAND.clicked.connect(self.setModeLand)

        self.OriginPushButton.clicked.connect(self.setOrigin)
        self.LatSpinBox.valueChanged.connect(self._changeOrigin)
        self.LonSpinBox.valueChanged.connect(self._changeOrigin)
        self.AltSpinBox.valueChanged.connect(self._changeOrigin)

        self.SaveParamsButton.clicked.connect(self._save_params)
        self.LoadParamsButton.clicked.connect(self._load_params)

    def _load_params(self):
        path = self.open_dialog()
        if path == "":
            return

        #read data from files
        file = open(path, "r")
        json_data = file.read()
        file.close()
        print(json_data)

        # set origin value
        lat = json.loads(json_data)['origin']['lat']
        lon = json.loads(json_data)['origin']['lon']
        alt = json.loads(json_data)['origin']['alt']

        self.LatSpinBox.setValue(lat)
        self.LonSpinBox.setValue(lon)
        self.AltSpinBox.setValue(alt)

        #clear list of drone
        self.delListOfDrone()
        size = json.loads(json_data)['size']

        for i in range(size):
            name = json.loads(json_data)[tag+str(i)]['name']
            ip = json.loads(json_data)[tag+str(i)]['ip']
            port = json.loads(json_data)[tag + str(i)]['port']

            droneClient = DroneConnect(ip, port, name)
            robotsList.append(droneClient)
            item_widget = DroneUI.Ui("%s%s" %(tag,i), droneClient)
            self.item = QtWidgets.QListWidgetItem(self.listOfDrones)
            self.item.setSizeHint(item_widget.sizeHint())
            self.listOfDrones.addItem(self.item)
            self.listOfDrones.setItemWidget(self.item, item_widget)

    def _save_params(self):
        global robotsList, origin_pose
        path = self.save_dialog()
        if path == "":
            return
        origin = {}
        origin['lat'] = origin_pose.latitude
        origin['lon'] = origin_pose.longitude
        origin['alt'] = origin_pose.altitude

        doc = {}
        doc["origin"] = origin
        doc["size"] = len(robotsList)

        for i in range(len(robotsList)):
            drone = {}
            drone['name'] = robotsList[i].name
            drone['ip'] = robotsList[i].ws._ip
            drone['port'] = robotsList[i].ws._port
            doc[str(tag)+str(i)] = drone

        json_data = json.dumps(doc)
        file = open(path, "w")
        file.write(json_data)
        file.close()
        print("Params save to: %s" %path)

    def save_dialog(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, param = QtWidgets.QFileDialog.getSaveFileName(self, "Save of drone list", "",
                                                                "Drone params (*.params);;All Files (*)", options=options)
        filter = ''
        if param.find('Drone params (*.params)') != -1 :
            filter =".params"

        return fileName+filter

    def open_dialog(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName = ""
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self, "QFileDialog.getOpenFileName()", "",
                                                            "Drone params (*.params);;All Files (*)", options=options)
        return fileName

    def _changeOrigin(self):
        """
        change origin data
        :return:
        """
        global origin_pose
        Lat = self.LatSpinBox.value()
        Lon = self.LonSpinBox.value()
        Alt = self.AltSpinBox.value()

        origin_pose.latitude = Lat
        origin_pose.longitude = Lon
        origin_pose.altitude = Alt
        # print("=======\n"
        #       "Lat: %s\n"
        #       "Lon: %s\n"
        #       "Alt: %s" % (Lat, Lon, Alt))

    def setOrigin(self):
        """
        push button set origin data
        :return:
        """
        global origin_pose
        print("===========\n"
              "Set origin\n"
              "===========\n"
              "Lat: %s\n"
              "Lon: %s\n"
              "Alt: %s" % (origin_pose.latitude,
                           origin_pose.longitude,
                           origin_pose.altitude))

        origin_pose.header.stamp = rospy.Time.now()
        for drone in robotsList:
            if drone.is_active():
                drone.set_origin(origin_topic, origin_pose)

    def ConnectAll(self):
        print("connect all")
        for drone in robotsList:
            if not drone.is_active():
                drone.connect()

    def DisconnectAll(self):
        print("disconnect all")

        for drone in robotsList:
            if drone.is_active():
                drone.disconnect()

    def ArmAll(self):
        print("ArmAll")
        for drone in robotsList:
            drone.arm()

    def DisarmAll(self):
        print("disarm all")
        global robotsList

        for drone in robotsList:
            drone.disarm()

    def setAllMode(self, mode):
        print("set mode all: %s" %mode)
        global robotsList

        for i in range(self.listOfDrones.count()):
            drone = self.listOfDrones.itemWidget(self.listOfDrones.item(i))
            drone.setMode(mode)
            drone.changeComboBox(mode)

    def setModeOffboard(self):
        self.setAllMode("OFFBOARD")

    def setModeStab(self):
        self.setAllMode("STABILIZED")

    def setModeLand(self):
        self.setAllMode("AUTO.LAND")


    def addItems(self):
        """
        :type robotsList: list
        """
        global robotsList

        droneClient = DroneConnect(common_ip, common_port, "%s%s" % (tag, len(robotsList)))
        robotsList.append(droneClient)

        item_widget = DroneUI.Ui("%s%s" % (tag, len(self.listOfDrones)), droneClient)
        self.item = QtWidgets.QListWidgetItem(self.listOfDrones)
        self.item.setSizeHint(item_widget.sizeHint())
        self.listOfDrones.addItem(self.item)
        self.listOfDrones.setItemWidget(self.item, item_widget)
        print("addItems:",len(self.listOfDrones))

    def delItems(self):
        global robotsList

        if self.listOfDrones.currentRow() < 0:
            return
        robotsList[self.listOfDrones.currentRow()].disconnect()
        del robotsList[self.listOfDrones.currentRow()]
        self.listOfDrones.takeItem(self.listOfDrones.currentRow())
        print("del: %d : len: %d" %(self.listOfDrones.currentRow(), len(robotsList)))

    def delListOfDrone(self):
        global robotsList

        if len(robotsList) == 0:
            print("List is empty:", len(robotsList))
            return

        for i in range(len(robotsList)-1,-1,-1):
            robotsList[i].disconnect()
            del robotsList[i]
            self.listOfDrones.takeItem(i)
            print("del: %d : len: %d" % (i, len(robotsList)))
        print("List is clear:")

class ROS_run(QObject):
    # def __init__(self):
    #     super(ROS_run, self).__init_()
    #     print("thead ros start")

    @pyqtSlot()
    def run(self):
        rate = rospy.Rate(2)
        try:
            while not rospy.is_shutdown() and not abortFlag:
                send_receive_signal()
                rate.sleep()
            app.exit()
        except rospy.ROSInterruptException:
            print "ROSInterruptException EXIT"
            app.exit()


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    global abortFlag
    sys.stderr.write('\r')
    abortFlag = True
    app.quit()

abortFlag = False

if __name__ == '__main__':
    rospy.init_node('swarm_server_node', anonymous=True)
    signal.signal(signal.SIGINT, sigint_handler)

    # run ros thread
    ros_t = QThread()
    ros_ = ROS_run()
    ros_.moveToThread(ros_t)
    ros_t.started.connect(ros_.run)
    ros_t.start()

    # run main
    app = QtWidgets.QApplication(sys.argv)  # Новый экземпляр QApplication

    timer = QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    window = WindowApp()  # Создаём объект класса ExampleApp
    window.show()  # Показываем окно
    sys.exit(app.exec_())
    abortFlag = True

