import os
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QListWidget

class PCDViewer(QWidget):
    def __init__(self, folder_path):
        super(PCDViewer, self).__init__()
        self.folder_path = folder_path
        self.pcl_process = None  # 用于跟踪当前运行的PCL可视化进程
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('PCD Viewer')
        self.setGeometry(300, 300, 800, 600)

        layout = QVBoxLayout(self)

        # 文件列表
        self.list_widget = QListWidget()
        self.list_widget.clicked.connect(self.on_item_clicked)
        layout.addWidget(self.list_widget)

        # 将点云文件列出来
        self.load_pcd_files()

    def load_pcd_files(self):
        for filename in os.listdir(self.folder_path):
            if filename.endswith('.pcd'):
                self.list_widget.addItem(filename)

    def on_item_clicked(self, index):
        file_name = self.list_widget.item(index.row()).text()
        pcd_path = os.path.join(self.folder_path, file_name)

        # 如果有正在运行的PCL可视化进程，先终止它
        if self.pcl_process:
            self.pcl_process.terminate()  # 或使用 self.pcl_process.kill() 强制终止
            self.pcl_process.wait()  # 确保进程已被终止

        # 启动新的PCL可视化进程
        self.pcl_process = subprocess.Popen(["./pclv", pcd_path])

app = QApplication([])
viewer = PCDViewer('..')
viewer.show()
app.exec_()
