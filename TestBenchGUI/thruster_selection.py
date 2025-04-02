import sys
from PyQt5.QtWidgets import (
    QApplication, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QDialog, QCheckBox,
)
import copy

class CheckboxInputDialog(QDialog):
    def __init__(self, title="Select thrusters", message="Select options:", parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setGeometry(500, 300, 300, 300)

        layout = QVBoxLayout()

        self.label = QLabel(message)
        layout.addWidget(self.label)

        self.checkboxes = [QCheckBox(f"{i}") for i in range(1, 9)]
        for i in range(8) : layout.addWidget(self.checkboxes[i])

        self.ok_button = QPushButton("OK")
        self.cancel_button = QPushButton("Cancel")
        self.ok_button.clicked.connect(self.accept)
        self.cancel_button.clicked.connect(self.reject)

        layout.addWidget(self.ok_button)
        layout.addWidget(self.cancel_button)

        self.setLayout(layout)

    def get_checked(self):
        return [i for i in range(8) if self.checkboxes[i].isChecked()]

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    dialog = CheckboxInputDialog()
    if dialog.exec_() == QDialog.Accepted:
        print("User selected:", dialog.get_checked())
    else:
        print("User cancelled")