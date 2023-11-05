#
# First prototype by Sungkyung-Shon (https://github.com/Sungkyung-Shon/)
#
#
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QPushButton, QLabel, QSlider, QSpinBox
from PyQt5.QtCore import Qt, QUrl, QFileInfo
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget
import json
import os
from PyQt5.QtWidgets import QLineEdit, QLabel


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Please score the robot's behavior")

        self.current_video = 0
        directory = 'episode_recordings/'
        self.videos = []
        for filename in os.listdir(directory):
            f = os.path.join(directory, filename)
            if os.path.isfile(f) and filename.endswith('.mp4'):
                print(f)
                self.videos.append(f)

        if os.path.exists('ratings.json'):
            with open('ratings.json', 'r') as json_file:
                self.ratings = json.load(json_file)
        else:
            self.ratings = {video: 0 for video in self.videos}

        for video in self.videos:
            if video not in self.ratings:
                self.ratings[video] = 0

        self.layout = QVBoxLayout(self)

        self.player = QMediaPlayer()
        self.video_widget = QVideoWidget()
        self.layout.addWidget(self.video_widget)

        self.player.setVideoOutput(self.video_widget)

        self.rating_slider = QSlider(Qt.Horizontal)
        self.rating_slider.setRange(0, 100)
        self.layout.addWidget(self.rating_slider)

        self.rating_spinbox = QSpinBox()
        self.rating_spinbox.setRange(0, 100)
        self.layout.addWidget(self.rating_spinbox)
        self.rating_slider.valueChanged.connect(self.rating_spinbox.setValue)

        self.rating_spinbox.valueChanged.connect(self.rating_slider.setValue)

        self.pause_play_button = QPushButton("Pause")
        self.pause_play_button.clicked.connect(self.toggle_pause_play)
        self.layout.addWidget(self.pause_play_button)

        self.prev_button = QPushButton("Previous Video")
        self.prev_button.clicked.connect(self.prev_video)
        self.layout.addWidget(self.prev_button)

        self.replay_button = QPushButton("Replay")
        self.replay_button.clicked.connect(self.play_video)
        self.layout.addWidget(self.replay_button)

        self.next_button = QPushButton("Next Video")
        self.next_button.clicked.connect(self.next_video)
        self.layout.addWidget(self.next_button)

        self.current_video_label = QLabel(f"Current Video: {self.current_video + 1}/{len(self.videos)}")
        self.layout.addWidget(self.current_video_label)

        # self.goto_video_input = QLineEdit()
        # self.goto_video_input.setPlaceholderText("Enter video number")
        # self.layout.addWidget(self.goto_video_input)

        # self.goto_video_button = QPushButton("Go to")
        # self.goto_video_button.clicked.connect(self.goto_video)
        # self.layout.addWidget(self.goto_video_button)

        self.quit_button = QPushButton("Quit")
        self.quit_button.clicked.connect(self.my_quit)
        self.layout.addWidget(self.quit_button)

        self.setLayout(self.layout)

        self.player.mediaStatusChanged.connect(self.handle_media_status)

        self.setWindowState(Qt.WindowMaximized)
        self.play_video()

    def handle_media_status(self, status):
        if status == QMediaPlayer.EndOfMedia:
            self.next_button.setEnabled(True)
            self.rating_slider.setEnabled(True)
            self.rating_spinbox.setEnabled(True)
            
    def play_video(self):
        print(self.current_video)
        self.player.setMedia(QMediaContent(QUrl.fromLocalFile(QFileInfo(self.videos[self.current_video]).absoluteFilePath())))
        self.player.setMuted(True)
        self.player.play()
        self.next_button.setEnabled(False)
        self.rating_slider.setEnabled(False)
        self.rating_spinbox.setEnabled(False)
        self.current_video_label.setText(f"Current Video: {self.current_video + 1}/{len(self.videos)}")

    def toggle_pause_play(self):
        if self.player.state() == QMediaPlayer.PlayingState:
            self.player.pause()
            self.pause_play_button.setText("Play")
        else:
            self.player.play()
            self.pause_play_button.setText("Pause")

    def next_video(self):
        self.ratings[self.videos[self.current_video]] = self.rating_slider.value()

        self.save_ratings()

        if self.current_video + 1 < len(self.videos):
            self.current_video += 1
            self.rating_slider.setValue(1)
            self.rating_spinbox.setValue(1)
            self.play_video()
        else:
            print("All videos have been rated. Ratings:", self.ratings)
            self.player.stop()

    def save_ratings(self):
        with open('ratings.json', 'w') as json_file:
            json.dump(self.ratings, json_file)
        print("Ratings saved.")

    def prev_video(self):
        if self.current_video > 0:
            self.current_video -= 1
            self.rating_slider.setValue(self.ratings[self.videos[self.current_video]])
            self.rating_spinbox.setValue(self.ratings[self.videos[self.current_video]])
            self.play_video()
            self.save_rantings()
    
    def my_quit(self):
        self.save_ratings()
        self.close()


    # def goto_video(self):
    #     video_number = self.goto_video_input.text()
    #     if video_number.isdigit():
    #         video_number = int(video_number) - 1  # Adjust to zero-indexed list
    #         if 0 <= video_number < len(self.videos):
    #             self.current_video = video_number
    #             self.play_video()
    #         else:
    #             print(f"Invalid video number. Please enter a number between 1 and {len(self.videos)}")
    #     else:
    #         print("Invalid input. Please enter a numeric value.")

app = QApplication([])

window = MainWindow()
window.show()

app.exec_()
