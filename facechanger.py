#!/usr/bin/env python3
import tkinter as tk
import subprocess

HOST = "192.168.1.2"  # Host IP
PORT = "9999"  # Port for VLC RC interface

# List of videos to choose from (corresponding to numbers 1-10)
videos = [f"tc_bot_1/faces/{i}.mp4" for i in range(1, 11)]

# Function to send commands via nc (Netcat)
def send_command(command):
    try:
        subprocess.run(f'echo "{command}" | nc -q 1 {HOST} {PORT}', shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error sending command: {e}")

# Function to handle button click and play the selected video
def play_video(video_index):
    selected_video = videos[video_index]  # Get the video file corresponding to the selected number
    # Stop current video, clear the playlist, and play the selected video
    send_command("stop")
    send_command("clear")
    send_command(f"add {selected_video}")
    send_command("play")

# Create the main window
root = tk.Tk()
root.title("Face Changer")  # Set the window title

# Create a grid of buttons (1-10)
for i in range(10):
    # Each button is labeled with the number (1-10)
    button = tk.Button(root, text=str(i + 1), width=3, height=1, 
                       command=lambda i=i: play_video(i))  # Use lambda to pass the index
    button.grid(row=i // 5, column=i % 5, padx=2, pady=2)  # Reduced padding for smaller gaps

# Configure row and column weights to adjust the window size automatically
for i in range(2):  # Two rows (0, 1)
    root.grid_rowconfigure(i, weight=1)
for i in range(5):  # Five columns (0-4)
    root.grid_columnconfigure(i, weight=1)

# Run the GUI
root.mainloop()
