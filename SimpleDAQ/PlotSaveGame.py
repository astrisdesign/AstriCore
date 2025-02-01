import os
import time
import tkinter as tk
from tkinter import messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import serial

# Hardcoded serial port settings (adjust as needed)
SERIAL_PORT = "COM3"  # e.g., "COM3" on Windows or "/dev/ttyUSB0" on Linux
BAUD_RATE = 9600

# Open the serial connection once (it remains open until the app terminates)
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


class USBPlotterApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("USB Data Plotter")
        self.user_name = ""
        self.data = []  # list of tuples: (elapsed_time, converted_value)
        self.start_time = None
        self.fig = None
        self.ax = None
        self.line = None
        self.canvas = None
        # Default conversion parameters
        self.tare = 0        # Value to subtract from raw data
        self.gain = 1.0      # Multiplier for the adjusted data
        self.geometry("1000x700") # Controls window size

        self.show_name_prompt()

    def show_name_prompt(self):
        # Clear the window
        for widget in self.winfo_children():
            widget.destroy()

        self.data = []  # reset collected data
        prompt_frame = tk.Frame(self)
        prompt_frame.pack(padx=10, pady=10)

        tk.Label(prompt_frame, text="Enter your name:").pack(pady=(0, 5))
        self.name_entry = tk.Entry(prompt_frame)
        self.name_entry.pack(pady=(0, 10))
        start_btn = tk.Button(prompt_frame, text="Start Run", command=self.start_run)
        start_btn.pack()

    def start_run(self):
        self.user_name = self.name_entry.get().strip()
        if not self.user_name:
            messagebox.showwarning("Input Error", "Please enter a name.")
            return

        # Clear window and initialize plot view
        for widget in self.winfo_children():
            widget.destroy()

        self.init_plot_view()

    def init_plot_view(self):
        self.start_time = time.time()
        self.data = []

        # Create a matplotlib figure and axis
        self.fig, self.ax = plt.subplots(figsize=(5, 4))
        self.line, = self.ax.plot([], [], "b-")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Force (lbf)")
        self.ax.set_title("LIVE HUG FORCE")

        # Embed the matplotlib figure into Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Save Run button
        save_btn = tk.Button(self, text="Save Run", command=self.save_run)
        save_btn.pack(pady=5)

        # Start updating the plot
        self.update_plot()

    def convert_value(self, raw_value):
        """
        Convert the raw integer value by subtracting tare and applying gain.
        """
        return (raw_value - self.tare) * self.gain

    def update_plot(self):
        # Read available serial data without blocking
        while ser.in_waiting:
            try:
                line = ser.readline().decode("utf-8").strip()
                raw_value = int(line)
                # Convert the raw value using tare and gain
                value = self.convert_value(raw_value)
                elapsed = time.time() - self.start_time
                self.data.append((elapsed, value))
            except (ValueError, UnicodeDecodeError):
                # Skip lines that cannot be parsed as integers
                continue

        # Update the plot if new data is available
        if self.data:
            x_vals, y_vals = zip(*self.data)
            self.line.set_data(x_vals, y_vals)
            self.ax.relim()
            self.ax.autoscale_view(True, True, True)
            self.canvas.draw()

        # Schedule the next update (every 100 ms)
        self.after(100, self.update_plot)

    def save_run(self):
        """
        Save the current plot as a PNG file in the "runs" folder.
        If a file with the username already exists, append a counter with zero-fill.
        """
        runs_folder = "runs"
        os.makedirs(runs_folder, exist_ok=True)

        # Base filename using the username
        base_filename = f"{self.user_name}.png"
        file_path = os.path.join(runs_folder, base_filename)
        counter = 1
        while os.path.exists(file_path):
            file_path = os.path.join(runs_folder, f"{self.user_name}_{str(counter).zfill(3)}.png")
            counter += 1

        self.ax.set_title(f"{self.user_name} HUG FORCE")
        self.fig.savefig(file_path, format="png", dpi=250)
        messagebox.showinfo("Run Saved", f"Plot saved to:\n{file_path}")

        # Return to the name-entry prompt
        self.show_name_prompt()


if __name__ == "__main__":
    app = USBPlotterApp()
    app.tare = 3450
    app.gain = -0.00003252718
    app.mainloop()