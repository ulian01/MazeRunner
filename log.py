# We import three libraries:
# - sqlite3: lets us write data to a SQLite database file
# - time: lets us get the current date and time
# - serial: lets Python talk to Arduino / hub over USB (pyserial)
import sqlite3, time, serial

# This is the USB port where the hub Arduino is connected.
# On macOS it usually looks like: /dev/tty.usbmodemXXXXXXXX
# ⚠️ If you unplug and replug the Arduino, this name might change.
PORT = "/dev/tty.usbmodemF0F5BD50E6942"  # change this to your actual port

# Open a serial connection to the hub Arduino:
# - PORT = which USB device to use
# - 9600 = must match Serial.begin(9600) in Arduino code
# - timeout=1 = wait at most 1 second when trying to read data
ser = serial.Serial(PORT, 9600, timeout=1)

# Open (or create if it doesn't exist) the SQLite database file.
# This file will store all messages from the robots.
db = sqlite3.connect("connectivity.db")

# This is an infinite loop.
# It will run forever until you stop the program (Ctrl+C).
while True:

    # Read one line from the serial port:
    # - ser.readline() reads until it sees '\n' (newline)
    # - .decode(...) converts bytes to text
    # - errors="ignore" means: if weird characters appear, skip them
    # - .strip() removes spaces and \n \r from the start/end
    m = ser.readline().decode(errors="ignore").strip()

    # If we actually received something (not empty):
    if m:
        # Insert a new row into the traffic table:
        # - ts = current time (YYYY-MM-DD HH:MM:SS)
        # - msg = the message that came from the robot/hub
        db.execute(
            "INSERT INTO traffic(ts, msg) VALUES(?, ?)",
            (time.strftime("%F %T"), m)
        )

        # Save the changes to the database file.
        # Without this, the data might not actually be written to disk.
        db.commit()

        # Also print the message to the terminal so you can see it live.
        print(m)
