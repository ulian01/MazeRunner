# We use Flask to create a small web server (an API) that the dashboard can talk to.
from flask import Flask, jsonify, request

# sqlite3 lets us read/write a SQLite database file (connectivity.db).
import sqlite3

# os lets us read environment variables (settings from the system / terminal).
import os

# datetime gives us the current date and time for timestamps in the database.
from datetime import datetime

# pyserial lets Python talk to the Arduino/Hub over USB serial.
# (Install with: pip install pyserial)
import serial

# This is the name/path of the SQLite database file on this laptop.
DB_PATH = "connectivity.db"

# Create the Flask app (this is the web server object).
app = Flask(__name__)

# -------------------------
# SERIAL CONNECTION SETTINGS
# -------------------------

# HUB_PORT is the USB port path where your hub Arduino is connected.
# If HUB_PORT exists as an environment variable, use that.
# Otherwise use the default string below.
#
# Example Mac paths look like:
#   /dev/tty.usbmodemF0F5BD50E6942
#
# This lets you avoid hardcoding the port in code when deploying / changing ports.
HUB_PORT = os.environ.get("HUB_PORT", "/dev/tty.usbmodemF0F5BD50E6942")

# BAUD rate must match Serial.begin(...) in your Arduino code.
BAUD = 9600

# _ser will store the serial connection object (so we reuse it and don't reopen every time).
_ser = None


def get_ser():
    """
    Returns an open serial connection to the hub Arduino.

    Why we do this:
    - Opening a serial port is "expensive"
    - So we keep it open and reuse it (stored in _ser)
    """
    global _ser

    # If we already have a serial connection and it is open, reuse it.
    if _ser and _ser.is_open:
        return _ser

    # Otherwise create a new connection and store it in _ser.
    _ser = serial.Serial(HUB_PORT, BAUD, timeout=1)

    return _ser


def log_to_db(message: str):
    """
    Writes a single message into the database (traffic table).

    traffic table is expected to have columns like:
      id (auto increment), ts (timestamp text), msg (message text)

    We store:
    - ts: current time formatted as YYYY-MM-DD HH:MM:SS
    - msg: the message string passed in
    """

    # Connect to the SQLite database file.
    conn = sqlite3.connect(DB_PATH)

    # Create a "cursor" to run SQL commands.
    cur = conn.cursor()

    # Insert a new row into the traffic table.
    # (ts, msg) are the values we store.
    cur.execute(
        "INSERT INTO traffic (ts, msg) VALUES (?, ?)",
        (datetime.now().strftime("%Y-%m-%d %H:%M:%S"), message)
    )

    # Save changes.
    conn.commit()

    # Close the connection (important so the file isn't locked).
    conn.close()


# -------------------------
# API ENDPOINT: GET /api/robots
# -------------------------

@app.get("/api/robots")
def robots():
    """
    This endpoint is called by the Render server.
    It returns the latest messages for each robot (R1, R2, R3).

    Output format example:
      {
        "R1": "2026-01-13 11:00:10 — motor speed: 120",
        "R2": "No data",
        "R3": "2026-01-13 11:00:14 — Race Finished"
      }

    Important:
    - The Render app turns R1/R2/R3 into names (Pink Panther, etc.)
    """

    # Open DB connection.
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()

    # Get the newest 500 rows from traffic, newest first (ORDER BY id DESC).
    cur.execute("SELECT ts, msg FROM traffic ORDER BY id DESC LIMIT 500")
    rows = cur.fetchall()

    # Close DB connection.
    conn.close()

    # Start with a default dictionary saying we have no data yet.
    latest = {"R1": "No data", "R2": "No data", "R3": "No data"}

    # We loop through newest -> oldest.
    # As soon as we find the first message for a robot, we store it.
    for ts, msg in rows:
        # Example msg might look like:
        #   "R1 motor speed: 123"
        #   "R2 Race Finished"
        parts = msg.strip().split()

        # We need at least 2 words:
        #   parts[0] = "R1"
        #   parts[1:] = actual message content
        #
        # Also:
        # - parts[0] must be one of R1/R2/R3
        # - only fill latest[R1] if it is still "No data"
        if len(parts) >= 2 and parts[0] in latest and latest[parts[0]] == "No data":
            # Build a nice string that includes timestamp + message text.
            latest[parts[0]] = f"{ts} — " + " ".join(parts[1:])

    # Return JSON to the caller.
    return jsonify(latest)


# -------------------------
# API ENDPOINT: POST /api/command
# -------------------------

@app.post("/api/command")
def command():
    """
    This endpoint is called by Render when you click the Start button.

    Expected JSON input:
      {"robot_id":"R2","command":"START_R2"}

    What we do:
    1) Validate robot_id and command
    2) Convert it to a single line like:
         R2:START_R2\n
    3) Send it to the hub via USB serial
    4) Optionally log it in the DB
    """

    # Read JSON body from the request.
    # - force=True: try to parse JSON even if headers aren't perfect
    # - silent=True: if parsing fails, it doesn't crash
    # If parsing fails, we use {} (empty dict).
    data = request.get_json(force=True, silent=True) or {}

    # Pull out robot_id and command, and remove extra spaces.
    robot_id = (data.get("robot_id") or "").strip()
    cmd = (data.get("command") or "").strip()

    # Validate robot_id is one of the allowed values.
    if robot_id not in ("R1", "R2", "R3"):
        return jsonify({"status": "error", "details": "robot_id must be R1/R2/R3"}), 400

    # Validate command exists.
    if not cmd:
        return jsonify({"status": "error", "details": "missing command"}), 400

    # Build the exact line we want to send over serial.
    #
    # Example:
    #   "R2:START_R2\n"
    #
    # The "\n" newline is important because on Arduino you often read until newline:
    #   readStringUntil('\n')
    line = f"{robot_id}:{cmd}\n"

    try:
        # Get the serial connection (opens it if needed).
        ser = get_ser()

        # Send the bytes over USB serial.
        ser.write(line.encode("utf-8"))

    except Exception as e:
        # If serial fails (port wrong, disconnected, permission issue),
        # return error to the caller.
        return jsonify({"status": "error", "details": f"serial write failed: {e}"}), 500

    # OPTIONAL: also write the command into the database so it appears in your logs.
    # This can help debugging (you can see commands in the same place as robot messages).
    try:
        log_to_db(f"{robot_id} CMD {cmd}")
    except Exception:
        # If logging fails, don't crash the API — just ignore it.
        pass

    # If everything went well, return success.
    return jsonify({"status": "ok"})


# -------------------------
# RUN THIS FILE DIRECTLY
# -------------------------

if __name__ == "__main__":
    # Start the Flask server.
    # host="0.0.0.0" means: listen on all network interfaces
    # port=5050 means: the API is available on port 5050
    #
    # Example URL on your laptop:
    #   http://127.0.0.1:5050/api/robots
    app.run(host="0.0.0.0", port=5050)
