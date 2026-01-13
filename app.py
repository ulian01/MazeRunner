# app.py
# This file runs a small Flask web server.
# The web page (index.html) loads from here,
# and this server also acts like a "middle-man":
#   Website  <->  Render Flask app  <->  Laptop API  <->  Robots

from flask import Flask, request, render_template, jsonify  # Flask tools for web routes and JSON
import os                                                    # Lets us read environment variables (like LAPTOP_API)
import requests                                              # Lets us send HTTP requests to your laptop API

# Create the Flask app object
app = Flask(__name__)

# -----------------------------
# CONFIG: Where your laptop API lives
# -----------------------------
# On Render, you set an environment variable called LAPTOP_API.
# Example (your comment):
# LAPTOP_API = "https://importance-ambien-relaxation-beer.trycloudflare.com"
#
# We read that value from the environment so we don't hardcode it in code.
LAPTOP_API = os.environ.get("LAPTOP_API")

# -----------------------------
# ROBOT NAME MAPPING
# -----------------------------
# The robots internally are identified as R1, R2, R3.
# But on the UI, we want friendly names.
ID_MAP = {
    "R1": "Pink Panther",
    "R2": "James Bond",
    "R3": "BDP"
}

# This creates the reverse mapping:
# "Pink Panther" -> "R1", etc.
# We use this when the website says "start Pink Panther" but laptop expects "R1".
NAME_TO_ID = {v: k for k, v in ID_MAP.items()}


# -----------------------------
# ROUTE 1: Home page
# -----------------------------
@app.route("/")
def index():
    """
    When someone goes to the website root ("/"),
    Flask returns the HTML template called index.html.

    Your index.html then runs JavaScript that calls /api/robots repeatedly.
    """
    return render_template("index.html")


# -----------------------------
# ROUTE 2: Robots API for the website
# -----------------------------
@app.route("/api/robots")
def api_robots():
    """
    This endpoint is called by the website (index.html JavaScript).
    It should return a JSON list of robots with fields like:
      - robot_id
      - robot_name
      - ping_count
      - last_seen
      - last_data

    This Flask server DOES NOT talk to the robots directly.
    Instead, it forwards a request to your LAPTOP_API, then returns the result.
    """

    # If LAPTOP_API is missing, we can't contact the laptop at all.
    if not LAPTOP_API:
        return jsonify({"error": "LAPTOP_API env var not set on Render"}), 500

    # Try to request /api/robots from your laptop server
    try:
        # Example request to: https://your-laptop-url/api/robots
        # timeout=4 means: if it takes more than 4 seconds, give up.
        r = requests.get(f"{LAPTOP_API}/api/robots", timeout=4)

        # Convert the response into Python data (dict or list)
        data = r.json()
    except Exception as e:
        # If anything goes wrong (network error, JSON error, timeout),
        # return a helpful error to the website.
        return jsonify({"error": "Could not reach laptop API", "details": str(e)}), 502

    # We'll build a clean list called robots to send back to the web UI.
    robots = []

    # -----------------------------
    # Case 1: Laptop returns a DICTIONARY
    # Example: {"R1": "No data", "R2": "...", "R3": "..."}
    # -----------------------------
    if isinstance(data, dict):
        # We loop through R1/R2/R3 in a fixed order (from ID_MAP),
        # and create a robot object for each one.
        for rid, name in ID_MAP.items():
            # If the laptop dict doesn't contain rid, default to "No data"
            status = data.get(rid, "No data")

            # Append a normalized robot entry in the format the UI expects
            robots.append({
                "robot_id": name,      # We expose friendly name to UI
                "robot_name": name,    # Same as robot_id (UI doesn't need internal ids)
                "ping_count": 0,       # Not available in this dict format
                "last_seen": None,     # Not available in this dict format
                "last_data": status    # The message/status string
            })

        # Return the list as JSON
        return jsonify(robots)

    # -----------------------------
    # Case 2: Laptop returns a LIST of dictionaries already
    # Example: [{"robot_id":"R1", "last_seen":"...", "last_data":"..."}]
    # -----------------------------
    if isinstance(data, list):
        # We'll build a new output list called out
        out = []

        # Loop through each robot object from laptop
        for r in data:
            # Read the internal id (like "R1")
            rid = r.get("robot_id")

            # Convert "R1" -> "Pink Panther" (if possible)
            # If it's not in ID_MAP, just keep rid as-is
            name = ID_MAP.get(rid, rid)

            # Append the normalized object
            out.append({
                "robot_id": name,                    # friendly name
                "robot_name": name,                  # friendly name
                "ping_count": r.get("ping_count", 0),# use laptop value or 0
                "last_seen": r.get("last_seen"),     # pass through
                "last_data": r.get("last_data", "")  # pass through or empty
            })

        # Return the cleaned list
        return jsonify(out)

    # -----------------------------
    # Fallback: Laptop returned something unexpected
    # -----------------------------
    return jsonify([])


# -----------------------------
# ROUTE 3: Command endpoint (START commands, etc.)
# -----------------------------
@app.route("/api/command", methods=["POST"])
def api_command():
    """
    This endpoint is called by your UI buttons (like the "On" button).
    The website sends JSON like:
      { "robot_id": "Pink Panther", "command": "START" }

    This Flask server converts the friendly name to internal id (R1),
    then forwards the command to your laptop API.
    """

    # If LAPTOP_API is missing, we can't forward commands
    if not LAPTOP_API:
        return jsonify({"status": "error", "error": "LAPTOP_API not set"}), 500

    # Read the JSON body from the request
    # force=True means: try to parse JSON even if headers aren't perfect
    data = request.get_json(force=True)

    # Get the robot name the UI sent (example: "Pink Panther")
    ui_robot_name = data.get("robot_id")

    # Get the command the UI sent (example: "START" or "START_R2")
    command = data.get("command")

    # Validate: we need both fields
    if not ui_robot_name or not command:
        return jsonify({"status": "error", "error": "Missing robot_id or command"}), 400

    # Convert UI name -> internal id
    # Example: "Pink Panther" -> "R1"
    rid = NAME_TO_ID.get(ui_robot_name)

    # If the name isn't known, return an error
    if not rid:
        return jsonify({"status": "error", "error": "Unknown robot"}), 400

    # Try to forward the command to the laptop API
    try:
        # Send POST request to: LAPTOP_API/api/command
        # with JSON body: {"robot_id": "R1", "command": "START"}
        r = requests.post(
            f"{LAPTOP_API}/api/command",
            json={"robot_id": rid, "command": command},
            timeout=4
        )

        # If laptop responded with success (HTTP 200-299)
        if r.ok:
            return jsonify({"status": "ok"})
        else:
            # If laptop responded with an error, forward the text for debugging
            return jsonify({"status": "error", "details": r.text}), 500

    except Exception as e:
        # If we couldn't reach laptop, return error
        return jsonify({"status": "error", "details": str(e)}), 502


# -----------------------------
# Run the app locally (not usually used on Render)
# -----------------------------
if __name__ == "__main__":
    # debug=True shows better error pages during development
    # host="0.0.0.0" makes it available on your network (not only localhost)
    app.run(debug=True, host="0.0.0.0")
