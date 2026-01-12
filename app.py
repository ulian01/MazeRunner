from flask import Flask, request, render_template, jsonify
import sqlite3
import os
from datetime import datetime
import requests

app = Flask(__name__)

# Local DB is still initialized (kept to minimize changes),
# but live robot data is proxied from your laptop.
DB = "robot_hub.db"

# Set this in Render Environment Variables:
# LAPTOP_API = "https://importance-ambien-relaxation-beer.trycloudflare.com"
LAPTOP_API = os.environ.get("LAPTOP_API")


def get_db():
    return sqlite3.connect(DB)


def init_db():
    """Create the database and tables if they don't exist"""
    db = get_db()

    db.execute("""
        CREATE TABLE IF NOT EXISTS robots (
            robot_id TEXT PRIMARY KEY,
            robot_name TEXT NOT NULL,
            ping_count INTEGER DEFAULT 0,
            last_seen TIMESTAMP,
            last_data TEXT
        )
    """)

    db.execute("""
        CREATE TABLE IF NOT EXISTS robot_logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            robot_id TEXT,
            timestamp TIMESTAMP,
            message TEXT
        )
    """)

    db.commit()
    db.close()


# Initialize database on startup in the container
init_db()


@app.route("/")
def index():
    """
    Homepage:
    - On Render: shows robot data fetched from your laptop API (SQLite on laptop).
    - If laptop API is unreachable, shows empty list (or you can show a message).
    """
    robots = []
    if LAPTOP_API:
        try:
            r = requests.get(f"{LAPTOP_API}/api/robots", timeout=3)
            if r.ok:
                robots = r.json()  # list of dicts
        except Exception:
            robots = []

    return render_template("index.html", robots=robots)


@app.route("/ping", methods=["POST"])
def ping():
    """
    This endpoint remains for compatibility.
    If you're keeping the DB on your laptop, you typically won't use /ping on Render.
    """
    robot_id = request.form.get("robot_id", "R1")
    robot_name = request.form.get("robot_name", "Unknown Robot")
    robot_data = request.form.get("data", "")

    db = get_db()

    db.execute(
        "INSERT OR IGNORE INTO robots (robot_id, robot_name, ping_count, last_seen, last_data) VALUES (?, ?, 0, ?, ?)",
        (robot_id, robot_name, datetime.now(), robot_data)
    )

    db.execute(
        "UPDATE robots SET ping_count = ping_count + 1, last_seen = ?, last_data = ? WHERE robot_id = ?",
        (datetime.now(), robot_data, robot_id)
    )

    db.commit()
    db.close()
    return "PING STORED"


@app.route("/api/robots")
def api_robots():
    """
    Render API:
    Proxies robot data from your laptop API so the frontend can call:
      /api/robots
    without caring where the DB lives.
    """
    if not LAPTOP_API:
        return jsonify({"error": "LAPTOP_API env var not set on Render"}), 500

    try:
        r = requests.get(f"{LAPTOP_API}/api/robots", timeout=3)
        return (r.text, r.status_code, {"Content-Type": "application/json"})
    except Exception as e:
        return jsonify({"error": "Could not reach laptop API", "details": str(e)}), 502


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0")
