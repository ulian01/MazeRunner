from flask import Flask, jsonify
import sqlite3

DB_PATH = "connectivity.db"
app = Flask(__name__)

@app.get("/api/robots")
def robots():
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("SELECT ts, msg FROM traffic ORDER BY id DESC LIMIT 500")
    rows = cur.fetchall()
    conn.close()

    latest = {"R1": "No data", "R2": "No data", "R3": "No data"}

    for ts, msg in rows:
        parts = msg.strip().split()
        if len(parts) >= 2 and parts[0] in latest and latest[parts[0]] == "No data":
            latest[parts[0]] = f"{ts} — " + " ".join(parts[1:])

    return jsonify(latest)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5050)
