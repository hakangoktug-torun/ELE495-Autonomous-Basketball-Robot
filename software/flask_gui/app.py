from flask import Flask, jsonify, render_template, request

from software.raspberry_pi.config import LIVE_CONFIG, SIMULATION_CONFIG
from software.raspberry_pi.robot import RobotController, RobotMode

app = Flask(__name__)

# Swap which line is active to switch between simulated wheels and real ones.
# Nothing else in this file, robot.py, or the GUI needs to change.
robot = RobotController(SIMULATION_CONFIG)
# robot = RobotController(LIVE_CONFIG)


@app.route("/")
def home():
    return render_template("index.html")


@app.get("/api/status")
def status():
    return jsonify(robot.status_dict())


@app.post("/api/mode")
def set_mode():
    payload = request.get_json(silent=True) or {}
    mode = payload.get("mode", RobotMode.IDLE.value)
    robot.set_mode(mode)
    return jsonify(robot.status_dict())


@app.post("/api/manual")
def manual_drive():
    payload = request.get_json(silent=True) or {}
    left = float(payload.get("left", 0.0))
    right = float(payload.get("right", 0.0))
    robot.manual_drive(left, right)
    return jsonify(robot.status_dict())


@app.post("/api/autonomous-step")
def autonomous_step():
    robot.autonomous_step()
    return jsonify(robot.status_dict())


@app.post("/api/shoot")
def shoot():
    robot.shoot()
    return jsonify(robot.status_dict())


@app.post("/api/score")
def add_score():
    payload = request.get_json(silent=True) or {}
    points = int(payload.get("points", 0))
    robot.add_score(points)
    return jsonify(robot.status_dict())


@app.post("/api/score/reset")
def reset_score():
    robot.reset_score()
    return jsonify(robot.status_dict())


@app.post("/api/emergency-stop")
def emergency_stop():
    robot.emergency_stop()
    return jsonify(robot.status_dict())


@app.post("/api/reset")
def reset():
    robot.reset_emergency()
    return jsonify(robot.status_dict())


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
    finally:
        robot.close()