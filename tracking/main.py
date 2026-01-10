import flask
import threading
import time

import cv2

from camera import Tracker

app = flask.Flask(__name__)


_tracker_lock = threading.Lock()
_tracker = Tracker()


@app.get("/position")
def camera_position():
    with _tracker_lock:
        pos = _tracker.get_position()

    return flask.jsonify(pos)


@app.get("/image")
def camera_image():
    with _tracker_lock:
        frame = _tracker.get_image()

    if frame is None:
        return flask.jsonify(
            {"error": "no image available; call POST /camera/update first"}
        ), 404

    ok, buf = cv2.imencode(".jpg", frame)
    if not ok:
        return flask.jsonify({"error": "failed to encode image"}), 500

    return flask.Response(buf.tobytes(), mimetype="image/jpeg")


def do_tracker_thread():
    while True:
        with _tracker_lock:
            _tracker.update()
        time.sleep(0.01)


def main():
    tracker_thread = threading.Thread(None, do_tracker_thread)
    tracker_thread.start()
    app.run()


if __name__ == "__main__":
    main()
