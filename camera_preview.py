import argparse
import threading
import time

import cv2


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=true max-buffers=1 sync=false"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


class ThreadedCamera:
    def __init__(self, pipeline):
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        self.frame = None
        self.ok = False
        self.running = False
        self.lock = threading.Lock()
        self.thread = None

    def start(self):
        if not self.cap.isOpened():
            raise RuntimeError("Could not open CSI camera. Check ribbon cable, camera enablement, and nvargus-daemon.")
        self.running = True
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True
        self.thread.start()

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            with self.lock:
                self.ok = ok
                if ok:
                    self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ok, self.frame.copy()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.cap.release()


def build_parser():
    parser = argparse.ArgumentParser(description="Threaded Jetson CSI/Raspberry camera preview")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--display-width", type=int, default=960)
    parser.add_argument("--display-height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", type=int, default=0, help="0 none, 2 rotate 180, other nvvidconv flip-method values allowed")
    parser.add_argument("--snapshot", default=None, help="Save one frame to this path and exit")
    parser.add_argument("--no-window", action="store_true", help="Run without preview window; useful over SSH")
    return parser


def main():
    args = build_parser().parse_args()
    pipeline = gstreamer_pipeline(
        sensor_id=args.sensor_id,
        capture_width=args.width,
        capture_height=args.height,
        display_width=args.display_width,
        display_height=args.display_height,
        framerate=args.fps,
        flip_method=args.flip,
    )
    print("Opening CSI camera...")
    camera = ThreadedCamera(pipeline)
    camera.start()

    try:
        # Let the sensor warm up and the capture thread receive its first frames.
        time.sleep(0.5)
        if args.snapshot:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Camera opened, but no frame was received.")
            cv2.imwrite(args.snapshot, frame)
            print("Saved snapshot:", args.snapshot)
            return

        if args.no_window:
            print("Camera is running. Press Ctrl+C to stop.")
            while True:
                ok, _ = camera.read()
                if ok:
                    print("frame ok")
                    time.sleep(1.0)
                else:
                    print("waiting for frame")
                    time.sleep(0.2)

        print("Preview running. Press Esc or q to quit.")
        while True:
            ok, frame = camera.read()
            if not ok:
                time.sleep(0.01)
                continue
            cv2.imshow("CSI camera", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
