"""Drain a live webcam continuously, keeping one recent frame for processing.

Use only for live cameras. File playback must continue reading every frame.
Timestamps describe host read completion, not exposure or sensor-to-screen time.
"""

import threading
import time

import cv2


class LatestFrameCapture:
    def __init__(self, capture):
        self._capture = capture
        self._opened = capture.isOpened()
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._sequence = self._delivered = 0
        self._frame = None
        self._timestamp = self._position_ms = 0.0
        self._max_capture_gap = self._max_read_seconds = 0.0
        self._ended = not self._opened
        self._error = None
        self.last_frame_timestamp = None
        self.last_frame_index = None
        self.dropped_frames = 0
        self._properties = {}
        self._thread = None
        if self._opened:
            # Some backends ignore this; the one-frame slot still bounds the
            # application queue independently of backend support.
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._properties = {p: capture.get(p) for p in (
                cv2.CAP_PROP_FPS, cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT,
            )}
            self._thread = threading.Thread(target=self._run, name="webcam-latest-frame", daemon=True)
            self._thread.start()

    def _run(self):
        try:
            while not self._stop.is_set():
                read_started = time.perf_counter()
                ok, frame = self._capture.read()
                timestamp = time.perf_counter()
                if not ok:
                    break
                position_ms = self._capture.get(cv2.CAP_PROP_POS_MSEC)
                with self._condition:
                    if self._sequence:
                        self._max_capture_gap = max(self._max_capture_gap, timestamp - self._timestamp)
                    self._max_read_seconds = max(self._max_read_seconds, timestamp - read_started)
                    self._frame = frame
                    self._timestamp = timestamp
                    self._position_ms = position_ms
                    self._sequence += 1
                    self._condition.notify_all()
        except Exception as exc:
            self._error = exc
        finally:
            # Release on the same thread as read; avoid closing a backend while
            # another thread is inside its native read operation.
            try:
                self._capture.release()
            finally:
                with self._condition:
                    self._ended = True
                    self._condition.notify_all()

    def isOpened(self):
        return self._opened

    def read(self):
        with self._condition:
            ready = self._condition.wait_for(
                lambda: self._sequence > self._delivered or self._ended or self._stop.is_set(),
                timeout=5,
            )
            if not ready or self._stop.is_set():
                return False, None
            if self._sequence <= self._delivered:
                if self._error is not None:
                    raise OSError("webcam capture failed") from self._error
                return False, None
            self.dropped_frames += self._sequence - self._delivered - 1
            self._delivered = self._sequence
            self.last_frame_index = self._sequence - 1
            self.last_frame_timestamp = self._timestamp
            self._properties[cv2.CAP_PROP_POS_MSEC] = self._position_ms
            return True, self._frame

    def get(self, property_id):
        return self._properties.get(property_id, 0.0)

    def diagnostics(self):
        """Snapshot host read completion, independent of the ROS publisher.

        Maxima cover this capture session, not just the latest report window.
        This does not measure hardware exposure or pre-read buffering.
        """
        with self._condition:
            return dict(
                captured_frames=self._sequence,
                capture_age_ms=(max(0.0, time.perf_counter() - self._timestamp) * 1000
                                if self._sequence else None),
                capture_gap_max_ms=self._max_capture_gap * 1000,
                device_read_max_ms=self._max_read_seconds * 1000,
                skipped_capture_frames=self.dropped_frames,
            )

    def release(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            # A stalled camera may block read. Its daemon worker owns cleanup;
            # never race capture.release() against that blocked native call.
            self._thread.join(timeout=2)
        else:
            self._capture.release()
        self._opened = False
