"""One bounded, low-priority display worker. No ROS, model or motor commands."""
from dataclasses import asdict
import multiprocessing
import os
import threading
import time


def _reference_process(connection, configuration):
    import cv2
    from road_navigation import DetectorConfig, RoadDetector
    cv2.setNumThreads(1)
    try:
        os.nice(10)
    except (AttributeError, OSError):
        pass
    detector = RoadDetector(DetectorConfig(**configuration))
    previous = None
    try:
        while True:
            job = connection.recv()
            if job is None:
                break
            generation, key, frame, probability, processing_ms = job
            try:
                if previous is not None and (generation != previous[0] or key[0] != previous[1][0]
                                             or key[1:] < previous[1][1:]):
                    detector.reset()
                result = detector.process(frame, drivable_probability=probability)
                result.backend = "segformer"
                result.processing_ms += processing_ms
                connection.send((generation,key,result,None))
                previous = (generation,key)
            except Exception as exc:
                detector.reset()
                connection.send((generation,key,None,type(exc).__name__+": "+str(exc)[:160]))
    except (EOFError, BrokenPipeError, OSError):
        pass
    finally:
        connection.close()


class ReferencePreview:
    """Inference only replaces a slot; IPC and display planning run elsewhere.

    At most one in flight, one pending, one completed result. No unbounded
    backlog, repeated inference, ROS initialization or stale image/path mixing.
    """
    def __init__(self, configuration, max_fps=2.):
        if not 0 < max_fps <= 10:
            raise ValueError("reference preview FPS must be >0 and <=10")
        self._configuration = asdict(configuration)
        self._interval = 1/max_fps
        self._lock = threading.Lock()
        self._wake, self._stop = threading.Event(), threading.Event()
        self._pending = self._result = self._thread = self._process = None
        self._generation = 0
        self._stats = dict(submitted=0,replaced=0,completed=0,error="")

    def submit(self, key, frame, probability, processing_ms):
        # Starting is lazy: no viewer means no process, IPC or pixel planning.
        if self._stop.is_set():
            return
        if self._process is None:
            context = multiprocessing.get_context("spawn")
            self._connection, child = context.Pipe()
            self._process = context.Process(target=_reference_process,
                args=(child,self._configuration),name="reference-display",daemon=True)
            try:
                self._process.start()
            except (OSError, RuntimeError) as exc:
                self._stats["error"] = type(exc).__name__+": "+str(exc)[:120]
                self._stop.set()
                self._connection.close()
                child.close()
                self._process = None
                return
            child.close()
            self._thread = threading.Thread(target=self._exchange,name="reference-display-ipc",daemon=True)
            self._thread.start()
        with self._lock:
            self._stats["submitted"] += 1
            self._stats["replaced"] += self._pending is not None
            self._pending = (self._generation,key,frame,probability,processing_ms)
        self._wake.set()

    def _exchange(self):
        next_at = 0.
        try:
            while not self._stop.is_set():
                if not self._wake.wait(.1):
                    continue
                if self._stop.wait(max(0.,next_at-time.monotonic())):
                    break
                with self._lock:
                    job,self._pending = self._pending,None
                    self._wake.clear()
                if job is None:
                    continue
                next_at = time.monotonic()+self._interval
                self._connection.send(job)
                answer = self._connection.recv()
                with self._lock:
                    if answer[0] == self._generation:
                        self._result = answer
                        self._stats["completed"] += 1
                        self._stats["error"] = answer[3] or ""
        except (EOFError, BrokenPipeError, OSError) as exc:
            with self._lock:
                self._stats["error"] = type(exc).__name__+": "+str(exc)[:120]
            self._stop.set()
        finally:
            try:
                self._connection.send(None)
            except (BrokenPipeError, EOFError, OSError):
                pass

    def take(self):
        with self._lock:
            result,self._result = self._result,None
            return result

    def reset(self):
        with self._lock:
            self._generation += 1
            self._pending = self._result = None

    def stats(self):
        with self._lock:
            return dict(self._stats)

    def close(self):
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(.2)
        if self._process is not None:
            self._process.join(.2)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(.2)
            self._connection.close()
            self._thread.join(.2)
