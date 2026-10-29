"""Real terminal key handling with ROS service/message stand-ins, no robot."""
import importlib.util
import io
import json
import os
from pathlib import Path
import pty
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest import mock

from husky_link_core.operator_permission import PERMISSION_TOPIC, PREVIEW_TOPIC
from husky_link_core.protocol import stamp_dict


class OperatorConsoleTests(unittest.TestCase):
    def test_terminal_g_space_and_session_expiry_publish_false_and_disarm(self):
        shutdown, received_state = threading.Event(), threading.Event()
        messages, calls, errors = [], [], []
        state = dict(armed=False)
        def publisher(topic, cls, **kwargs):
            self.assertEqual(topic, PERMISSION_TOPIC)
            self.assertFalse(kwargs["latch"])
            return NS(publish=lambda m: messages.append((m.data, time.monotonic())))
        def subscribe(topic, cls, callback, **kwargs):
            def emit():
                while not shutdown.is_set():
                    value = dict(protocol=1, stamp=stamp_dict(time.time_ns()), armed=state["armed"],
                        driver_output_enabled=False, output_topic=PREVIEW_TOPIC)
                    callback(NS(data=json.dumps(value)))
                    state["reported_armed"] = value["armed"]
                    received_state.set()
                    shutdown.wait(.02)
            thread = threading.Thread(target=emit)
            thread.start()
            return thread
        def service(enabled):
            calls.append(enabled)
            if enabled:
                self.assertTrue(messages[-1][0])
                self.assertLess(time.monotonic()-messages[-1][1], .3)
            state["armed"] = enabled
            return NS(success=True, message="armed" if enabled else "disarmed")
        with tempfile.TemporaryDirectory() as folder:
            profile = Path(folder)/"profile.json"
            profile.write_text(json.dumps(dict(robot=dict(ros1_permission_topic=PERMISSION_TOPIC,
                joy_topic=None, joy_deadman_button=None))))
            params = {"~profile_path":str(profile), "~session_sec":1.0}
            ros = NS(init_node=lambda *a,**kw:None,
                get_param=lambda name,*default:params.get(name,default[0] if default else None),
                Publisher=publisher, Subscriber=subscribe, wait_for_service=lambda *a,**kw:None,
                ServiceProxy=lambda *a:service, is_shutdown=shutdown.is_set,
                signal_shutdown=lambda *a:shutdown.set(), Time=NS(now=lambda:NS(to_nsec=time.time_ns)),
                ServiceException=RuntimeError, ROSException=RuntimeError)
            modules = {"rospy":ros, "std_msgs":NS(), "std_msgs.msg":NS(Bool=NS,String=NS),
                       "std_srvs":NS(), "std_srvs.srv":NS(SetBool=object)}
            path = Path(__file__).parents[1]/"ros1/husky_vision_bridge/scripts/operator_console.py"
            spec = importlib.util.spec_from_file_location("_operator_console_test",path)
            module = importlib.util.module_from_spec(spec)
            with mock.patch.dict("sys.modules",modules): spec.loader.exec_module(module)
            master, slave = pty.openpty()
            terminal = os.fdopen(slave,"r")
            def run():
                try: module.main()
                except BaseException as exc: errors.append(exc)
            def until(predicate):
                deadline = time.monotonic()+2
                while not predicate() and time.monotonic()<deadline:
                    time.sleep(.01)
                self.assertTrue(predicate(), errors)
            try:
                with mock.patch.object(module.sys,"stdin",terminal), \
                     mock.patch.object(module.sys,"stdout",io.StringIO()), \
                     mock.patch.object(module.signal,"signal"):
                    thread = threading.Thread(target=run)
                    thread.start()
                    try:
                        self.assertTrue(received_state.wait(1))
                        until(lambda: len(messages)>0)
                        self.assertFalse(messages[-1][0])
                        os.write(master,b"g")
                        until(lambda: state["armed"])
                        os.write(master,b" ")
                        until(lambda: not state["armed"])
                        self.assertFalse(messages[-1][0])
                        until(lambda: not state["reported_armed"])
                        os.write(master,b"g")
                        until(lambda: state["armed"])
                        # No keypress: the finite deadline must still stop.
                        until(lambda: not state["armed"])
                        self.assertFalse(messages[-1][0])
                        os.write(master,b"q")
                        thread.join(timeout=2)
                        self.assertFalse(thread.is_alive())
                    finally:
                        shutdown.set()
                        thread.join(timeout=2)
            finally:
                shutdown.set()
                terminal.close()
                os.close(master)
            self.assertEqual(errors,[])
            self.assertEqual(calls.count(True),2)
            self.assertFalse(calls[-1])
