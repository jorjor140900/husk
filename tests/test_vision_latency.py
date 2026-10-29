"""Differentiate delivery latency, repeated captures and between-frame outages."""
import json
import unittest

from tools.measure_vision_latency import StreamTiming,VisionTiming


class VisionLatencyTests(unittest.TestCase):
    def test_fifteen_fps_with_50ms_source_age_stays_fresh(self):
        stream=StreamTiming()
        for i in range(150):
            now=100.+i/15
            stream.observe(round((now-.05)*1e9),round(now*1e9),now)
            stream.sample_gate(round(now*1e9),now)
            stream.sample_gate(round((now+.05)*1e9),now+.05)
        report=stream.report(10.)
        self.assertEqual(report["delivered_fps"],15.)
        self.assertEqual(report["sampled_age_gate_closed_percent"],0.)
        self.assertEqual(report["source_age_at_receive_ms"]["p95"],50.)

    def test_good_receipt_latency_can_still_stop_between_slow_ai_results(self):
        stream=StreamTiming()
        for i in range(100):
            now=100.+i*.05
            if i%10==0:stream.observe(round((now-.3)*1e9),round(now*1e9),now)
            stream.sample_gate(round(now*1e9),now)
        report=stream.report(5.)
        self.assertEqual(report["delivered_fps"],2.)
        self.assertEqual(report["source_age_at_receive_ms"]["p95"],300.)
        self.assertGreater(report["sampled_age_gate_closed_percent"],40.)

    def test_real_two_second_source_delay_closes_the_age_gate(self):
        stream=StreamTiming()
        for i in range(20):
            now=100.+i*.1
            stream.observe(round((now-2)*1e9),round(now*1e9),now)
            stream.sample_gate(round(now*1e9),now)
        self.assertEqual(stream.report(2.)["sampled_age_gate_closed_percent"],100.)

    def test_heartbeat_repeats_do_not_count_as_new_frames_or_hide_staleness(self):
        stream=StreamTiming()
        for i in range(20):
            now=100.+i*.1
            stream.observe(100*10**9,round(now*1e9),now)
            stream.sample_gate(round(now*1e9),now)
        report=stream.report(2.)
        self.assertEqual(report["unique_captures"],1)
        self.assertGreater(report["sampled_age_gate_closed_percent"],60.)

    def test_invalid_packet_clears_freshness_and_fast_mode_empty_path_is_not_a_mask_error(self):
        timing=VisionTiming()
        packet=json.dumps(dict(protocol=1,valid=True,source_header=dict(stamp=dict(sec=100,nanosec=0))))
        timing.packet(packet,100*10**9,100.)
        timing.status(json.dumps(dict(valid_image_path=False,reason="nav2_segmentation_only",inference_ms=120.)))
        timing.sample(100*10**9,100.)
        self.assertEqual(timing.streams["ai_mask"].closed,0)
        timing.packet('{"protocol":1,"valid":false}',100*10**9,100.)
        timing.sample(100*10**9,100.)
        self.assertEqual(timing.streams["ai_mask"].closed,1)
        report=timing.report(1.)
        self.assertEqual(report["ai_reported_timings_ms"]["inference_ms"]["p95"],120.)
        self.assertFalse(report["ready_for_control"])
        json.dumps(report,allow_nan=False)

    def test_future_clock_and_zero_stamp_cannot_pass_as_low_latency(self):
        stream=StreamTiming()
        stream.observe(102*10**9,100*10**9,100.)
        stream.sample_gate(100*10**9,100.)
        self.assertEqual(stream.closed,1)
        self.assertEqual(stream.future,1)
        stream.observe(0,100*10**9,100.)
        self.assertIsNone(stream.latest)


if __name__=="__main__":unittest.main()
