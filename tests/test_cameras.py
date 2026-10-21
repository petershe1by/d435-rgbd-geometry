import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from rgbd_geometry.cameras import RealSenseSource, SecondaryFusion, USBSource
from rgbd_geometry.configuration import validate_config


def fake_sdk():
    rs = MagicMock()
    raw, color, frames, profile = MagicMock(), MagicMock(), MagicMock(), MagicMock()
    raw.get_data.return_value = np.array([[0, 1200], [1000, 900]], np.uint16)
    raw.get_frame_number.return_value = 12
    color.get_data.return_value = np.zeros((2, 2, 3), np.uint8)
    color.profile.as_video_stream_profile.return_value.get_intrinsics.return_value = SimpleNamespace(
        width=2, height=2, fx=2., fy=2., ppx=1., ppy=1., coeffs=[0]*5)
    frames.get_depth_frame.return_value = raw
    frames.get_color_frame.return_value = color
    rs.align.return_value.process.return_value = frames
    rs.pipeline.return_value.wait_for_frames.return_value = frames
    rs.pipeline.return_value.start.return_value = profile
    profile.get_device.return_value.first_depth_sensor.return_value.get_depth_scale.return_value = .001
    rs.disparity_transform.side_effect = lambda *args: SimpleNamespace(process=lambda frame: frame)
    rs.spatial_filter.return_value.process.side_effect = lambda frame: frame
    rs.temporal_filter.return_value.process.side_effect = lambda frame: frame
    return rs, profile


class CameraTests(unittest.TestCase):
    def test_resolution_failure_reports_requested_stream_without_stopping_unstarted_pipeline(self):
        rs, _ = fake_sdk()
        rs.pipeline.return_value.start.side_effect = RuntimeError("Couldn't resolve requests")
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            with self.assertRaisesRegex(RuntimeError, "848x480 @ 30 fps"):
                RealSenseSource(validate_config({"width": 848, "fps": 30}))
            rs.pipeline.return_value.stop.assert_not_called()

    def test_frame_timeout_reports_usb_connection_and_releases_on_close(self):
        rs, profile = fake_sdk()
        device = profile.get_device.return_value
        device.supports.return_value = True
        device.get_info.return_value = "2.1"
        rs.pipeline.return_value.wait_for_frames.side_effect = RuntimeError("Frame didn't arrive within 5000")
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            source = RealSenseSource(validate_config({}))
            with self.assertRaisesRegex(RuntimeError, "USB 2.1"):
                source.read()
            source.close()
            rs.pipeline.return_value.stop.assert_called_once()

    def test_non_resolution_start_failure_is_preserved(self):
        rs, _ = fake_sdk()
        rs.pipeline.return_value.start.side_effect = RuntimeError("Device unavailable")
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            with self.assertRaisesRegex(RuntimeError, "Device unavailable"):
                RealSenseSource(validate_config({}))
            rs.pipeline.return_value.stop.assert_not_called()

    def test_sdk_scale_validity_filter_toggle_and_close(self):
        rs, _ = fake_sdk()
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            source = RealSenseSource(validate_config({}))
            frame = source.read()
            self.assertEqual(frame.frame_number, 12)
            self.assertEqual(frame.depth[0, 0], 0)
            self.assertAlmostEqual(float(frame.depth[0, 1]), 1.2, places=6)
            source.configure(validate_config({"temporal_depth_filter": True}))
            source.read()
            self.assertTrue(rs.temporal_filter.return_value.process.called)
            source.configure(validate_config({}))
            self.assertIsNone(source.temporal)
            source.close()
            source.close()
            rs.pipeline.return_value.stop.assert_called_once()

    def test_live_disconnect_is_not_reported_as_eof(self):
        rs, _ = fake_sdk()
        rs.pipeline.return_value.wait_for_frames.side_effect = RuntimeError("disconnect")
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            source = RealSenseSource(validate_config({}))
            with self.assertRaises(RuntimeError):
                source.read()
            source.close()

    def test_playback_pauses_between_reads_and_handles_eof(self):
        rs, profile = fake_sdk()
        rs.playback_status = SimpleNamespace(stopped="stopped", paused="paused")
        playback = profile.get_device.return_value.as_playback.return_value
        playback.current_status.return_value = "paused"
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            source = RealSenseSource(validate_config({}), bag=Path("mock.bag"))
            source.read()
            playback.resume.assert_called_once()
            self.assertEqual(playback.pause.call_count, 2)
            playback.current_status.return_value = "stopped"
            self.assertIsNone(source.read())
            rs.pipeline.return_value.wait_for_frames.assert_called_once()
            source.close()

    def test_incomplete_frames_do_not_silently_end_stream(self):
        rs, _ = fake_sdk()
        rs.align.return_value.process.return_value.get_color_frame.return_value = None
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            source = RealSenseSource(validate_config({}))
            with self.assertRaises(RuntimeError):
                source.read()
            source.close()

    def test_startup_failure_releases_started_pipeline(self):
        rs, profile = fake_sdk()
        profile.get_device.return_value.first_depth_sensor.return_value.get_depth_scale.side_effect = RuntimeError("sensor failure")
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            with self.assertRaises(RuntimeError):
                RealSenseSource(validate_config({}))
            rs.pipeline.return_value.stop.assert_called_once()

    def test_usb_failure_and_thread_shutdown_release(self):
        capture = MagicMock()
        capture.isOpened.return_value = False
        with patch("rgbd_geometry.cameras.cv2.VideoCapture", return_value=capture):
            with self.assertRaises(RuntimeError):
                USBSource(1)
            capture.release.assert_called_once()
        capture = MagicMock()
        capture.isOpened.return_value = True
        capture.read.return_value = (False, None)
        with patch("rgbd_geometry.cameras.cv2.VideoCapture", return_value=capture):
            usb = USBSource(1)
            self.assertIsNone(usb.read())
            usb.close()
            self.assertFalse(usb.thread.is_alive())
            capture.release.assert_called_once()

    def test_secondary_maps_brightness_and_invalid_rotation(self):
        data = {"usb_matrix": [[90., 0, 40], [0, 90., 30], [0, 0, 1]], "usb_distortion": [0]*5,
                "rotation_d435_to_usb": np.eye(3).tolist(), "translation_d435_to_usb_m": [0, 0, 0],
                "usb_image_size": [80, 60], "d435_image_size": [80, 60], "stereo_rms_px": .2}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"stereo.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            fusion = SecondaryFusion(path)
            fusion.background = np.full((60, 80, 3), 150, np.uint8)
            bright = np.full((60, 80, 3), 170, np.uint8)
            self.assertEqual(fusion.masks(bright), [])
            bright[20:40, 30:50] = [30, 40, 220]
            self.assertEqual(len(fusion.masks(bright)), 1)
            self.assertEqual(fusion.rectify(bright).shape, bright.shape)
            data["rotation_d435_to_usb"] = np.zeros((3, 3)).tolist()
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(ValueError):
                SecondaryFusion(path)


if __name__ == "__main__":
    unittest.main()
