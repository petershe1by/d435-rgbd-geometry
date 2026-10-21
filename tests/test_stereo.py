import unittest
import numpy as np
import cv2
from calibrate_stereo import calibrate


class StereoCalibrationTests(unittest.TestCase):
    def test_known_metric_stereo_transform(self):
        rng = np.random.default_rng(17)
        board = np.zeros((54, 3), np.float32)
        board[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)*.025
        K1 = np.array([[700., 0, 424], [0, 700., 240], [0, 0, 1]])
        K2 = np.array([[850., 0, 640], [0, 860., 360], [0, 0, 1]])
        R, _ = cv2.Rodrigues(np.array([.08, -.20, .04]))
        t = np.array([.12, .015, .02])
        objects, first, second = [], [], []
        for _ in range(22):
            rotation = rng.uniform(-.4, .4, 3)
            translation = np.array([rng.uniform(-.15, .05), rng.uniform(-.10, .04), rng.uniform(.65, 1.)])
            board_R, _ = cv2.Rodrigues(rotation)
            projected1, _ = cv2.projectPoints(board, rotation, translation, K1, np.zeros(5))
            other_R, _ = cv2.Rodrigues(R @ board_R)
            projected2, _ = cv2.projectPoints(board, other_R, R @ translation+t, K2, np.zeros(5))
            projected1 += rng.normal(0, .04, projected1.shape).astype(np.float32)
            projected2 += rng.normal(0, .04, projected2.shape).astype(np.float32)
            objects.append(board.copy())
            first.append(projected1)
            second.append(projected2)
        result = calibrate(objects, first, second, K1, (848, 480), (1280, 720))
        self.assertLess(result["stereo_rms_px"], .15)
        self.assertLess(np.linalg.norm(np.array(result["translation_d435_to_usb_m"])-t), .004)
        self.assertLess(np.linalg.norm(np.array(result["rotation_d435_to_usb"])-R), .015)

    def test_reject_too_few_views(self):
        with self.assertRaises(ValueError):
            calibrate([], [], [], np.eye(3), (848, 480), (1280, 720))


if __name__ == "__main__":
    unittest.main()
