# hand_tracking.py
import numpy as np
import cv2
import mediapipe as mp

# Handle both import layouts for MediaPipe Tasks
try:
    BaseOptions = mp.tasks.BaseOptions
    VisionRunningMode = mp.tasks.vision.RunningMode
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
except AttributeError:
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    BaseOptions = mp_python.BaseOptions
    VisionRunningMode = mp_vision.RunningMode
    HandLandmarker = mp_vision.HandLandmarker
    HandLandmarkerOptions = mp_vision.HandLandmarkerOptions

class HandTracker:
    def __init__(self, model_path: str, max_hands: int = 1):
        self.max_hands = int(max_hands)

        # More permissive thresholds help initial lock in typical room lighting
        opts = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            num_hands=self.max_hands,
            running_mode=VisionRunningMode.VIDEO,
            min_hand_detection_confidence=0.2,
            min_hand_presence_confidence=0.2,
            min_tracking_confidence=0.2,
        )
        self._hand = HandLandmarker.create_from_options(opts)

    def detect(self, frame_bgr: np.ndarray, ts_ms: int):
        """
        Returns {"hands": [ {"landmarks": (21,3) float32, "world": (21,3) float32 optional}, ... ] }
        """
        if frame_bgr is None or frame_bgr.size == 0:
            return {"hands": []}

        # Light denoise to help detection, then BGR -> RGB. Ensure contiguous uint8.
        frame_bgr = cv2.GaussianBlur(frame_bgr, (3, 3), 0)
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._hand.detect_for_video(mp_image, ts_ms)

        hands = []
        if result and getattr(result, "hand_landmarks", None):
            world_sets = getattr(result, "hand_world_landmarks", None)
            for i, lm_list in enumerate(result.hand_landmarks[: self.max_hands]):
                img_pts = np.array([[lm.x, lm.y, lm.z] for lm in lm_list], dtype=np.float32)
                item = {"landmarks": img_pts}
                if world_sets and i < len(world_sets):
                    world_pts = np.array([[lm.x, lm.y, lm.z] for lm in world_sets[i]], dtype=np.float32)
                    item["world"] = world_pts
                hands.append(item)

        return {"hands": hands}

    def close(self):
        if getattr(self, "_hand", None):
            self._hand.close()

