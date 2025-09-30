import cv2
import mediapipe as mp
from angles import extract_finger_angles

BaseOptions = mp.tasks.BaseOptions
vision = mp.tasks.vision
HandLandmarker = vision.HandLandmarker
HandLandmarkerOptions = vision.HandLandmarkerOptions
RunningMode = vision.RunningMode

class HandTracker:
    def __init__(self, model_path: str, max_hands: int = 2,
                 det_conf: float = 0.4, track_conf: float = 0.4):
        opts = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=det_conf,
            min_hand_presence_confidence=track_conf,
            min_tracking_confidence=track_conf,
        )
        self._landmarker = HandLandmarker.create_from_options(opts)

    def detect(self, bgr_frame, timestamp_ms: int):
        """
        Run once and return both landmarks and 15 joint angles for each hand.
        Returns: {"hands": [ {handedness, score, landmarks, angles_rad}, ... ]}
        """
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        result = self._landmarker.detect_for_video(mp_image, timestamp_ms)
        out = {"hands": []}
        if not result.hand_landmarks:
            return out

        for i, hand in enumerate(result.hand_landmarks):
            lms = [(lm.x, lm.y, lm.z) for lm in hand]

            if result.handedness and len(result.handedness) > i and len(result.handedness[i]) > 0:
                cat = result.handedness[i][0]
                label = cat.category_name
                score = float(cat.score)
            else:
                label, score = None, 0.0

            ang = extract_finger_angles(lms)
            out["hands"].append({
                "handedness": label,
                "score": score,
                "landmarks": lms,
                "angles_rad": ang
            })
        return out

    def close(self):
        self._landmarker.close()

