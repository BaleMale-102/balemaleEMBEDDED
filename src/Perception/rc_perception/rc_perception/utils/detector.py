import cv2
import numpy as np

# --- Optional dependency (RANSAC / skimage) ---
try:
    from skimage.measure import LineModelND, ransac
    _HAS_SKIMAGE = True
except Exception:
    LineModelND = None
    ransac = None
    _HAS_SKIMAGE = False


class Detector:
    def __init__(self, scan_range={'start': 0, 'stop': 240, 'steps': 20},
                 scan_window={'height': 15, 'max_adjust': 10}):
        self.scan_range = scan_range
        self.scan_window = scan_window

        # ✅ skimage 없는 환경에서도 import/실행되게
        self.model = LineModelND() if _HAS_SKIMAGE else None
        self.debug = []

    def img_filter(self, iimg):
        """
        Filters an BGR image to get lane boundaries (single channel output)
        """
        iimg = cv2.cvtColor(iimg, cv2.COLOR_BGR2GRAY)

        ciimg = cv2.Canny(iimg, 50, 250)
        ciimg = cv2.morphologyEx(ciimg, cv2.MORPH_DILATE, (1, 1), iterations=5)

        siimg = cv2.Sobel(iimg, cv2.CV_8U, 1, 0, ksize=1)
        siimg = cv2.morphologyEx(siimg, cv2.MORPH_DILATE, (1, 1), iterations=5)
        _, siimg = cv2.threshold(siimg, 50, 255, cv2.THRESH_OTSU)
        siimg = cv2.morphologyEx(siimg, cv2.MORPH_CLOSE, (3, 3))

        out = cv2.bitwise_and(ciimg, ciimg, mask=siimg)
        return out

    def get_lane_detections(self, iimg,
                            start={'x': 105, 'y': 230},
                            stop={'x': 135, 'y': 230},
                            label='mid',
                            use_RANSAC=True,
                            debug=False):
        """
        Parses the input (1ch) image with virtual sensors, detects peaks, returns points
        """
        self.debug = []  # ✅ call마다 초기화 (프레임 누적 방지)

        adjust = 0
        minx = min(start['x'], stop['x'])
        maxx = max(start['x'], stop['x']) + adjust
        detections = []

        # y 범위 방어(이미지 밖 접근 방지)
        H, W = iimg.shape[:2]
        win_h = int(self.scan_window.get('height', 15))
        max_adjust = int(self.scan_window.get('max_adjust', 10))
        steps = int(self.scan_range.get('steps', 20))
        y_start = int(self.scan_range.get('start', 0))
        y_stop = int(self.scan_range.get('stop', H))

        # stop이 이미지보다 크면 clamp
        y_stop = min(y_stop, H)

        for i in range(y_start, y_stop, steps):
            y = int(start['y'] - i)

            # window가 이미지 밖이면 skip
            if y < 0 or (y + win_h) > H:
                continue

            # x 범위 clamp
            x0 = int(clamp(minx, 0, W - 1))
            x1 = int(clamp(maxx, 0, W))
            if x1 <= x0 + 1:
                continue

            det_line = iimg[y:y + win_h, x0:x1]
            hist = np.sum(det_line, axis=0)

            if hist.size == 0:
                continue

            peek = int(np.argmax(hist))

            # threshold = average
            if hist[peek] > np.average(hist):
                x1p = x0 + peek
                y1p = y
                det_mid_x = x0 + len(hist) // 2

                adjust = x1p - det_mid_x

                # apply adjust only if in defined range
                if abs(adjust) >= max_adjust:
                    adjust = int(np.sign(adjust) * max_adjust)

                minx += adjust
                maxx += adjust

                detections.append([x1p, y1p])

                if debug:
                    self.debug.append({
                        'detection': [x1p, y1p],
                        'detection_mid': [det_mid_x, y1p],
                        'rectangle': [x0, y1p, x0 + len(hist), y1p + win_h]
                    })

        # ✅ RANSAC은 “가능할 때만” 적용 (없으면 그냥 패스)
        if use_RANSAC and _HAS_SKIMAGE and len(detections) >= 2:
            _, inliers = self.filter_outliers(detections)
            if inliers is not None and np.any(inliers):
                detections = np.array(detections)[inliers]
            else:
                detections = np.array(detections)
        else:
            detections = np.array(detections)

        return {label: detections}

    def filter_outliers(self, data):
        """
        Apply RANSAC (only if skimage is available)
        """
        if (not _HAS_SKIMAGE) or (ransac is None) or (LineModelND is None):
            return None, None

        data = np.array(data)
        if data.shape[0] < 2:
            return None, None

        try:
            model_robust, inliers = ransac(
                data, LineModelND,
                min_samples=2,
                residual_threshold=1,
                max_trials=200
            )
            return model_robust, inliers
        except Exception:
            return None, None

    def draw_detections(self, iimg, data):
        """
        Visualize detections (expects 1-channel image)
        """
        limg = iimg.copy()
        for v in data:
            cv2.circle(limg, (int(v[0]), int(v[1])), 2, [255], -1)

        if len(self.debug) > 0:
            for v in self.debug:
                cv2.circle(limg, (int(v['detection_mid'][0]), int(v['detection_mid'][1] + 5)), 1, [255])
                cv2.rectangle(
                    limg,
                    (int(v['rectangle'][0]), int(v['rectangle'][1])),
                    (int(v['rectangle'][2]), int(v['rectangle'][3])),
                    [255], 1
                )

        return limg


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v
