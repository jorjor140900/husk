"""
RoadDetection_v5.py

CHANGES IN v5 (marked with  # [v5]  in the code) -- a NEW detection method, 'Method' trackbar:
  Why: k-means compares every pixel with ONE global color per cluster. A polished floor
  has a brightness gradient (reflections), so it is split into several clusters (road
  found only as a narrow strip); raising the brightness weight to stop that makes pillars
  and walls leak instead. No single setting works for both scenes.
  Method 1 (default) = REGION GROWING from the bottom-center of the frame (the ground
  right in front of the robot IS road):
    - each pixel is compared with its NEIGHBOR, not with a global color, so a smooth
      brightness gradient is accepted while an abrupt step (floor -> pillar base / wall)
      stops the growth;
    - growth cannot cross the edge barrier or go above the horizon;
    - a chroma (a, b) gate keeps a slowly drifting fill from wandering off the floor color.
  Method 0 = the old k-means pipeline (unchanged).
  New trackbars: 'Method', 'Flood Step' (how big a neighbor-to-neighbor brightness step is
  still "same floor"), 'Chroma Tol' (how far the color tint may differ from the floor in
  front of the robot).

CHANGES IN v4 (marked with  # [v4]  in the code) -- fixes wrong boundary lines and over-strict colors:
  1. EDGE LINES are no longer fitted to the far end of the mask. The top 30% of the
     road's rows (where the "edge" is just the horizon cut / far boundary, not a side
     of the road) are skipped. A side also needs points spread over >= 35% of the road
     height, and its line must not be almost horizontal (|dx/dy| <= 4); otherwise that
     side counts as "missing" (previous line held for a few frames, then frame border).
  2. 'Color Tolerance' is now a trackbar (default 18) and 'Brightness Weight' drives
     BOTH the clustering and the color comparison (default 3). v3 was too strict for
     dark / wet concrete and split one road into two.
  3. Horizon default 44 -> 30. A fixed line cuts real road when the camera pitch
     differs. Re-tune 'Horizon %' whenever the camera tilt changes.

Road detection via generic COLOR CLUSTERING -- no hardcoded "road is gray"
or "vegetation is green" assumption anywhere.

CHANGES IN v3 (marked with  # [v3]  in the code) -- fixes "mask leaks onto walls / pillars":
  1. HORIZON LIMIT: new trackbar 'Horizon %' (replaces 'Ground Top %').
     Everything above this line (walls, ceiling, sky) is NEVER road:
       - k-means is fitted only below it,
       - cluster statistics only count pixels below it,
       - every road mask is cut at it (low-res, and again at full-res).
     A thin white line at the horizon is drawn on the output while debug is ON,
     so you can tune it by eye.
  2. BRIGHTNESS now matters: Brightness Weight default 1 -> 5,
     COLOR_DIST_L_WEIGHT 0.15 -> 0.40. A light-gray floor and a white wall can
     now be told apart.
  3. STRICTER color gates: ROAD_COLOR_TOLERANCE / MERGE_COLOR_TOLERANCE 24 -> 14,
     seed strip narrower (0.60 -> 0.45), a cluster needs more share in the strip
     (0.12 -> 0.20).
  4. STRONGER edge barrier: BARRIER_MIN_CONTOUR_LEN 25 -> 15, BARRIER_DILATE_PX 0 -> 1
     (pillar bases / wall bottoms now block the leak).
  5. Fill Mode default 1 -> 0 (raw mask). Mode 1 fills whole rows left-to-right,
     so ONE leaked pixel turned a whole row into "road".
  6. SANITY CHECK: if the road covers almost nothing, or practically the whole
     area under the horizon, the frame is declared INVALID (valid=False and NO
     lateral/heading error is published) instead of steering on garbage.

Everything else (stability, multi-reference road colors, edge-point handling)
is unchanged from v2.

ROAD AREA ('Fill Mode' trackbar):
  0 (default) raw detected mask
  1 fill every row between its left and right edge
  2 area between two straight lines fitted to the left/right edge

Press 'q' to quit, 'd' to show / hide the processing windows.
"""

import os
import time
import queue
import threading

import cv2
import numpy as np
from std_msgs.msg import Float32, Bool

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


cv2.setRNGSeed(12345)   # reproducible k-means starts

SOURCE_PATH = None  # ROS 2 camera input is received from image_topic

FRAME_W, FRAME_H = 640, 360   # output/display resolution
ORIGINAL_RESOLUTION = False   # True = keep the video's native size instead of resizing

ROAD_PAINT_COLOR = (0, 220, 0)   # BGR -- color painted over the detected road area

GROUP_DISPLAY_COLORS = [
    (60, 200, 255), (255, 120, 60), (120, 255, 120), (255, 60, 200),
    (60, 255, 255), (200, 60, 255), (60, 120, 255), (180, 180, 60),
]

SHOW_DEBUG = True
TILE_DEBUG_WINDOWS = True

DEBUG_VIEW_KEYS = ["kmeans", "seed_cc", "edge_morph", "boundary"]
DEBUG_WINDOWS_ENABLED = {
    "kmeans": True,
    "seed_cc": True,
    "edge_morph": True,
    "boundary": True,
}
DEBUG_WINDOW_NAMES = [
    "1) K-means Color Clustering",
    "2) Seed-based Connected Component",
    "3) Edge + Morphological Processing",
    "4) Road Boundary + Reference",
]

DEBUG_AREA_X0 = FRAME_W + 24

MAIN_WINDOW = "Color Cluster Road Detection"

# ---------------------------------------------------------------------- #
# Tunable constants
# ---------------------------------------------------------------------- #
# [v3] narrower seed strip, and a cluster needs a bigger share of it
SEED_REGION_W = 0.45            # v2: 0.60
SEED_REGION_H = 0.18
SEED_LABEL_MIN_FRACTION = 0.20  # v2: 0.12

# [v3] brightness counts much more when comparing colors (floor vs white wall)
COLOR_DIST_L_WEIGHT = 0.30      # [v4] overwritten every frame from the 'Brightness Weight' trackbar
ROAD_COLOR_TOLERANCE = 18.0     # [v4] overwritten every frame from the 'Color Tolerance' trackbar
MERGE_COLOR_TOLERANCE = 18.0    # [v4] same trackbar; a cluster may only merge into the road if this close
ROAD_JUMP_FRAMES = 12           # a different-looking "road" must persist this many frames to be believed
ROAD_COLOR_EMA = 0.1            # how fast tracked road colors adapt

# Multi-reference road colors (sunlit / shaded / ...)
MAX_ROAD_REFS = 3               # how many distinct road colors to remember
REF_NEW_TOL = 12.0              # a color farther than this from every reference becomes a NEW reference
REF_MIN_SHARE = 0.15            # a cluster must be >= this share of the road mask to update references

# [v3] Horizon: fraction of the frame (from the top) that can NEVER be road.
# Default value of the 'Horizon %' trackbar. Tune it to where the floor meets the walls.
DEFAULT_HORIZON_PCT = 30   # [v4] was 44 -- move the trackbar to where the far ground ends

# [v3] Sanity check on the final mask, as a fraction of the area BELOW the horizon
SANITY_MIN_GROUND_FRACTION = 0.03   # less than this -> nothing useful found
SANITY_MAX_GROUND_FRACTION = 0.97   # more than this -> mask leaked over everything

# [v5] Region growing (Method 1)
FLOOD_STEP_DEFAULT = 6          # max brightness (L) step between two neighboring pixels of the same floor
FLOOD_CHROMA_TOL_DEFAULT = 10   # max (a, b) color difference from the floor right in front of the robot
FLOOD_SEED_COUNT = 5            # seeds spread along the bottom row (only floor-colored ones are used)
FLOOD_SEED_SPAN = (0.30, 0.70)  # seeds are placed between these fractions of the frame width

FILL_OBSTACLE_HOLES = True
OBSTACLE_BRIDGE_PX = 5
HOLE_MAX_FRACTION = 0.10

# [v3] Edge barrier: a bit stronger than v2 so pillar bases / wall bottoms block leaks
BARRIER_MIN_CONTOUR_LEN = 15    # v2: 25
BARRIER_DILATE_PX = 1           # v2: 0

# Boundary fitting
EDGE_BORDER_IGNORE_PX = 3       # points this close to the image border are not real edges
MIN_EDGE_POINTS = 5             # fewer valid points than this -> that side is "missing"
EDGE_TOP_TRIM = 0.30            # [v4] skip the far 30% of the road's rows when picking edge points
MIN_EDGE_SPAN = 0.35            # [v4] a side's points must cover this fraction of the road height
MAX_EDGE_SLOPE = 4.0            # [v4] |dx/dy| above this = almost horizontal = not a road side
MAX_HOLD_FRAMES = 3             # reuse the previous line for this many frames when a side is missing


def nothing(x):
    pass


cv2.namedWindow('Trackbars')
cv2.createTrackbar('K Clusters', 'Trackbars', 5, 8, nothing)
cv2.createTrackbar('Downscale Factor', 'Trackbars', 3, 8, nothing)
cv2.createTrackbar('Use Edge Barrier', 'Trackbars', 1, 1, nothing)
cv2.createTrackbar('Use Dynamic Seed', 'Trackbars', 1, 1, nothing)
cv2.createTrackbar('Show FPS', 'Trackbars', 1, 1, nothing)
cv2.createTrackbar('Fill Mode', 'Trackbars', 0, 2, nothing)                         # [v3] was 1
cv2.createTrackbar('Stability', 'Trackbars', 4, 9, nothing)
cv2.createTrackbar('Brightness Weight', 'Trackbars', 3, 10, nothing)                # [v3] was 1
cv2.createTrackbar('Realtime Sync', 'Trackbars', 1, 1, nothing)
cv2.createTrackbar('Color Tolerance', 'Trackbars', 18, 40, nothing)                 # [v4] new (Method 0 only)
cv2.createTrackbar('Method', 'Trackbars', 1, 1, nothing)                            # [v5] 0 = k-means, 1 = region growing
cv2.createTrackbar('Flood Step', 'Trackbars', FLOOD_STEP_DEFAULT, 20, nothing)      # [v5] Method 1 only
cv2.createTrackbar('Chroma Tol', 'Trackbars', FLOOD_CHROMA_TOL_DEFAULT, 40, nothing)  # [v5] Method 1 only
cv2.createTrackbar('Horizon %', 'Trackbars', DEFAULT_HORIZON_PCT, 80, nothing)      # [v3] replaces 'Ground Top %'


# ---------------------------------------------------------------------- #
# Color clustering (runs entirely at low resolution)
# ---------------------------------------------------------------------- #
_light_w = 0.5      # clustering weight of the L channel; set from the trackbar each frame
_ground_top = DEFAULT_HORIZON_PCT / 100.0   # [v3] horizon: fraction of the frame (from the top) that is never road


def horizon_row(h):
    """[v3] First image row (of an image h rows tall) that may be road."""
    return int(np.clip(int(h * _ground_top), 0, h - 1))


def cut_above_horizon(mask):
    """[v3] Zero every row above the horizon (in place) and return the mask."""
    mask[:horizon_row(mask.shape[0])] = 0
    return mask


def compute_color_clusters(small_bgr, k):
    """Returns (labels, centers) at the SAME resolution as small_bgr.

    k-means is fitted ONLY on the ground region (rows below the horizon),
    then ALL pixels (including sky / buildings) are assigned to the nearest center.
    Brightness (L) is weighted by the 'Brightness Weight' trackbar."""
    lab_small = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2LAB)
    h, w = lab_small.shape[:2]
    samples = lab_small.reshape(-1, 3).astype(np.float32)
    samples[:, 0] *= _light_w

    y0 = horizon_row(h)
    fit = samples.reshape(h, w, 3)[y0:].reshape(-1, 3)
    if fit.shape[0] < k * 10:                      # safety: too little ground to fit on
        fit = samples

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

    centers = None
    if _prev_centers is not None and _prev_centers.shape[0] == k:
        # Warm start from the previous frame's centers (deterministic, stable identities)
        try:
            d = ((fit[:, None, :] - _prev_centers[None, :, :]) ** 2).sum(axis=2)
            init_labels = d.argmin(axis=1).astype(np.int32).reshape(-1, 1)
            _, _, centers = cv2.kmeans(fit, k, init_labels, criteria, 1, cv2.KMEANS_USE_INITIAL_LABELS)
        except cv2.error:
            centers = None
    if centers is None:
        _, _, centers = cv2.kmeans(fit, k, None, criteria, 2, cv2.KMEANS_PP_CENTERS)

    # Assign EVERY pixel of the frame to its nearest center
    d_all = ((samples[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)
    labels = d_all.argmin(axis=1).astype(np.int32)
    return labels.reshape(h, w), centers


_prev_centers = None


def match_clusters_to_previous(new_centers, prev_centers):
    k = new_centers.shape[0]
    if prev_centers is None or prev_centers.shape[0] != k:
        return np.arange(k), new_centers.copy()

    cost = np.linalg.norm(new_centers[:, None, :] - prev_centers[None, :, :], axis=2).astype(np.float64)
    mapping = np.full(k, -1, dtype=int)
    for _ in range(k):
        new_idx, slot_idx = np.unravel_index(np.argmin(cost), cost.shape)
        mapping[new_idx] = slot_idx
        cost[new_idx, :] = np.inf
        cost[:, slot_idx] = np.inf

    stable_centers = np.zeros_like(new_centers)
    for new_idx, slot_idx in enumerate(mapping):
        stable_centers[slot_idx] = new_centers[new_idx]
    return mapping, stable_centers


def compute_stable_clusters(small_bgr, k):
    global _prev_centers
    labels_raw, centers = compute_color_clusters(small_bgr, k)
    mapping, stable_centers = match_clusters_to_previous(centers, _prev_centers)
    labels = mapping[labels_raw]
    _prev_centers = stable_centers
    return labels, stable_centers


# ---------------------------------------------------------------------- #
# Edge barrier (runs at low resolution)
# ---------------------------------------------------------------------- #
def compute_edge_barrier(small_bgr, min_contour_len=BARRIER_MIN_CONTOUR_LEN, dilate_px=BARRIER_DILATE_PX):
    gray = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2GRAY)
    smooth = cv2.bilateralFilter(gray, 5, 40, 40)

    median = float(np.median(smooth))
    lower = int(max(0, 0.66 * median))
    upper = int(min(255, 1.33 * median))
    edges = cv2.Canny(smooth, lower, upper)

    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    barrier = np.zeros_like(edges)
    for c in contours:
        if cv2.arcLength(c, False) >= min_contour_len:
            cv2.drawContours(barrier, [c], -1, 255, 1)

    if dilate_px > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_px * 2 + 1, dilate_px * 2 + 1))
        barrier = cv2.dilate(barrier, kernel, iterations=1)

    return barrier


# ---------------------------------------------------------------------- #
# Dynamic seed (tracked in LOW-RES coordinates throughout)
# ---------------------------------------------------------------------- #
_prev_seed_small = None
_prev_road_mean_lab = None
_seed_fail_count = 0
_MAX_SEED_FAILURES = 10
_COLOR_MATCH_TOLERANCE = 25.0


def get_dynamic_seed(lab_small, small_shape):
    h, w = small_shape[:2]
    default_x, default_y = w // 2, h - 2

    if _prev_seed_small is None or _prev_road_mean_lab is None:
        return default_x, default_y, False

    px, py = _prev_seed_small
    px = int(np.clip(px, 0, w - 1))
    py = int(np.clip(py, 0, h - 1))

    sample_lab = lab_small[py, px].astype(np.float64)
    dist = float(np.linalg.norm(sample_lab - _prev_road_mean_lab))

    if dist <= _COLOR_MATCH_TOLERANCE:
        return px, py, True
    return default_x, default_y, False


def update_dynamic_seed(road_mask_small, lab_small, road_found):
    global _prev_seed_small, _prev_road_mean_lab, _seed_fail_count

    if not road_found:
        _seed_fail_count += 1
        if _seed_fail_count >= _MAX_SEED_FAILURES:
            _prev_seed_small = None
            _prev_road_mean_lab = None
        return

    _seed_fail_count = 0
    h, w = road_mask_small.shape[:2]

    near_band_top = int(h * 0.7)
    band_mask = np.zeros_like(road_mask_small)
    band_mask[near_band_top:h, :] = road_mask_small[near_band_top:h, :]

    ys, xs = np.where(band_mask > 0)
    if xs.size == 0:
        ys, xs = np.where(road_mask_small > 0)
        if xs.size == 0:
            return

    cx, cy = int(xs.mean()), int(ys.mean())
    if _prev_seed_small is not None:
        cx = int(round(0.7 * _prev_seed_small[0] + 0.3 * cx))   # smooth the seed so it can't jitter
    _prev_seed_small = (cx, cy)
    _prev_road_mean_lab = lab_small[road_mask_small > 0].astype(np.float64).mean(axis=0)


# ---------------------------------------------------------------------- #
# Road mask (built entirely at low resolution)
# ---------------------------------------------------------------------- #
def color_dist(c1, c2):
    d = np.asarray(c1, dtype=np.float64) - np.asarray(c2, dtype=np.float64)
    return float(np.sqrt((COLOR_DIST_L_WEIGHT * d[0]) ** 2 + d[1] ** 2 + d[2] ** 2))


# Tracked road colors: a LIST (sunlit, shaded, ...) instead of a single color.
_ref_roads = []


def min_ref_dist(c):
    """Distance from color c to the NEAREST tracked road color (inf if none yet)."""
    if not _ref_roads:
        return float('inf')
    return min(color_dist(c, r) for r in _ref_roads)


def update_road_refs(c):
    """Blend c into the nearest tracked road color, or remember it as a new one."""
    c = np.asarray(c, dtype=np.float64)
    if _ref_roads:
        dists = [color_dist(c, r) for r in _ref_roads]
        i = int(np.argmin(dists))
        if dists[i] <= REF_NEW_TOL:
            _ref_roads[i] = (1.0 - ROAD_COLOR_EMA) * _ref_roads[i] + ROAD_COLOR_EMA * c
            return
    if len(_ref_roads) >= MAX_ROAD_REFS:
        _ref_roads.pop(0)                  # forget the oldest
    _ref_roads.append(c.copy())


def cluster_stats(lab_small, labels, k):
    """Pixel count and mean (unweighted) Lab color of every cluster.

    [v3] Only pixels BELOW the horizon are counted, so walls / ceiling
    can't skew the cluster colors or sizes."""
    y0 = horizon_row(labels.shape[0])
    flat = labels[y0:].ravel()
    counts = np.bincount(flat, minlength=k).astype(np.float64)
    means = np.zeros((k, 3), dtype=np.float64)
    for ch in range(3):
        means[:, ch] = np.bincount(flat, weights=lab_small[y0:, :, ch].ravel(), minlength=k) / np.maximum(counts, 1)
    return counts, means


def group_color(means, counts, label_set):
    idx = list(label_set)
    w = counts[idx]
    return (means[idx] * w[:, None]).sum(axis=0) / max(w.sum(), 1.0)


def make_seed_region(small_shape, center_x):
    """Bottom strip of the low-res frame, centered on center_x. -> (x0, x1, y0, y1)"""
    h, w = small_shape[:2]
    rw = max(4, int(w * SEED_REGION_W))
    rh = max(3, int(h * SEED_REGION_H))
    x0 = int(np.clip(center_x - rw // 2, 0, w - rw))
    return x0, x0 + rw, h - rh, h


_collect_debug = False
_dbg = {}

_prev_road_group = set()


def seed_labels_from_region(labels, region):
    """Cluster ids that cover >= SEED_LABEL_MIN_FRACTION of the seed region
    (always at least the dominant one)."""
    x0, x1, y0, y1 = region
    sub = labels[y0:y1, x0:x1].ravel()
    counts = np.bincount(sub, minlength=int(labels.max()) + 1)
    frac = counts / max(1, sub.size)
    chosen = {int(i) for i in np.where(frac >= SEED_LABEL_MIN_FRACTION)[0]}
    chosen |= {int(i) for i in _prev_road_group if i < frac.size and frac[i] >= 0.5 * SEED_LABEL_MIN_FRACTION}
    if not chosen:
        chosen = {int(counts.argmax())}
    return chosen


def pick_seed_component(labels_cc, region):
    """Connected component that overlaps the given region the most (0 = none)."""
    x0, x1, y0, y1 = region
    sub = labels_cc[y0:y1, x0:x1].ravel()
    sub = sub[sub > 0]
    if sub.size == 0:
        return 0
    return int(np.bincount(sub).argmax())


def fill_obstacle_holes(road_mask):
    """Make the road area solid: obstacles inside it no longer leave gaps."""
    if OBSTACLE_BRIDGE_PX > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OBSTACLE_BRIDGE_PX, OBSTACLE_BRIDGE_PX))
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, k, iterations=1)

    contours, hierarchy = cv2.findContours(road_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return road_mask

    max_area = HOLE_MAX_FRACTION * road_mask.size
    for c, hh in zip(contours, hierarchy[0]):
        if hh[3] != -1 and cv2.contourArea(c) <= max_area:
            cv2.drawContours(road_mask, [c], -1, 255, cv2.FILLED)
    return road_mask


def build_road_mask(labels, small_shape, region, seed_labels, counts, means, edge_barrier=None,
                    dilate_px=5, overlap_thresh=0.4, max_merge_iters=3):
    global _prev_road_group
    h, w = small_shape[:2]
    kernel3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_px, dilate_px))

    def apply_barrier(mask):
        # [v3] always cut at the horizon, even when the edge barrier is switched off
        mask = cut_above_horizon(mask)
        if edge_barrier is None:
            return mask
        return cv2.bitwise_and(mask, cv2.bitwise_not(edge_barrier))

    road_group_labels = set(seed_labels)
    unique_labels = [int(u) for u in np.unique(labels)]

    for _ in range(max_merge_iters):
        current_mask = np.isin(labels, list(road_group_labels)).astype(np.uint8) * 255
        current_mask = apply_barrier(current_mask)
        current_mask = cv2.morphologyEx(current_mask, cv2.MORPH_CLOSE, kernel3, iterations=1)

        num_cc, labels_cc, _, _ = cv2.connectedComponentsWithStats(current_mask, connectivity=8)
        seed_cc = pick_seed_component(labels_cc, region)
        if seed_cc == 0:
            break

        component_mask = np.where(labels_cc == seed_cc, 255, 0).astype(np.uint8)
        dilated = cv2.dilate(component_mask, dilate_kernel, iterations=1)

        merged_new = False
        road_color = group_color(means, counts, road_group_labels)
        for lbl in unique_labels:
            if lbl in road_group_labels:
                continue
            # Color gate: the cluster must be close to the current road group
            # OR match ANY tracked road color. Walls / grass match neither.
            if (color_dist(means[lbl], road_color) > MERGE_COLOR_TOLERANCE
                    and min_ref_dist(means[lbl]) > ROAD_COLOR_TOLERANCE):
                continue
            lbl_mask = (labels == lbl)
            lbl_count = int(lbl_mask.sum())
            if lbl_count == 0:
                continue
            overlap = int(np.logical_and(lbl_mask, dilated > 0).sum())
            thr = overlap_thresh * (0.6 if lbl in _prev_road_group else 1.0)   # hysteresis
            if overlap / lbl_count >= thr:
                road_group_labels.add(lbl)
                merged_new = True

        if not merged_new:
            break

    _prev_road_group = set(road_group_labels)
    same_color_group = np.isin(labels, list(road_group_labels)).astype(np.uint8) * 255
    same_color_group = apply_barrier(same_color_group)
    if _collect_debug:
        _dbg["pre_morph"] = same_color_group.copy()
    same_color_group = cv2.morphologyEx(same_color_group, cv2.MORPH_OPEN, kernel3, iterations=1)
    same_color_group = cv2.morphologyEx(same_color_group, cv2.MORPH_CLOSE, kernel5, iterations=1)
    same_color_group = cut_above_horizon(same_color_group)      # [v3] closing may bleed upward
    if _collect_debug:
        _dbg["post_morph"] = same_color_group.copy()
        _dbg["region"] = region

    num_labels_cc, labels_cc, _, _ = cv2.connectedComponentsWithStats(same_color_group, connectivity=8)
    seed_cc_label = pick_seed_component(labels_cc, region)

    if seed_cc_label == 0:
        return np.zeros((h, w), dtype=np.uint8), False

    road_mask = np.where(labels_cc == seed_cc_label, 255, 0).astype(np.uint8)
    if _collect_debug:
        _dbg["seed_component"] = road_mask.copy()
    if FILL_OBSTACLE_HOLES:
        road_mask = fill_obstacle_holes(road_mask)
    road_mask = cut_above_horizon(road_mask)                    # [v3]
    return road_mask, True


def detect_road_floodfill(small_bgr, edge_barrier, region, step, chroma_tol):
    """[v5] Region growing from the bottom-center of the frame.

    Every pixel is compared with its NEIGHBOR that is already part of the road
    (floating range), not with one global color:
      - a smooth brightness gradient (reflection on a polished floor) is accepted,
      - an abrupt step (floor -> pillar base, wall, table leg) stops the growth.
    Growth is blocked by the edge barrier and by everything above the horizon.
    Returns (road_mask_small, found)."""
    h, w = small_bgr.shape[:2]
    y0 = horizon_row(h)

    # extra smoothing kills floor texture (cracks, grains) but keeps real steps
    lab = cv2.cvtColor(cv2.medianBlur(small_bgr, 5), cv2.COLOR_BGR2LAB)

    # floor color right in front of the robot (median is robust to a stray object)
    x0, x1, ry0, ry1 = region
    ref_ab = np.median(lab[ry0:ry1, x0:x1].reshape(-1, 3)[:, 1:3].astype(np.float32), axis=0)

    # floodFill mask: nonzero = "do not fill here". It is 1 px larger than the image on each side.
    ff_mask = np.zeros((h + 2, w + 2), np.uint8)
    inner = ff_mask[1:-1, 1:-1]
    if edge_barrier is not None:
        inner[edge_barrier > 0] = 1
    inner[:y0] = 1                                   # never grow above the horizon

    step = int(max(1, step))
    chroma_step = max(2, int(round(step * 0.6)))
    diff = (step, chroma_step, chroma_step)          # allowed neighbor-to-neighbor change in (L, a, b)
    flags = 4 | cv2.FLOODFILL_MASK_ONLY | (255 << 8)  # 4-connected: no leaking through diagonal gaps

    sy = h - 2
    for sx in np.linspace(FLOOD_SEED_SPAN[0] * w, FLOOD_SEED_SPAN[1] * w, FLOOD_SEED_COUNT).astype(int):
        sx = int(np.clip(sx, 0, w - 1))
        if ff_mask[sy + 1, sx + 1] != 0:             # barrier, or already filled by an earlier seed
            continue
        if np.abs(lab[sy, sx, 1:3].astype(np.float32) - ref_ab).max() > chroma_tol:
            continue                                  # this seed sits on something that is not floor
        cv2.floodFill(lab, ff_mask, (sx, sy), 0, diff, diff, flags)

    filled = ((ff_mask[1:-1, 1:-1] == 255).astype(np.uint8)) * 255
    if not filled.any():
        return np.zeros((h, w), dtype=np.uint8), False

    # chroma gate: a fill that drifts slowly must not wander away from the floor's color
    far = np.abs(lab[:, :, 1:3].astype(np.float32) - ref_ab).max(axis=2) > chroma_tol
    filled[far] = 0
    if _collect_debug:
        _dbg["pre_morph"] = filled.copy()

    kernel3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    filled = cv2.morphologyEx(filled, cv2.MORPH_OPEN, kernel3, iterations=1)
    filled = cv2.morphologyEx(filled, cv2.MORPH_CLOSE, kernel5, iterations=1)   # heals 1-3 px barrier lines
    filled = cut_above_horizon(filled)
    if _collect_debug:
        _dbg["post_morph"] = filled.copy()
        _dbg["region"] = region

    _, labels_cc, _, _ = cv2.connectedComponentsWithStats(filled, connectivity=8)
    seed_cc = pick_seed_component(labels_cc, region)
    if seed_cc == 0:
        return np.zeros((h, w), dtype=np.uint8), False

    road_mask = np.where(labels_cc == seed_cc, 255, 0).astype(np.uint8)
    if _collect_debug:
        _dbg["seed_component"] = road_mask.copy()
    if FILL_OBSTACLE_HOLES:
        road_mask = fill_obstacle_holes(road_mask)
    road_mask = cut_above_horizon(road_mask)
    return road_mask, True


_jump_count = 0


def detect_road(labels, small_shape, region, edge_barrier, lab_small, counts, means, k):
    """Seed-region road detection, guarded by the tracked road colors.

    - Normal case: clusters under the seed strip that match a tracked road color
      (or all of them, before any color is tracked) are the road.
    - If NONE of the strip's clusters match any tracked road color (e.g. the strip
      is full of lawn), don't switch to them: use the clusters that DO match.
    - If that "different road" persists for ROAD_JUMP_FRAMES frames, believe it.
    - After every accepted frame, each major cluster of the mask updates the
      tracked colors, so sunlit AND shaded pavement are both remembered.
    """
    global _jump_count
    h, w = small_shape[:2]

    cand_labels = seed_labels_from_region(labels, region)
    seed_labels, comp_region = cand_labels, region

    if _ref_roads:
        matches = {l for l in cand_labels if counts[l] > 0 and min_ref_dist(means[l]) <= ROAD_COLOR_TOLERANCE}
        if matches:
            seed_labels = matches               # drops strip clusters that look like grass etc.
            _jump_count = 0
        else:
            _jump_count += 1
            if _jump_count >= ROAD_JUMP_FRAMES:
                _ref_roads.clear()              # persisted: accept the new surface
                _jump_count = 0
            else:
                guided = {i for i in range(k)
                          if counts[i] > 0 and min_ref_dist(means[i]) <= ROAD_COLOR_TOLERANCE}
                if not guided:
                    return np.zeros((h, w), dtype=np.uint8), False
                seed_labels = guided
                comp_region = (0, w, max(h // 2, horizon_row(h)), h)   # [v3] never below-horizon only
    else:
        _jump_count = 0

    road_mask, found = build_road_mask(labels, small_shape, comp_region, seed_labels,
                                       counts, means, edge_barrier)

    if found:
        in_mask = road_mask > 0
        total = int(in_mask.sum())
        if total > 0:
            for l in range(k):
                sel = in_mask & (labels == l)
                n = int(sel.sum())
                if n < REF_MIN_SHARE * total:
                    continue
                update_road_refs(lab_small[sel].astype(np.float64).mean(axis=0))
    return road_mask, found


# ---------------------------------------------------------------------- #
# Linear boundary line
# ---------------------------------------------------------------------- #
def extract_road_edge_points(road_mask, num_rows=40, min_row_pixels=3, ignore_border=EDGE_BORDER_IGNORE_PX,
                             top_trim=0.0):
    """Left/right road edge points per row.

    A point that lies on the image border (within ignore_border px) is NOT a
    road edge -- the road is simply cut by the frame -- so it is skipped."""
    h_img, w_img = road_mask.shape[:2]
    row_sums = road_mask.sum(axis=1)
    ys_nonzero = np.where(row_sums > 0)[0]
    if ys_nonzero.size == 0:
        return [], [], None, None

    y_bottom = int(ys_nonzero.max())
    y_top = int(ys_nonzero.min())
    # [v4] top_trim: skip the far end of the mask (its "edge" there is the horizon cut, not a road side)
    y_use_top = int(round(y_top + top_trim * (y_bottom - y_top)))
    row_ys = np.linspace(y_bottom, y_use_top, num_rows).astype(int)

    left_pts, right_pts = [], []
    for y in row_ys:
        xs = np.where(road_mask[y, :] > 0)[0]
        if xs.size >= min_row_pixels:
            xl, xr = int(xs.min()), int(xs.max())
            if xl > ignore_border:
                left_pts.append((y, xl))
            if xr < w_img - 1 - ignore_border:
                right_pts.append((y, xr))

    return left_pts, right_pts, y_bottom, y_top


def fit_edge_line(points, outlier_px=40):
    """Straight line x = m*y + b (degree 1)."""
    if len(points) < 2:
        return None

    ys = np.array([p[0] for p in points], dtype=np.float64)
    xs = np.array([p[1] for p in points], dtype=np.float64)

    coeffs = np.polyfit(ys, xs, 1)
    residuals = np.abs(np.polyval(coeffs, ys) - xs)
    keep = residuals <= outlier_px
    if keep.sum() >= 2 and keep.sum() < len(ys):
        coeffs = np.polyfit(ys[keep], xs[keep], 1)

    return coeffs


def compute_perspective_lines(road_mask):
    left_pts, right_pts, y_bottom, y_top = extract_road_edge_points(road_mask)
    if y_bottom is None or (y_bottom - y_top) < 10:
        return None, None, None, None

    left_coeffs = fit_edge_line(left_pts)
    right_coeffs = fit_edge_line(right_pts)
    return left_coeffs, right_coeffs, y_bottom, y_top


def draw_perspective_lines(img, left_coeffs, right_coeffs, y_bottom, y_top):
    if left_coeffs is None and right_coeffs is None:
        return img

    h, w = img.shape[:2]
    ys_draw = np.linspace(y_top, y_bottom, 60)

    for coeffs in (left_coeffs, right_coeffs):
        if coeffs is None:
            continue
        xs_draw = np.polyval(coeffs, ys_draw)
        pts = np.array([
            [int(np.clip(x, 0, w - 1)), int(y)]
            for x, y in zip(xs_draw, ys_draw)
        ], dtype=np.int32)
        cv2.polylines(img, [pts], isClosed=False, color=(0, 0, 255), thickness=3, lineType=cv2.LINE_AA)

    return img


def span_fill(road_mask):
    """Fill every row from its left-most to its right-most road pixel."""
    m = road_mask > 0
    h, w = m.shape
    any_row = m.any(axis=1)
    first = m.argmax(axis=1)
    last = w - 1 - m[:, ::-1].argmax(axis=1)
    cols = np.arange(w)[None, :]
    out = (cols >= first[:, None]) & (cols <= last[:, None]) & any_row[:, None]
    return out.astype(np.uint8) * 255


_mask_acc = None        # running average of the road mask (low-res, float 0..1)
_prev_out_full = None   # last output mask (full-res bool), for hysteresis


def smooth_mask_over_time(src_small, found, stability):
    """Exponential moving average of the low-res road mask."""
    global _mask_acc, _prev_out_full
    m = (src_small > 0).astype(np.float32) if found else np.zeros(src_small.shape[:2], np.float32)
    if _mask_acc is None or _mask_acc.shape != m.shape:
        _mask_acc = m.copy()
        _prev_out_full = None
    else:
        alpha = 1.0 / (1.0 + stability)
        _mask_acc += alpha * (m - _mask_acc)
    return _mask_acc


def threshold_with_hysteresis(acc, full_shape, stability):
    """Upscale the averaged mask, then binarize: ON above 0.6, OFF below 0.4."""
    global _prev_out_full
    H, W = full_shape
    acc_full = cv2.resize(acc, (W, H), interpolation=cv2.INTER_LINEAR)
    if stability > 0 and _prev_out_full is not None and _prev_out_full.shape == acc_full.shape:
        out = (acc_full > 0.6) | (_prev_out_full & (acc_full > 0.4))
    else:
        out = acc_full > 0.5
    _prev_out_full = out
    return out.astype(np.uint8) * 255


def wedge_mask_full(road_mask_small, full_shape, outlier_px=10):
    """Paint EVERYTHING between the left and right road lines (Fill Mode 2).

    Returns None if the lines can't be fitted (caller falls back to the mask)."""
    hs, ws = road_mask_small.shape[:2]
    H, W = full_shape

    left_pts, right_pts, y_bottom, y_top = extract_road_edge_points(
        road_mask_small, num_rows=40, min_row_pixels=2, ignore_border=1)
    if y_bottom is None or (y_bottom - y_top) < 4:
        return None
    left_c = fit_edge_line(left_pts, outlier_px=outlier_px)
    right_c = fit_edge_line(right_pts, outlier_px=outlier_px)
    if left_c is None or right_c is None:
        return None

    sy, sx = H / hs, W / ws
    ys_full = np.arange(H, dtype=np.float64)
    ys_small = (ys_full + 0.5) / sy - 0.5
    xl = np.polyval(left_c, ys_small) * sx
    xr = (np.polyval(right_c, ys_small) + 1.0) * sx - 1.0

    valid = (ys_full >= y_top * sy) & (xr > xl)
    cols = np.arange(W, dtype=np.float64)[None, :]
    inside = (cols >= xl[:, None]) & (cols <= xr[:, None]) & valid[:, None]
    return inside.astype(np.uint8) * 255


# ---------------------------------------------------------------------- #
# Debug window builders -- only called while SHOW_DEBUG is on
# ---------------------------------------------------------------------- #
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _gray_to_bgr(mask):
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def build_kmeans_view(frame, labels, k, counts):
    """1) original | Lab image | clusters filled with their mean color + red borders."""
    pw, ph = 400, 225
    small_frame = cv2.resize(frame, (pw, ph), interpolation=cv2.INTER_AREA)

    lab_img = cv2.cvtColor(small_frame, cv2.COLOR_BGR2LAB)

    hw, hh = pw // 2, ph // 2
    scores = np.stack([cv2.resize((labels == i).astype(np.float32), (hw, hh), interpolation=cv2.INTER_LINEAR)
                       for i in range(k)], axis=0)
    lf_half = scores.argmax(axis=0).astype(np.uint8)
    edge = np.zeros((hh, hw), dtype=np.uint8)
    edge[:, :-1] |= (lf_half[:, :-1] != lf_half[:, 1:]).astype(np.uint8)
    edge[:-1, :] |= (lf_half[:-1, :] != lf_half[1:, :]).astype(np.uint8)

    lf = cv2.resize(lf_half, (pw, ph), interpolation=cv2.INTER_NEAREST)
    flat = lf.ravel()
    cnt = np.maximum(np.bincount(flat, minlength=k), 1)
    means = np.stack([np.bincount(flat, weights=small_frame[:, :, ch].ravel(), minlength=k) / cnt
                      for ch in range(3)], axis=1).astype(np.uint8)
    mean_img = means[lf]
    mean_img[cv2.resize(edge, (pw, ph), interpolation=cv2.INTER_NEAREST) > 0] = (0, 0, 255)

    # show the horizon (rows above are never road and are ignored when fitting)
    gy = int(ph * _ground_top)
    cv2.line(mean_img, (0, gy), (pw - 1, gy), (255, 255, 255), 1)

    panels = []
    for img in (small_frame, lab_img, mean_img):
        p = np.ascontiguousarray(img)
        cv2.rectangle(p, (0, 0), (pw - 1, ph - 1), (110, 110, 110), 1)
        panels.append(p)
    return np.hstack(panels)


def build_seed_view(frame, small_shape):
    """2) Seed region -> which connected component of road-colored pixels it picks."""
    pw, ph = 480, 270
    hs, ws = small_shape[:2]
    vis = cv2.convertScaleAbs(cv2.resize(frame, (pw, ph), interpolation=cv2.INTER_AREA), alpha=0.45)

    def tint(mask_small, bgr, strength):
        nonlocal vis
        m = cv2.resize(mask_small, (pw, ph), interpolation=cv2.INTER_NEAREST)
        layer = cv2.addWeighted(vis, 1.0 - strength, np.full_like(vis, bgr), strength, 0)
        cv2.copyTo(layer, m, vis)

    post = _dbg.get("post_morph")
    comp = _dbg.get("seed_component")
    if post is not None:
        tint(post, (255, 120, 60), 0.5)
    if comp is not None:
        tint(comp, (0, 220, 0), 0.65)
    region = _dbg.get("region")
    if region is not None:
        x0, x1, y0, y1 = region
        sx, sy = pw / ws, ph / hs
        cv2.rectangle(vis, (int(x0 * sx), int(y0 * sy)), (int(x1 * sx) - 1, int(y1 * sy) - 1), (255, 255, 255), 2)
    return vis


def build_edge_morph_view(frame, edge_barrier_small):
    """3) Edge barrier -> cluster mask cut by the edges -> after open/close morphology."""
    pw, ph = 320, 180

    def panel(mask_small):
        if mask_small is None:
            img = np.zeros((ph, pw, 3), np.uint8)
        else:
            img = _gray_to_bgr(cv2.resize(mask_small, (pw, ph), interpolation=cv2.INTER_NEAREST))
        cv2.rectangle(img, (0, 0), (pw - 1, ph - 1), (110, 110, 110), 1)
        return img

    panels = [
        panel(edge_barrier_small),
        panel(_dbg.get("pre_morph")),
        panel(_dbg.get("post_morph")),
    ]
    return np.hstack(panels)


# ---------------------------------------------------------------------- #
# Road / robot reference geometry
# ---------------------------------------------------------------------- #
_prev_reference_center = None
_prev_lateral_error = None
_prev_heading_error = None

# last good left/right boundary lines and how many frames each has been missing
_prev_left_c = None
_prev_right_c = None
_left_missing = 0
_right_missing = 0


def resolve_edge_lines(left_c, right_c, w):
    """A side with too few REAL edge points (road runs off the frame edge) is
    handled instead of being fitted to the border:
      - hold the previous line for up to MAX_HOLD_FRAMES (avoids one-frame flicker);
      - after that use the frame border (x = 0 or x = w-1), i.e. 'road continues
        past the edge of the image'."""
    global _prev_left_c, _prev_right_c, _left_missing, _right_missing

    if left_c is not None:
        _prev_left_c, _left_missing = left_c, 0
    else:
        _left_missing += 1
        if _prev_left_c is not None and _left_missing <= MAX_HOLD_FRAMES:
            left_c = _prev_left_c
        else:
            left_c = np.array([0.0, 0.0])             # x = 0

    if right_c is not None:
        _prev_right_c, _right_missing = right_c, 0
    else:
        _right_missing += 1
        if _prev_right_c is not None and _right_missing <= MAX_HOLD_FRAMES:
            right_c = _prev_right_c
        else:
            right_c = np.array([0.0, w - 1.0])        # x = w - 1

    return left_c, right_c


def usable_edge_line(points, y_bottom, y_top):
    """[v4] Fit a side line only if it looks like a REAL side edge of the road:
    enough points, spread over enough of the road height, and not almost horizontal
    (a near-horizontal 'edge' is the far boundary of the mask, not a road side).
    Returns None otherwise -> the caller holds the previous line / uses the frame border."""
    if len(points) < MIN_EDGE_POINTS:
        return None
    ys = [p[0] for p in points]
    if (max(ys) - min(ys)) < MIN_EDGE_SPAN * max(1, (y_bottom - y_top)):
        return None
    coeffs = fit_edge_line(points)
    if coeffs is None or abs(float(coeffs[0])) > MAX_EDGE_SLOPE:
        return None
    return coeffs


def compute_reference_geometry(road_mask_full, smooth_alpha=0.25):
    """Calculate road/robot reference geometry and control errors."""
    global _prev_reference_center, _prev_lateral_error, _prev_heading_error

    h, w = road_mask_full.shape[:2]
    left_pts, right_pts, y_bottom, y_top = extract_road_edge_points(
        road_mask_full, num_rows=40, min_row_pixels=3, top_trim=EDGE_TOP_TRIM
    )

    if y_bottom is None or y_top is None or (y_bottom - y_top) < 10:
        return None

    left_c = usable_edge_line(left_pts, y_bottom, y_top)      # [v4]
    right_c = usable_edge_line(right_pts, y_bottom, y_top)    # [v4]
    left_c, right_c = resolve_edge_lines(left_c, right_c, w)

    center_c = (left_c + right_c) / 2.0

    lookahead_y = int(round(y_top + 0.70 * (y_bottom - y_top)))
    road_x = float(np.polyval(center_c, lookahead_y))

    robot_x = w / 2.0

    # Positive lateral error means the road reference is to the right.
    lateral_error_px = road_x - robot_x
    lateral_error_norm = lateral_error_px / max(1.0, w / 2.0)

    # Positive heading error means the road reference points to the right.
    x_far = float(np.polyval(center_c, y_top))
    x_near = float(np.polyval(center_c, y_bottom))
    dx = x_far - x_near
    dy_forward = float(y_bottom - y_top)
    heading_error_deg = float(np.degrees(np.arctan2(dx, dy_forward)))

    if _prev_reference_center is None:
        _prev_reference_center = center_c.copy()
        _prev_lateral_error = lateral_error_px
        _prev_heading_error = heading_error_deg
    else:
        alpha = float(np.clip(smooth_alpha, 0.01, 1.0))
        _prev_reference_center = (1.0 - alpha) * _prev_reference_center + alpha * center_c
        _prev_lateral_error = (1.0 - alpha) * _prev_lateral_error + alpha * lateral_error_px
        _prev_heading_error = (1.0 - alpha) * _prev_heading_error + alpha * heading_error_deg

        center_c = _prev_reference_center
        lateral_error_px = _prev_lateral_error
        lateral_error_norm = lateral_error_px / max(1.0, w / 2.0)
        heading_error_deg = _prev_heading_error
        road_x = float(np.polyval(center_c, lookahead_y))

    return {
        "left": left_c,
        "right": right_c,
        "center": center_c,
        "y_top": y_top,
        "y_bottom": y_bottom,
        "lookahead_y": lookahead_y,
        "road_x": road_x,
        "robot_x": robot_x,
        "lateral_error_px": float(lateral_error_px),
        "lateral_error_norm": float(lateral_error_norm),
        "heading_error_deg": float(heading_error_deg),
    }


def draw_reference_lines(img, reference):
    """Draw road boundaries, road center reference, and robot reference."""
    if reference is None:
        return img

    vis = img.copy()
    h, w = vis.shape[:2]
    y_top = int(np.clip(reference["y_top"], 0, h - 1))
    y_bottom = int(np.clip(reference["y_bottom"], 0, h - 1))
    ys = np.linspace(y_top, y_bottom, 80)

    # Red = road boundaries.
    for key in ("left", "right"):
        xs = np.polyval(reference[key], ys)
        pts = np.array([
            [int(np.clip(x, 0, w - 1)), int(np.clip(y, 0, h - 1))]
            for x, y in zip(xs, ys)
        ], dtype=np.int32)
        cv2.polylines(vis, [pts], False, (0, 0, 255), 3, cv2.LINE_AA)

    # Green = road reference line (center of the road).
    xs = np.polyval(reference["center"], ys)
    road_pts = np.array([
        [int(np.clip(x, 0, w - 1)), int(np.clip(y, 0, h - 1))]
        for x, y in zip(xs, ys)
    ], dtype=np.int32)
    cv2.polylines(vis, [road_pts], False, (0, 255, 0), 3, cv2.LINE_AA)

    # Blue = robot reference line (camera/robot center).
    robot_x = int(round(reference["robot_x"]))
    cv2.line(vis, (robot_x, y_bottom), (robot_x, y_top), (255, 0, 0), 3, cv2.LINE_AA)

    # Yellow = look-ahead target and lateral error.
    look_y = int(np.clip(reference["lookahead_y"], 0, h - 1))
    look_x = int(np.clip(reference["road_x"], 0, w - 1))
    cv2.circle(vis, (look_x, look_y), 7, (0, 255, 255), -1, cv2.LINE_AA)
    cv2.line(vis, (robot_x, look_y), (look_x, look_y), (0, 255, 255), 2, cv2.LINE_AA)

    return vis


def build_boundary_visualization(frame, road_mask_full, reference):
    """Debug view for road boundaries and reference lines."""
    vis = frame.copy()
    contours, _ = cv2.findContours(road_mask_full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, contours, -1, (0, 255, 255), 1)

    left_pts, right_pts, _, _ = extract_road_edge_points(road_mask_full, top_trim=EDGE_TOP_TRIM)   # [v4] shows the points actually used
    for (y, x) in left_pts:
        cv2.circle(vis, (x, y), 3, (255, 255, 0), -1)
    for (y, x) in right_pts:
        cv2.circle(vis, (x, y), 3, (255, 0, 255), -1)

    return cv2.resize(draw_reference_lines(vis, reference), (480, 270))


def build_debug_views(frame, labels, k, counts, small_shape,
                      edge_barrier_small, road_mask_full, small_bgr,
                      reference):
    builders = {
        "kmeans": lambda: build_kmeans_view(frame, labels, k, counts),
        "seed_cc": lambda: build_seed_view(frame, small_shape),
        "edge_morph": lambda: build_edge_morph_view(frame, edge_barrier_small),
        "boundary": lambda: build_boundary_visualization(frame, road_mask_full, reference),
    }
    return {
        key: build()
        for key, build in builders.items()
        if DEBUG_WINDOWS_ENABLED.get(key, False) and not (key == "kmeans" and labels is None)   # [v5] no k-means in Method 1
    }


def paint_road(frame, road_mask_full, road_found, fps=None, reference=None):
    result = frame.copy()

    if road_found:
        color_layer = np.zeros_like(frame)
        color_layer[road_mask_full > 0] = ROAD_PAINT_COLOR
        result = cv2.addWeighted(frame, 0.6, color_layer, 0.4, 0)

    result = draw_reference_lines(result, reference)
    return result


# ---------------------------------------------------------------------- #
# Per-frame pipeline
# ---------------------------------------------------------------------- #
def process_frame(frame, fps=None, show_debug=SHOW_DEBUG):
    k = max(2, cv2.getTrackbarPos('K Clusters', 'Trackbars'))
    downscale_factor = max(1, cv2.getTrackbarPos('Downscale Factor', 'Trackbars'))
    use_edge_barrier = cv2.getTrackbarPos('Use Edge Barrier', 'Trackbars') == 1
    use_dynamic_seed = cv2.getTrackbarPos('Use Dynamic Seed', 'Trackbars') == 1
    show_fps = cv2.getTrackbarPos('Show FPS', 'Trackbars') == 1
    fill_mode = cv2.getTrackbarPos('Fill Mode', 'Trackbars')

    h, w = frame.shape[:2]
    small_w = max(20, w // downscale_factor)
    small_h = max(20, h // downscale_factor)
    small = cv2.resize(frame, (small_w, small_h), interpolation=cv2.INTER_AREA)
    small_blur = cv2.GaussianBlur(small, (3, 3), 0)
    lab_small = cv2.cvtColor(small_blur, cv2.COLOR_BGR2LAB)

    global _light_w, _collect_debug, _ground_top
    global COLOR_DIST_L_WEIGHT, ROAD_COLOR_TOLERANCE, MERGE_COLOR_TOLERANCE   # [v4]
    _collect_debug = bool(show_debug) and (
        DEBUG_WINDOWS_ENABLED["seed_cc"] or DEBUG_WINDOWS_ENABLED["edge_morph"]
    )
    _dbg.clear()
    _light_w = cv2.getTrackbarPos('Brightness Weight', 'Trackbars') / 10.0
    _ground_top = cv2.getTrackbarPos('Horizon %', 'Trackbars') / 100.0
    # [v4] the same brightness weight is used for clustering AND for comparing colors
    COLOR_DIST_L_WEIGHT = max(0.05, _light_w)
    ROAD_COLOR_TOLERANCE = MERGE_COLOR_TOLERANCE = float(max(4, cv2.getTrackbarPos('Color Tolerance', 'Trackbars')))

    use_flood = cv2.getTrackbarPos('Method', 'Trackbars') == 1      # [v5]
    if use_flood:
        labels = counts = means = None                              # [v5] k-means is not used in Method 1
    else:
        labels, centers = compute_stable_clusters(small_blur, k)
        counts, means = cluster_stats(lab_small, labels, k)

    # [v5] Method 1 always seeds at the bottom-center: the ground right in front of the robot is road
    if use_dynamic_seed and not use_flood:
        seed_x, _, _ = get_dynamic_seed(lab_small, small.shape)
    else:
        seed_x = small_w // 2
    region = make_seed_region(small.shape, seed_x)

    edge_barrier_small = compute_edge_barrier(small_blur) if use_edge_barrier else None

    if use_flood:                                                   # [v5]
        road_mask_small, road_found = detect_road_floodfill(
            small_blur, edge_barrier_small, region,
            cv2.getTrackbarPos('Flood Step', 'Trackbars'),
            float(cv2.getTrackbarPos('Chroma Tol', 'Trackbars')),
        )
    else:
        road_mask_small, road_found = detect_road(
            labels, small.shape, region, edge_barrier_small,
            lab_small, counts, means, k
        )
    road_mask_small = cut_above_horizon(road_mask_small)                    # [v3] safety cut
    update_dynamic_seed(road_mask_small, lab_small, road_found)

    stability = cv2.getTrackbarPos('Stability', 'Trackbars')
    src_small = (
        span_fill(road_mask_small)
        if (road_found and fill_mode == 1)
        else road_mask_small
    )
    acc = smooth_mask_over_time(src_small, road_found, stability)

    road_mask_full = None
    if fill_mode == 2 and road_found:
        stable_small = (acc > 0.5).astype(np.uint8) * 255
        road_mask_full = wedge_mask_full(
            stable_small, (h, w),
            outlier_px=max(3, 40 // downscale_factor)
        )

    if road_mask_full is None:
        road_mask_full = threshold_with_hysteresis(acc, (h, w), stability)

    road_mask_full = cut_above_horizon(road_mask_full)                      # [v3] full-res cut
    road_found = bool(road_mask_full.any())

    # [v3] Sanity check: refuse to steer on a mask that is empty or covers everything
    if road_found:
        ground_px = max(1, (h - horizon_row(h)) * w)
        ratio = float((road_mask_full > 0).sum()) / ground_px
        if ratio < SANITY_MIN_GROUND_FRACTION or ratio > SANITY_MAX_GROUND_FRACTION:
            road_found = False
            road_mask_full[:] = 0

    reference = compute_reference_geometry(road_mask_full) if road_found else None

    result = paint_road(
        frame, road_mask_full, road_found,
        fps if show_fps else None, reference
    )

    if show_debug:
        # [v3] thin white line = horizon (tune with the 'Horizon %' trackbar)
        gy = horizon_row(h)
        cv2.line(result, (0, gy), (w - 1, gy), (255, 255, 255), 1)

    debug_views = (
        build_debug_views(
            frame, labels, k, counts, small.shape,
            edge_barrier_small, road_mask_full, small_blur,
            reference
        )
        if show_debug else None
    )

    return result, road_mask_full, debug_views, reference


# ---------------------------------------------------------------------- #
# Display helpers
# ---------------------------------------------------------------------- #
_debug_placed = False
_debug_scale = 1.0
_WIN_TITLE_H, _WIN_BORDER, _WIN_GAP = 34, 16, 6


def get_screen_size():
    """Screen size in the same units OpenCV uses for window positions."""
    try:
        import ctypes
        user32 = ctypes.windll.user32            # Windows only
        return int(user32.GetSystemMetrics(0)), int(user32.GetSystemMetrics(1))
    except Exception:
        return 1920, 1080


def compute_debug_layout(sizes, screen):
    """Place the processing windows next to each other without overlap."""
    sw, sh = screen
    x0, gap = DEBUG_AREA_X0, _WIN_GAP
    max_y = sh - 48

    def win(key, sc):
        w, h = sizes[key]
        return int(w * sc) + _WIN_BORDER, int(h * sc) + _WIN_TITLE_H

    pos, sc = {}, 1.0
    for sc in np.arange(1.0, 0.249, -0.05):
        pos, ok, y = {}, True, 0
        if "kmeans" in sizes:
            w, h = win("kmeans", sc)
            pos["kmeans"] = (x0, y)
            ok &= x0 + w <= sw
            y += h + gap
        x, row_h, col_y, col_w = x0, 0, y, 0
        for key in ("seed_cc", "boundary"):
            if key in sizes:
                w, h = win(key, sc)
                pos[key] = (x, col_y)
                col_y += h + gap
                col_w = max(col_w, w)
        if col_w:
            ok &= x + col_w <= sw
        row_h = max(row_h, col_y - y - gap if col_y > y else 0)
        y += row_h + gap if row_h else 0
        if "edge_morph" in sizes:
            w, h = win("edge_morph", sc)
            pos["edge_morph"] = (x0, y)
            ok &= x0 + w <= sw
            y += h
        if ok and y <= max_y:
            break
    return pos, float(sc)


def show_results(result, debug_views):
    global _debug_placed, _debug_scale
    cv2.imshow(MAIN_WINDOW, result)

    if debug_views is not None:
        first = TILE_DEBUG_WINDOWS and not _debug_placed
        if first:
            sizes = {key: (v.shape[1], v.shape[0]) for key, v in debug_views.items()}
            positions, _debug_scale = compute_debug_layout(sizes, get_screen_size())
        for key, name in zip(DEBUG_VIEW_KEYS, DEBUG_WINDOW_NAMES):
            if key in debug_views:
                img = debug_views[key]
                if _debug_scale < 1.0:
                    img = cv2.resize(img, None, fx=_debug_scale, fy=_debug_scale, interpolation=cv2.INTER_AREA)
                cv2.imshow(name, img)
        if first:
            for key, name in zip(DEBUG_VIEW_KEYS, DEBUG_WINDOW_NAMES):
                if key in positions:
                    cv2.moveWindow(name, *positions[key])
            _debug_placed = True


def toggle_debug():
    """Bound to the 'd' key: show / hide the four processing windows."""
    global SHOW_DEBUG, _debug_placed
    SHOW_DEBUG = not SHOW_DEBUG
    if not SHOW_DEBUG:
        for name in DEBUG_WINDOW_NAMES:
            try:
                cv2.destroyWindow(name)
            except cv2.error:
                pass
        _debug_placed = False
    print(f"Processing windows: {'ON' if SHOW_DEBUG else 'OFF'}")


def fit_frame(frame):
    return frame if ORIGINAL_RESOLUTION else cv2.resize(frame, (FRAME_W, FRAME_H))


class FpsMeter:
    """Smoothed FPS of the PROCESSING step."""
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.value = None

    def update(self, dt):
        inst = 1.0 / max(1e-6, dt)
        self.value = inst if self.value is None else (1 - self.alpha) * self.value + self.alpha * inst
        return self.value


class FrameReader:
    """Decodes + resizes video frames in a background thread (offline video testing)."""
    def __init__(self, path, queue_size=3):
        self.cap = cv2.VideoCapture(path)
        self.opened = self.cap.isOpened()
        self.fps = (self.cap.get(cv2.CAP_PROP_FPS) or 30) if self.opened else 30
        self.q = queue.Queue(maxsize=queue_size)
        self.stop_flag = threading.Event()
        self.decode_time = 0.0
        self.decoded = 0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _put(self, item):
        while not self.stop_flag.is_set():
            try:
                self.q.put(item, timeout=0.05)
                return
            except queue.Full:
                continue

    def _run(self):
        idx = 0
        try:
            while not self.stop_flag.is_set():
                t0 = time.perf_counter()
                ok, frame = self.cap.read()
                if not ok:
                    break
                frame = fit_frame(frame)
                self.decode_time += time.perf_counter() - t0
                self.decoded += 1
                self._put((idx, frame))
                idx += 1
        finally:
            self._put(None)          # end-of-video marker (always sent)

    def get(self):
        return self.q.get()

    def get_nowait(self):
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return "EMPTY"

    def stop(self):
        self.stop_flag.set()
        self.thread.join(timeout=1.0)
        self.cap.release()


def wait_until(t_target):
    """Sleep (while keeping the GUI alive) until perf_counter() == t_target.
    Returns True if 'q' was pressed."""
    while True:
        remaining = t_target - time.perf_counter()
        if remaining <= 0:
            return False
        key = cv2.waitKey(max(1, int(remaining * 1000))) & 0xFF
        if key == ord('q'):
            return True
        if key == ord('d'):
            toggle_debug()


# ---------------------------------------------------------------------- #
# ROS 2 entry point -- subscribes to a live sensor_msgs/CompressedImage topic
# ---------------------------------------------------------------------- #
class RoadDetectionNode(Node):
    """ROS 2 node that processes camera images with the road pipeline."""

    def __init__(self):
        super().__init__('road_detection_node')

        self.declare_parameter('image_topic', '/image_raw/compressed')
        self.declare_parameter('output_topic', '/road_detection/image')
        self.declare_parameter('mask_topic', '/road_detection/mask')
        self.declare_parameter('show_debug', False)
        self.declare_parameter('show_windows', True)

        image_topic = str(self.get_parameter('image_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        mask_topic = str(self.get_parameter('mask_topic').value)
        self.show_debug = bool(self.get_parameter('show_debug').value)
        self.show_windows = bool(self.get_parameter('show_windows').value)

        global SHOW_DEBUG
        SHOW_DEBUG = self.show_debug

        self.bridge = CvBridge()
        self.fps_meter = FpsMeter()

        compressed_output_topic = output_topic.rstrip('/') + '/compressed'

        self.image_pub = self.create_publisher(
            CompressedImage, compressed_output_topic, qos_profile_sensor_data
        )
        self.mask_pub = self.create_publisher(
            Image, mask_topic, qos_profile_sensor_data
        )
        # Publishers for road-following control
        self.lateral_error_pub = self.create_publisher(
            Float32,
            '/road_detection/lateral_error',
            10
        )

        self.heading_error_pub = self.create_publisher(
            Float32,
            '/road_detection/heading_error',
            10
        )

        self.valid_pub = self.create_publisher(
            Bool,
            '/road_detection/valid',
            10
        )

        self.image_sub = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, qos_profile_sensor_data
        )

        if not self.show_windows:
            self.get_logger().warning(
                'OpenCV windows are disabled. This version still uses the existing '
                'trackbar-based parameters, so run with show_windows:=true for the '
                'full interactive controls.'
            )

        self.get_logger().info(f'Subscribing to camera images: {image_topic}')
        self.get_logger().info(f'Publishing annotated image: {output_topic}')
        self.get_logger().info(f'Publishing road mask: {mask_topic}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = fit_frame(frame)

            start = time.perf_counter()
            result, road_mask, debug_views, reference = process_frame(
                frame, self.fps_meter.value, self.show_debug
            )

            # Publish whether valid road reference geometry is available
            valid_msg = Bool()
            valid_msg.data = reference is not None
            self.valid_pub.publish(valid_msg)

            # [v3] errors are published ONLY when the frame passed the sanity check
            if reference is not None:
                lateral_msg = Float32()
                lateral_msg.data = float(reference["lateral_error_norm"])
                self.lateral_error_pub.publish(lateral_msg)

                heading_msg = Float32()
                heading_msg.data = float(reference["heading_error_deg"])
                self.heading_error_pub.publish(heading_msg)

            elapsed = time.perf_counter() - start
            fps = self.fps_meter.update(elapsed)

            output_msg = self.bridge.cv2_to_compressed_imgmsg(result, dst_format='jpg')
            output_msg.header = msg.header
            self.image_pub.publish(output_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(road_mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            if self.show_windows:
                show_results(result, debug_views)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('d'):
                    toggle_debug()
                    self.show_debug = SHOW_DEBUG
                elif key == ord('q'):
                    self.get_logger().info('Quit key pressed; shutting down road detection.')
                    rclpy.shutdown()

        except Exception as exc:
            self.get_logger().error(f'Road image processing failed: {exc}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoadDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
