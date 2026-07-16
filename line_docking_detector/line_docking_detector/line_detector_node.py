#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
line_detector_node.py

전방 LiDAR 스캔에서 V자 형태(130°)의 두 라인을 검출하고,
두 라인의 연장선이 만나는 교차점(apex)에 TF를 발행합니다.
교차점의 방향은 로봇을 바라보도록 설정됩니다.

알고리즘:
1. LaserScan → 2D 포인트 변환 (전방 영역 필터링)
2. Sequential RANSAC로 다수 후보 라인 검출
3. 모든 라인 쌍에서 V자 각도(~130°) 조건 매칭
4. 두 직선의 연장선 교차점 계산
5. V자 이등분선 방향 → 로봇을 향하는 TF orientation 계산
6. TF broadcast + RViz MarkerArray + PoseStamped 발행
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from collections import deque
from typing import Optional, Tuple, List

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, TransformStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class LineDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('line_detector_node')
        self._declare_parameters()
        self._load_parameters()
        self._initialize_variables()
        self._initialize_ros_entities()
        self.get_logger().info(
            f'Line Detector Node started | '
            f'expected_angle={math.degrees(self.expected_angle):.1f}° | '
            f'scan_topic={self.scan_topic}')

    # ─────────────────────────── Parameters ───────────────────────────

    def _declare_parameters(self) -> None:
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('target_frame', 'docking_target')

        # V-shape 기대 각도 (두 라인 사이 각도)
        self.declare_parameter('expected_angle_deg', 130.0)
        self.declare_parameter('angle_tolerance_deg', 3.0)

        # 스캔 필터
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 1.3)
        self.declare_parameter('scan_angle_min_deg', -50.0)
        self.declare_parameter('scan_angle_max_deg', 50.0)

        # RANSAC 파라미터
        self.declare_parameter('ransac_iterations', 150)
        self.declare_parameter('ransac_threshold', 0.03)
        self.declare_parameter('min_line_points', 8)
        self.declare_parameter('max_candidate_lines', 5)

        # 포인트 연속성 (간격이 이 값 이상이면 별도 선으로 분리)
        self.declare_parameter('gap_threshold', 0.03)

        # Outlier 제거 (인접점 대비 거리가 이 값 이상이면 이상치로 판정)
        self.declare_parameter('outlier_dist', 0.05)

        # 최소 라인 길이 (이보다 짧은 라인은 무시)
        self.declare_parameter('min_line_length', 0.04)

        # Collinear 라인 병합 (두 라인이 유사하면 병합)
        self.declare_parameter('merge_angle_thresh_deg', 5.0)  # [deg] 방향 차이 임계값
        self.declare_parameter('merge_dist_thresh', 0.05)       # [m] 라인 간 거리 임계값

        # V-pair 두 라인 간 최대 거리 (너무 멀리 떨어진 라인 쌍은 거부)
        self.declare_parameter('max_line_pair_dist', 0.3)       # [m]

        # 스무딩 / 이력
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('history_size', 5)

        # Jump rejection (이전 대비 급격한 위치 변화 거부)
        self.declare_parameter('max_jump_distance', 0.15)   # [m] 이 값 이상 뛰면 거부
        self.declare_parameter('max_jump_yaw_deg', 20.0)    # [deg] 각도 점프 한계

        # Confidence gate (연속 N회 유사 검출 후 발행)
        self.declare_parameter('confidence_count', 3)
        self.declare_parameter('confidence_radius', 0.08)   # [m] 유사 판정 반경

    def _load_parameters(self) -> None:
        self.scan_topic: str = self.get_parameter('scan_topic').value
        self.laser_frame: str = self.get_parameter('laser_frame').value
        self.target_frame: str = self.get_parameter('target_frame').value

        self.expected_angle: float = math.radians(
            self.get_parameter('expected_angle_deg').value)
        self.angle_tolerance: float = math.radians(
            self.get_parameter('angle_tolerance_deg').value)

        self.min_range: float = self.get_parameter('min_range').value
        self.max_range: float = self.get_parameter('max_range').value
        self.scan_angle_min: float = math.radians(
            self.get_parameter('scan_angle_min_deg').value)
        self.scan_angle_max: float = math.radians(
            self.get_parameter('scan_angle_max_deg').value)

        self.ransac_iterations: int = self.get_parameter('ransac_iterations').value
        self.ransac_threshold: float = self.get_parameter('ransac_threshold').value
        self.min_line_points: int = self.get_parameter('min_line_points').value
        self.max_candidate_lines: int = self.get_parameter('max_candidate_lines').value

        self.gap_threshold: float = self.get_parameter('gap_threshold').value

        self.outlier_dist: float = self.get_parameter('outlier_dist').value
        self.min_line_length: float = self.get_parameter('min_line_length').value
        self.merge_angle_thresh: float = math.radians(
            self.get_parameter('merge_angle_thresh_deg').value)
        self.merge_dist_thresh: float = self.get_parameter('merge_dist_thresh').value

        self.max_line_pair_dist: float = self.get_parameter('max_line_pair_dist').value

        self.smoothing_alpha: float = self.get_parameter('smoothing_alpha').value
        self.history_size: int = self.get_parameter('history_size').value

        self.max_jump_distance: float = self.get_parameter('max_jump_distance').value
        self.max_jump_yaw: float = math.radians(
            self.get_parameter('max_jump_yaw_deg').value)
        self.confidence_count: int = self.get_parameter('confidence_count').value
        self.confidence_radius: float = self.get_parameter('confidence_radius').value

    # ─────────────────────────── Initialisation ───────────────────────

    def _initialize_variables(self) -> None:
        self.smoothed_x: Optional[float] = None
        self.smoothed_y: Optional[float] = None
        self.smoothed_yaw: Optional[float] = None
        self.detection_history: deque = deque(maxlen=self.history_size)
        self.last_valid_x: Optional[float] = None
        self.last_valid_y: Optional[float] = None
        self.last_valid_yaw: Optional[float] = None
        self.consecutive_good: int = 0          # confidence gate 카운터
        self.gate_passed: bool = False           # 최초 confidence 달성 여부

    def _initialize_ros_entities(self) -> None:
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, 'line_detection_markers', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, 'docking_target_pose', 10)

        # Subscriber (LiDAR 드라이버는 대부분 BEST_EFFORT로 발행)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, scan_qos)

    # ──────────────────────── Scan Callback ───────────────────────────

    def _scan_callback(self, msg: LaserScan) -> None:
            # 1) 스캔 → 2D 포인트 변환 (laser_line_extraction 스타일 거리 기반 필터링)
            points = self._scan_to_points(msg)
            if len(points) < self.min_line_points * 2:
                return

            # 1.2) 거리 기반 필터링 (min_range, max_range)
            points = self._filter_range(points)
            if len(points) < self.min_line_points * 2:
                return

            # 1.5) 이상치(outlier) 제거 (laser_line_extraction 스타일)
            points = self._filter_outliers(points)
            if len(points) < self.min_line_points * 2:
                return

            # 2) 다수 후보 라인 검출 (Sequential RANSAC)
            candidate_lines = self._find_candidate_lines(points)

            # 2.5) 짧은 라인 제거
            candidate_lines = self._filter_short_lines(candidate_lines)

            # 2.6) collinear 라인 병합
            candidate_lines = self._merge_collinear_lines(candidate_lines)

            # 2.7) 병합 후 라인 길이 재검증 (laser_line_extraction 스타일)
            candidate_lines = self._filter_short_lines(candidate_lines)

            if len(candidate_lines) < 2:
                return

            # 3) 최적 V-쌍 선택
            result = self._find_best_v_pair(candidate_lines)
            if result is None:
                return

            line1, pts1, line2, pts2, intersection = result

            # 4) 방향 계산 (V자 이등분선 → 로봇 방향)
            yaw = self._compute_orientation(line1, pts1, line2, pts2, intersection)

            # 4.5) Jump rejection — 이전 위치 대비 급격한 변화를 거부
            if not self._check_jump(intersection[0], intersection[1], yaw):
                self.get_logger().debug(
                    f'Jump rejected: ({intersection[0]:.3f}, {intersection[1]:.3f})')
                return

            # 4.6) 이력에 추가 → 중앙값 필터
            self.detection_history.append(
                (float(intersection[0]), float(intersection[1]), float(yaw)))
            mx, my, myaw = self._median_filter()

            # 4.7) Confidence gate — 연속 유사 검출이 충분해야 발행
            if not self._update_confidence_gate(mx, my):
                self.get_logger().debug(
                    f'Confidence gate: {self.consecutive_good}/{self.confidence_count}')
                return

            # 5) 시간 스무딩 (EMA)
            sx, sy, syaw = self._apply_smoothing(mx, my, myaw)

            # 6) TF / Pose / Marker 발행
            stamp = msg.header.stamp
            self._publish_tf(sx, sy, syaw, stamp)
            self._publish_pose(sx, sy, syaw, stamp)
            self._publish_markers(
                line1, pts1, line2, pts2,
                np.array([sx, sy]), syaw, stamp)

            self.get_logger().debug(
                f'Docking target: ({sx:.3f}, {sy:.3f}) yaw={math.degrees(syaw):.1f}°')
    def _filter_range(self, points: np.ndarray) -> np.ndarray:
        """
        laser_line_extraction 스타일 거리 기반 필터링
        """
        if len(points) == 0:
            return points
        dists = np.linalg.norm(points, axis=1)
        mask = (dists >= self.min_range) & (dists <= self.max_range)
        return points[mask]

    # ───────────────────── Scan → Points ──────────────────────────────

    def _scan_to_points(self, msg: LaserScan) -> np.ndarray:
        """LaserScan 메시지를 전방 영역의 2D 포인트 배열로 변환"""
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float64)

        # 유효 범위 필터
        valid = (
            (ranges > self.min_range) &
            (ranges < self.max_range) &
            (angles >= self.scan_angle_min) &
            (angles <= self.scan_angle_max) &
            np.isfinite(ranges)
        )

        angles = angles[valid]
        ranges = ranges[valid]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # 전방 포인트만 (x > 0.05)
        front_mask = x > 0.05
        return np.column_stack((x[front_mask], y[front_mask]))

    # ───────────────────── Outlier Filtering ──────────────────────────

    def _filter_outliers(self, points: np.ndarray) -> np.ndarray:
        """
        laser_line_extraction 스타일 이상치 제거.
        각 포인트의 두 최근접 이웃까지의 거리가 모두 outlier_dist 이상이면
        이상치로 간주하여 제거합니다.
        """
        if len(points) < 3:
            return points

        keep = np.ones(len(points), dtype=bool)
        for i in range(len(points)):
            if i == 0:
                d1 = np.linalg.norm(points[1] - points[0])
                d2 = np.linalg.norm(points[2] - points[0])
            elif i == len(points) - 1:
                d1 = np.linalg.norm(points[-1] - points[-2])
                d2 = np.linalg.norm(points[-1] - points[-3])
            else:
                d1 = np.linalg.norm(points[i] - points[i - 1])
                d2 = np.linalg.norm(points[i] - points[i + 1])

            if d1 > self.outlier_dist and d2 > self.outlier_dist:
                keep[i] = False

        return points[keep]

    # ───────────────────── RANSAC Line Detection ──────────────────────

    def _ransac_line_fit(
        self, points: np.ndarray
    ) -> Tuple[Optional[Tuple[np.ndarray, np.ndarray]], np.ndarray]:
        """
        RANSAC 알고리즘으로 하나의 직선을 검출합니다.
        Returns: (line, inlier_indices)
            line = (point_on_line, unit_direction)
        """
        n = len(points)
        if n < 2:
            return None, np.array([], dtype=int)

        best_inliers = np.array([], dtype=int)
        best_line = None

        for _ in range(self.ransac_iterations):
            idx = np.random.choice(n, 2, replace=False)
            p1, p2 = points[idx[0]], points[idx[1]]
            d = p2 - p1
            length = np.linalg.norm(d)
            if length < 1e-6:
                continue

            normal = np.array([-d[1], d[0]]) / length
            dists = np.abs((points - p1) @ normal)
            inliers = np.where(dists < self.ransac_threshold)[0]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                # 인라이어의 PCA로 더 정확한 방향 계산
                inlier_pts = points[inliers]
                centroid = inlier_pts.mean(axis=0)
                centered = inlier_pts - centroid
                cov = centered.T @ centered
                eigenvalues, eigenvectors = np.linalg.eigh(cov)
                direction = eigenvectors[:, np.argmax(eigenvalues)]
                best_line = (centroid, direction / np.linalg.norm(direction))

        return best_line, best_inliers

    def _split_by_gap(
        self,
        line: Tuple[np.ndarray, np.ndarray],
        points: np.ndarray
    ) -> List[np.ndarray]:
        """
        인라이어 포인트를 직선 방향으로 투영하여 정렬한 뒤,
        인접 포인트 간 거리가 gap_threshold 이상이면 별도 세그먼트로 분할합니다.
        """
        if len(points) < 2:
            return [points]

        p, d = line
        # 직선 방향으로 투영 → 정렬
        proj = (points - p) @ d
        order = np.argsort(proj)
        sorted_pts = points[order]

        # 인접 포인트 간 유클리드 거리 계산
        diffs = np.linalg.norm(np.diff(sorted_pts, axis=0), axis=1)

        # gap_threshold 이상 떨어진 곳에서 분할
        split_indices = np.where(diffs >= self.gap_threshold)[0] + 1
        segments = np.split(sorted_pts, split_indices)

        return segments

    def _find_candidate_lines(
        self, points: np.ndarray
    ) -> List[Tuple[Tuple[np.ndarray, np.ndarray], np.ndarray]]:
        """Sequential RANSAC로 라인 후보를 검출하고, gap 기준으로 세그먼트를 분할합니다."""
        lines = []
        remaining = points.copy()

        for _ in range(self.max_candidate_lines):
            if len(remaining) < self.min_line_points:
                break

            line, inlier_idx = self._ransac_line_fit(remaining)
            if line is None or len(inlier_idx) < self.min_line_points:
                break

            inlier_points = remaining[inlier_idx]

            # 연속성 기반 분할: gap_threshold 이상 떨어지면 별도 선으로 분리
            segments = self._split_by_gap(line, inlier_points)
            for seg in segments:
                if len(seg) >= self.min_line_points:
                    # 세그먼트별로 PCA 재계산하여 정확한 라인 파라미터 산출
                    centroid = seg.mean(axis=0)
                    centered = seg - centroid
                    cov = centered.T @ centered
                    eigenvalues, eigenvectors = np.linalg.eigh(cov)
                    direction = eigenvectors[:, np.argmax(eigenvalues)]
                    seg_line = (centroid, direction / np.linalg.norm(direction))
                    lines.append((seg_line, seg))

            # 인라이어 제거
            mask = np.ones(len(remaining), dtype=bool)
            mask[inlier_idx] = False
            remaining = remaining[mask]

        return lines

    # ───────────────────── Line Length Filtering ──────────────────────

    def _filter_short_lines(
        self, lines: List[Tuple[Tuple[np.ndarray, np.ndarray], np.ndarray]]
    ) -> List[Tuple[Tuple[np.ndarray, np.ndarray], np.ndarray]]:
        """min_line_length보다 짧은 라인을 제거합니다."""
        filtered = []
        for line, pts in lines:
            p, d = line
            proj = (pts - p) @ d
            length = np.max(proj) - np.min(proj)
            if length >= self.min_line_length:
                filtered.append((line, pts))
        return filtered

    # ───────────────────── Collinear Line Merging ─────────────────────

    def _merge_collinear_lines(
        self, lines: List[Tuple[Tuple[np.ndarray, np.ndarray], np.ndarray]]
    ) -> List[Tuple[Tuple[np.ndarray, np.ndarray], np.ndarray]]:
        """
        방향이 유사하고 거리가 가까운 collinear 라인 쌍을 병합합니다.
        laser_line_extraction의 mergeLines와 유사한 접근.
        """
        if len(lines) < 2:
            return lines

        merged = list(lines)  # 복사본
        changed = True

        while changed:
            changed = False
            new_merged = []
            used = set()

            for i in range(len(merged)):
                if i in used:
                    continue

                best_j = -1
                for j in range(i + 1, len(merged)):
                    if j in used:
                        continue

                    line_i, pts_i = merged[i]
                    line_j, pts_j = merged[j]

                    # 방향 유사성 검사
                    cos_angle = np.abs(line_i[1] @ line_j[1])
                    if cos_angle < math.cos(self.merge_angle_thresh):
                        continue

                    # 한 라인의 centroid에서 다른 라인까지의 직교 거리 검사
                    diff = line_j[0] - line_i[0]
                    normal_i = np.array([-line_i[1][1], line_i[1][0]])
                    perp_dist = abs(diff @ normal_i)
                    if perp_dist > self.merge_dist_thresh:
                        continue

                    # 두 라인의 끝점 사이 gap 검사
                    proj_i = (pts_i - line_i[0]) @ line_i[1]
                    proj_j = (pts_j - line_i[0]) @ line_i[1]
                    gap = max(np.min(proj_j) - np.max(proj_i),
                              np.min(proj_i) - np.max(proj_j))
                    if gap > self.gap_threshold:
                        continue

                    best_j = j
                    break

                if best_j >= 0:
                    # 두 라인 병합
                    _, pts_i = merged[i]
                    _, pts_j = merged[best_j]
                    combined_pts = np.vstack((pts_i, pts_j))

                    # PCA 재계산
                    centroid = combined_pts.mean(axis=0)
                    centered = combined_pts - centroid
                    cov = centered.T @ centered
                    eigenvalues, eigenvectors = np.linalg.eigh(cov)
                    direction = eigenvectors[:, np.argmax(eigenvalues)]
                    new_line = (centroid, direction / np.linalg.norm(direction))

                    new_merged.append((new_line, combined_pts))
                    used.add(i)
                    used.add(best_j)
                    changed = True
                else:
                    new_merged.append(merged[i])
                    used.add(i)

            merged = new_merged

        return merged

    # ───────────────────── V-Pair Selection ───────────────────────────

    def _angle_between_lines(
        self,
        line1: Tuple[np.ndarray, np.ndarray],
        line2: Tuple[np.ndarray, np.ndarray]
    ) -> float:
        """
        두 직선 사이의 V자 각도를 계산합니다.
        방향 벡터 사이의 예각을 구한 뒤 π에서 빼서 둔각(V-angle)을 반환합니다.
        """
        d1 = line1[1]
        d2 = line2[1]
        cos_acute = np.clip(np.abs(d1 @ d2), 0.0, 1.0)
        acute = math.acos(cos_acute)
        # V-angle = π - acute (e.g., 예각 50° → V각 130°)
        return math.pi - acute

    def _find_best_v_pair(
        self, lines: List
    ) -> Optional[Tuple]:
        """
        모든 라인 쌍 중 V-angle 조건을 만족하는 최적 쌍을 선택합니다.
        Returns: (line1, points1, line2, points2, intersection) or None
        """
        best = None
        best_score = float('inf')

        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                line1, pts1 = lines[i]
                line2, pts2 = lines[j]

                # V-angle 검사
                v_angle = self._angle_between_lines(line1, line2)
                angle_diff = abs(v_angle - self.expected_angle)
                if angle_diff > self.angle_tolerance:
                    continue

                # 두 라인 간 최소 거리 검사 (너무 멀면 거부)
                min_inter_line_dist = self._min_distance_between_lines(
                    line1, pts1, line2, pts2)
                if min_inter_line_dist > self.max_line_pair_dist:
                    continue

                # 교차점 계산
                intersection = self._find_line_intersection(line1, line2)
                if intersection is None:
                    continue

                # 교차점이 전방에 있어야 함
                if intersection[0] < 0.0:
                    continue

                # 합리적 거리 내에 있어야 함
                if np.linalg.norm(intersection) > self.max_range * 1.5:
                    continue

                # 두 라인의 인라이어 포인트가 교차점 근처에서 시작해야 함
                # (교차점에서 너무 먼 라인 쌍은 제외)
                dist1 = np.min(np.linalg.norm(pts1 - intersection, axis=1))
                dist2 = np.min(np.linalg.norm(pts2 - intersection, axis=1))
                proximity = dist1 + dist2

                # 스코어: 각도 차이 + 교차점까지 근접도
                score = angle_diff + 0.5 * proximity
                if score < best_score:
                    best_score = score
                    best = (line1, pts1, line2, pts2, intersection)

        return best

    @staticmethod
    def _min_distance_between_lines(
        line1: Tuple[np.ndarray, np.ndarray], pts1: np.ndarray,
        line2: Tuple[np.ndarray, np.ndarray], pts2: np.ndarray
    ) -> float:
        """
        두 라인의 끝점 4개 사이의 최소 거리를 반환합니다.
        라인을 구성하는 포인트의 투영 양 끝점을 사용하여
        두 라인이 물리적으로 얼마나 떨어져 있는지 측정합니다.
        """
        p1, d1 = line1
        proj1 = (pts1 - p1) @ d1
        ep1_a = p1 + np.min(proj1) * d1
        ep1_b = p1 + np.max(proj1) * d1

        p2, d2 = line2
        proj2 = (pts2 - p2) @ d2
        ep2_a = p2 + np.min(proj2) * d2
        ep2_b = p2 + np.max(proj2) * d2

        # 4개 끝점 조합 중 최소 거리
        d_min = min(
            np.linalg.norm(ep1_a - ep2_a),
            np.linalg.norm(ep1_a - ep2_b),
            np.linalg.norm(ep1_b - ep2_a),
            np.linalg.norm(ep1_b - ep2_b),
        )
        return float(d_min)

    # ───────────────────── Line Intersection ──────────────────────────

    def _find_line_intersection(
        self,
        line1: Tuple[np.ndarray, np.ndarray],
        line2: Tuple[np.ndarray, np.ndarray]
    ) -> Optional[np.ndarray]:
        """
        두 2D 직선의 교차점을 계산합니다.
        line = (point_on_line, unit_direction)
        p1 + t·d1 = p2 + s·d2  →  [d1 | -d2]·[t,s]^T = p2-p1
        """
        p1, d1 = line1
        p2, d2 = line2

        A = np.array([[d1[0], -d2[0]],
                       [d1[1], -d2[1]]])
        det = np.linalg.det(A)
        if abs(det) < 1e-8:
            return None  # 평행

        b = p2 - p1
        params = np.linalg.solve(A, b)
        t = params[0]
        intersection = p1 + t * d1
        return intersection

    # ───────────────────── Orientation (Bisector) ─────────────────────

    def _compute_orientation(
        self,
        line1: Tuple[np.ndarray, np.ndarray],
        pts1: np.ndarray,
        line2: Tuple[np.ndarray, np.ndarray],
        pts2: np.ndarray,
        intersection: np.ndarray
    ) -> float:
        """
        V자의 이등분선 방향을 계산하여, 교차점에서 로봇을 바라보는 yaw를 반환합니다.
        """
        # 각 라인에서 교차점으로부터 먼 쪽 끝점을 구합니다
        far1 = self._get_far_endpoint(line1, pts1, intersection)
        far2 = self._get_far_endpoint(line2, pts2, intersection)

        # 교차점에서 바깥쪽으로의 단위 벡터
        out1 = far1 - intersection
        n1 = np.linalg.norm(out1)
        out2 = far2 - intersection
        n2 = np.linalg.norm(out2)

        if n1 < 1e-6 or n2 < 1e-6:
            # fallback: 교차점 → 원점 방향
            return math.atan2(-intersection[1], -intersection[0])

        out1 /= n1
        out2 /= n2

        # 바깥 방향의 이등분선 (V자가 벌어지는 방향)
        bisector_out = out1 + out2
        bn = np.linalg.norm(bisector_out)
        if bn < 1e-6:
            return math.atan2(-intersection[1], -intersection[0])

        bisector_out /= bn

        # 로봇을 향하는 방향 = 반대
        facing_robot = -bisector_out
        yaw = math.atan2(facing_robot[1], facing_robot[0])
        return yaw

    def _get_far_endpoint(
        self,
        line: Tuple[np.ndarray, np.ndarray],
        points: np.ndarray,
        intersection: np.ndarray
    ) -> np.ndarray:
        """라인의 인라이어 포인트 중 교차점에서 가장 먼 투영 끝점"""
        p, d = line
        proj = (points - p) @ d
        start_pt = p + np.min(proj) * d
        end_pt = p + np.max(proj) * d

        if np.linalg.norm(end_pt - intersection) > np.linalg.norm(start_pt - intersection):
            return end_pt
        return start_pt

    # ───────────────────── Jump Rejection ─────────────────────────────

    def _check_jump(self, x: float, y: float, yaw: float) -> bool:
        """
        이전 유효 위치 대비 급격한 점프를 감지하여 거부합니다.
        첫 검출이거나 점프가 허용 범위 이내이면 True를 반환합니다.
        """
        if self.last_valid_x is None:
            # 첫 검출 — 기록만 하고 통과
            self.last_valid_x = float(x)
            self.last_valid_y = float(y)
            self.last_valid_yaw = float(yaw)
            return True

        dx = float(x) - self.last_valid_x
        dy = float(y) - self.last_valid_y
        dist = math.sqrt(dx * dx + dy * dy)

        dyaw = abs(math.atan2(
            math.sin(yaw - self.last_valid_yaw),
            math.cos(yaw - self.last_valid_yaw)))

        if dist > self.max_jump_distance or dyaw > self.max_jump_yaw:
            return False

        # 유효 → 기록 갱신
        self.last_valid_x = float(x)
        self.last_valid_y = float(y)
        self.last_valid_yaw = float(yaw)
        return True

    # ───────────────────── Median Filter ──────────────────────────────

    def _median_filter(self) -> Tuple[float, float, float]:
        """
        detection_history의 최근 N개 검출에서 x, y는 중앙값,
        yaw는 원형 중앙값(circular median)을 반환합니다.
        """
        xs = [h[0] for h in self.detection_history]
        ys = [h[1] for h in self.detection_history]
        yaws = [h[2] for h in self.detection_history]

        mx = float(np.median(xs))
        my = float(np.median(ys))

        # 원형 중앙값: 각 후보 yaw에 대해 모든 값과의 원형 거리 합이 최소인 것
        if len(yaws) == 1:
            myaw = yaws[0]
        else:
            best_yaw = yaws[0]
            best_cost = float('inf')
            for candidate in yaws:
                cost = sum(
                    abs(math.atan2(
                        math.sin(y - candidate),
                        math.cos(y - candidate)))
                    for y in yaws)
                if cost < best_cost:
                    best_cost = cost
                    best_yaw = candidate
            myaw = best_yaw

        return mx, my, myaw

    # ───────────────────── Confidence Gate ────────────────────────────

    def _update_confidence_gate(self, x: float, y: float) -> bool:
        """
        연속 N회 이상 유사 위치가 검출되어야 TF/Pose를 발행합니다.
        한 번 gate를 통과하면, 이후 jump rejection만으로 충분하므로
        연속 판정이 깨지지 않는 한 계속 발행합니다.
        """
        if self.gate_passed:
            return True

        if len(self.detection_history) < 2:
            self.consecutive_good = 1
            return False

        # 직전 값과 비교
        prev = self.detection_history[-2]
        dist = math.sqrt((x - prev[0]) ** 2 + (y - prev[1]) ** 2)

        if dist < self.confidence_radius:
            self.consecutive_good += 1
        else:
            self.consecutive_good = 1

        if self.consecutive_good >= self.confidence_count:
            self.gate_passed = True
            self.get_logger().info(
                f'Confidence gate passed after {self.consecutive_good} '
                f'consecutive detections')
            return True

        return False

    # ───────────────────── Temporal Smoothing ─────────────────────────

    def _apply_smoothing(
        self, x: float, y: float, yaw: float
    ) -> Tuple[float, float, float]:
        """지수 이동 평균(EMA)을 적용하여 노이즈를 줄입니다."""
        alpha = self.smoothing_alpha
        if self.smoothed_x is None:
            self.smoothed_x = x
            self.smoothed_y = y
            self.smoothed_yaw = yaw
        else:
            self.smoothed_x = alpha * x + (1 - alpha) * self.smoothed_x
            self.smoothed_y = alpha * y + (1 - alpha) * self.smoothed_y
            # 각도 스무딩 (래핑 처리)
            diff = yaw - self.smoothed_yaw
            diff = math.atan2(math.sin(diff), math.cos(diff))
            self.smoothed_yaw += alpha * diff

        return self.smoothed_x, self.smoothed_y, self.smoothed_yaw

    # ───────────────────── Docking Quaternion ─────────────────────────

    @staticmethod
    def _yaw_to_docking_quaternion(yaw: float) -> Tuple[float, float, float, float]:
        """
        yaw(이등분선 방향)를 도킹 타겟 프레임의 쿼터니언(x, y, z, w)으로 변환합니다.

        도킹 타겟 프레임 규약 (laser_frame 기준):
          X축 : V자 방향에서 왼쪽   → (sin(yaw), -cos(yaw), 0)
          Y축 : 위쪽 (up)           → (0, 0, 1)
          Z축 : 130° 중앙 반대 방향  → (-cos(yaw), -sin(yaw), 0)

        유도:
          q = q_base ⊗ q_Y180
          q_base: 기존 도킹 쿼터니언
          q_Y180: Y축 180° 회전 = (0, 1, 0, 0)
        """
        half = yaw / 2.0
        c = math.cos(half)
        s = math.sin(half)
        qx = 0.5 * (c + s)
        qy = 0.5 * (s - c)
        qz = 0.5 * (s - c)
        qw = 0.5 * (c + s)
        return qx, qy, qz, qw

    # ───────────────────── TF Broadcasting ────────────────────────────

    def _publish_tf(
        self, x: float, y: float, yaw: float, stamp
    ) -> None:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.laser_frame
        t.child_frame_id = self.target_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = self._yaw_to_docking_quaternion(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    # ───────────────────── Pose Publishing ────────────────────────────

    def _publish_pose(
        self, x: float, y: float, yaw: float, stamp
    ) -> None:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.laser_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = self._yaw_to_docking_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

    # ───────────────────── RViz Marker Publishing ─────────────────────

    def _project_line_endpoints(
        self,
        line: Tuple[np.ndarray, np.ndarray],
        points: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """인라이어를 직선 위에 투영한 양 끝점을 반환"""
        p, d = line
        proj = (points - p) @ d
        return p + np.min(proj) * d, p + np.max(proj) * d

    def _publish_markers(
        self,
        line1, pts1, line2, pts2,
        intersection: np.ndarray,
        yaw: float,
        stamp
    ) -> None:
        markers = MarkerArray()
        frame = self.laser_frame

        # ── 1) 라인 1 인라이어 포인트 (빨강) ──
        m_pts1 = self._make_points_marker(
            frame, stamp, 'line1_points', 0, pts1,
            ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0))
        markers.markers.append(m_pts1)

        # ── 2) 라인 2 인라이어 포인트 (파랑) ──
        m_pts2 = self._make_points_marker(
            frame, stamp, 'line2_points', 1, pts2,
            ColorRGBA(r=0.2, g=0.2, b=1.0, a=1.0))
        markers.markers.append(m_pts2)

        # ── 3) 라인 1 피팅 직선 (주황) ──
        ep1_start, ep1_end = self._project_line_endpoints(line1, pts1)
        m_line1 = self._make_line_marker(
            frame, stamp, 'line1_fit', 2,
            ep1_start, ep1_end,
            ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0), width=0.015)
        markers.markers.append(m_line1)

        # ── 4) 라인 2 피팅 직선 (하늘) ──
        ep2_start, ep2_end = self._project_line_endpoints(line2, pts2)
        m_line2 = self._make_line_marker(
            frame, stamp, 'line2_fit', 3,
            ep2_start, ep2_end,
            ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0), width=0.015)
        markers.markers.append(m_line2)

        # ── 5) 연장선 1 → 교차점 (주황 반투명) ──
        closer1 = self._closer_endpoint(ep1_start, ep1_end, intersection)
        m_ext1 = self._make_line_marker(
            frame, stamp, 'ext_line1', 4,
            closer1, intersection,
            ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.4), width=0.008)
        markers.markers.append(m_ext1)

        # ── 6) 연장선 2 → 교차점 (하늘 반투명) ──
        closer2 = self._closer_endpoint(ep2_start, ep2_end, intersection)
        m_ext2 = self._make_line_marker(
            frame, stamp, 'ext_line2', 5,
            closer2, intersection,
            ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.4), width=0.008)
        markers.markers.append(m_ext2)

        # ── 7) 교차점 구 (녹색) ──
        m_sphere = Marker()
        m_sphere.header.frame_id = frame
        m_sphere.header.stamp = stamp
        m_sphere.ns = 'intersection'
        m_sphere.id = 6
        m_sphere.type = Marker.SPHERE
        m_sphere.action = Marker.ADD
        m_sphere.pose.position.x = float(intersection[0])
        m_sphere.pose.position.y = float(intersection[1])
        m_sphere.pose.position.z = 0.0
        m_sphere.scale.x = 0.08
        m_sphere.scale.y = 0.08
        m_sphere.scale.z = 0.08
        m_sphere.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        markers.markers.append(m_sphere)

        # ── 8) 방향 화살표 (노랑 — 로봇을 바라봄) ──
        m_arrow = Marker()
        m_arrow.header.frame_id = frame
        m_arrow.header.stamp = stamp
        m_arrow.ns = 'direction'
        m_arrow.id = 7
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.pose.position.x = float(intersection[0])
        m_arrow.pose.position.y = float(intersection[1])
        m_arrow.pose.position.z = 0.0
        m_arrow.pose.orientation.z = math.sin(yaw / 2.0)
        m_arrow.pose.orientation.w = math.cos(yaw / 2.0)
        m_arrow.scale.x = 0.30   # 길이
        m_arrow.scale.y = 0.035  # 폭
        m_arrow.scale.z = 0.035  # 높이
        m_arrow.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        markers.markers.append(m_arrow)

        # ── 9) 이등분선 표시 (흰색 점선 느낌) ──
        bisector_len = 0.4
        bx = float(intersection[0]) + bisector_len * math.cos(yaw)
        by = float(intersection[1]) + bisector_len * math.sin(yaw)
        m_bisect = self._make_line_marker(
            frame, stamp, 'bisector', 8,
            intersection, np.array([bx, by]),
            ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.6), width=0.01)
        markers.markers.append(m_bisect)

        self.marker_pub.publish(markers)

    # ── Marker helpers ──

    @staticmethod
    def _make_points_marker(
        frame: str, stamp, ns: str, mid: int,
        points: np.ndarray, color: ColorRGBA
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.scale.y = 0.02
        m.color = color
        for p in points:
            m.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))
        return m

    @staticmethod
    def _make_line_marker(
        frame: str, stamp, ns: str, mid: int,
        p_start: np.ndarray, p_end: np.ndarray,
        color: ColorRGBA, width: float = 0.01
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = width
        m.color = color
        m.points.append(
            Point(x=float(p_start[0]), y=float(p_start[1]), z=0.0))
        m.points.append(
            Point(x=float(p_end[0]), y=float(p_end[1]), z=0.0))
        return m

    @staticmethod
    def _closer_endpoint(
        ep_start: np.ndarray, ep_end: np.ndarray, target: np.ndarray
    ) -> np.ndarray:
        """target에 더 가까운 끝점 반환"""
        if np.linalg.norm(ep_start - target) < np.linalg.norm(ep_end - target):
            return ep_start
        return ep_end


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
