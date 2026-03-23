import argparse
import os
from typing import Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares


def load_image(path: str) -> np.ndarray:
	img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
	if img is None:
		raise FileNotFoundError(f"Cannot read image: {path}")
	return img


def save_image(path: str, img: np.ndarray) -> None:
	ok = cv2.imwrite(path, img)
	if not ok:
		raise RuntimeError(f"Failed to write image: {path}")


def orb_match_points(
	img_a: np.ndarray,
	img_b: np.ndarray,
	nfeatures: int = 3500,
	ratio_thresh: float = 0.75,
) -> Tuple[np.ndarray, np.ndarray]:
	orb = cv2.ORB_create(nfeatures=nfeatures)
	kp_a, des_a = orb.detectAndCompute(img_a, None)
	kp_b, des_b = orb.detectAndCompute(img_b, None)

	if des_a is None or des_b is None:
		raise RuntimeError("ORB failed: descriptor is None on one of the images")

	matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
	knn_ab = matcher.knnMatch(des_a, des_b, k=2)
	knn_ba = matcher.knnMatch(des_b, des_a, k=2)

	best_ba = {}
	for pair in knn_ba:
		if len(pair) < 2:
			continue
		m, n = pair
		if m.distance < ratio_thresh * n.distance:
			best_ba[m.queryIdx] = m.trainIdx

	good = []
	for pair in knn_ab:
		if len(pair) < 2:
			continue
		m, n = pair
		if m.distance < ratio_thresh * n.distance:
			# Keep only symmetric matches to suppress outliers.
			if m.trainIdx in best_ba and best_ba[m.trainIdx] == m.queryIdx:
				good.append(m)

	if len(good) < 8:
		raise RuntimeError(f"Not enough good matches: {len(good)} < 8")

	pts_a = np.float32([kp_a[m.queryIdx].pt for m in good])
	pts_b = np.float32([kp_b[m.trainIdx].pt for m in good])

	F, inlier_mask = cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_RANSAC, 1.5, 0.99)
	if F is not None and inlier_mask is not None:
		mask = inlier_mask.ravel().astype(bool)
		pts_a_in = pts_a[mask]
		pts_b_in = pts_b[mask]
		if len(pts_a_in) >= 8:
			pts_a, pts_b = pts_a_in, pts_b_in

	return pts_a, pts_b


def recover_translation_direction(
	pts_ref: np.ndarray,
	pts_query: np.ndarray,
	K: np.ndarray,
) -> np.ndarray:
	E, mask = cv2.findEssentialMat(
		pts_ref,
		pts_query,
		K,
		method=cv2.RANSAC,
		prob=0.999,
		threshold=1.0,
	)
	if E is None:
		raise RuntimeError("findEssentialMat failed: E is None")

	_, _, t, _ = cv2.recoverPose(E, pts_ref, pts_query, K, mask=mask)
	t = t.reshape(3)
	norm_t = np.linalg.norm(t)
	if norm_t < 1e-12:
		raise RuntimeError("recoverPose returned near-zero translation")
	return t / norm_t


def project_points(
	points_w: np.ndarray,
	K: np.ndarray,
	C: np.ndarray,
	R: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
	# Camera coordinates: Pc = R * (Pw - C)
	pw_minus_c = points_w - C.reshape(1, 3)
	points_c = (R @ pw_minus_c.T).T
	z = points_c[:, 2]
	valid = z > 1e-6
	points_c = points_c[valid]

	uv_h = (K @ points_c.T).T
	uv = uv_h[:, :2] / uv_h[:, 2:3]
	return uv, valid


def draw_patch(img: np.ndarray, x: int, y: int, patch: np.ndarray) -> None:
	h, w = img.shape
	ph, pw = patch.shape
	hh, hw = ph // 2, pw // 2
	if x - hw < 0 or y - hh < 0 or x + hw >= w or y + hh >= h:
		return
	roi = img[y - hh : y + hh + 1, x - hw : x + hw + 1]
	np.maximum(roi, patch, out=roi)


def render_synthetic_view(
	points_w: np.ndarray,
	patches: np.ndarray,
	K: np.ndarray,
	C: np.ndarray,
	R: np.ndarray,
	width: int,
	height: int,
) -> np.ndarray:
	img = np.zeros((height, width), dtype=np.uint8)
	uv, valid = project_points(points_w, K, C, R)
	valid_patches = patches[valid]

	for (u, v), patch in zip(uv, valid_patches):
		ui = int(round(u))
		vi = int(round(v))
		if 6 <= ui < width - 6 and 6 <= vi < height - 6:
			draw_patch(img, ui, vi, patch)

	img = cv2.GaussianBlur(img, (3, 3), 0)
	noise = np.random.normal(0, 3, size=img.shape).astype(np.int16)
	img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
	return img


def generate_synthetic_demo_data(
	out_dir: str,
	width: int = 960,
	height: int = 640,
	num_points: int = 1200,
	seed: int = 42,
) -> dict:
	np.random.seed(seed)
	os.makedirs(out_dir, exist_ok=True)

	fx, fy = 700.0, 700.0
	cx, cy = width / 2.0, height / 2.0
	K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

	# Ground truth 2D camera positions on XOY plane.
	xL_gt, yL_gt = -0.2, 0.5
	xR_gt, yR_gt = 0.4, 0.5
	xD, yD = 1.4, 1.0
	baseline = float(np.hypot(xL_gt - xR_gt, yL_gt - yR_gt))

	# 3D camera centers (all with z=0) and identical orientation.
	C_left = np.array([xL_gt, yL_gt, 0.0], dtype=np.float64)
	C_right = np.array([xR_gt, yR_gt, 0.0], dtype=np.float64)
	C_query = np.array([xD, yD, 0.0], dtype=np.float64)
	R = np.eye(3, dtype=np.float64)

	# Static 3D points in front of cameras.
	X = np.random.uniform(-2.0, 2.5, size=(num_points, 1))
	Y = np.random.uniform(-1.8, 1.8, size=(num_points, 1))
	Z = np.random.uniform(4.0, 8.0, size=(num_points, 1))
	points_w = np.hstack([X, Y, Z]).astype(np.float64)
	patch_base = np.random.randint(0, 200, size=(num_points, 9, 9), dtype=np.uint8)
	center_boost = np.zeros((num_points, 9, 9), dtype=np.uint8)
	center_boost[:, 4, 4] = 255
	center_boost[:, 4, 3:6] = 220
	center_boost[:, 3:6, 4] = 220
	patches = np.maximum(patch_base, center_boost)

	left_img = render_synthetic_view(points_w, patches, K, C_left, R, width, height)
	right_img = render_synthetic_view(points_w, patches, K, C_right, R, width, height)
	query_img = render_synthetic_view(points_w, patches, K, C_query, R, width, height)

	left_path = os.path.join(out_dir, "left_demo.png")
	right_path = os.path.join(out_dir, "right_demo.png")
	query_path = os.path.join(out_dir, "query_demo.png")
	save_image(left_path, left_img)
	save_image(right_path, right_img)
	save_image(query_path, query_img)

	return {
		"left_path": left_path,
		"right_path": right_path,
		"query_path": query_path,
		"K": K,
		"xD": xD,
		"yD": yD,
		"B": baseline,
		"gt": np.array([xL_gt, yL_gt, xR_gt, yR_gt], dtype=np.float64),
	}


def planar_equations(
	vars_xy: np.ndarray,
	tL_xy: np.ndarray,
	tR_xy: np.ndarray,
	CD_xy: np.ndarray,
	baseline: float,
) -> np.ndarray:
	xL, yL, xR, yR = vars_xy
	xD, yD = CD_xy
	tLx, tLy = tL_xy
	tRx, tRy = tR_xy

	# (1) Left-query collinearity
	eq1 = tLy * (xD - xL) - tLx * (yD - yL)
	# (2) Right-query collinearity
	eq2 = tRy * (xD - xR) - tRx * (yD - yR)
	# (3) Baseline length
	eq3 = (xL - xR) ** 2 + (yL - yR) ** 2 - baseline ** 2
	# (4) Extra constraint: assume stereo baseline is parallel to world X axis
	# so left and right cameras share the same Y on the plane.
	eq4 = yL - yR

	return np.array([eq1, eq2, eq3, eq4], dtype=np.float64)


def solve_planar_positions(
	tL: np.ndarray,
	tR: np.ndarray,
	CD_xy: np.ndarray,
	baseline: float,
	initial_guess: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, float]:
	tL_xy = tL[:2]
	tR_xy = tR[:2]

	if np.linalg.norm(tL_xy) < 1e-9 or np.linalg.norm(tR_xy) < 1e-9:
		raise RuntimeError(
			"Projected translation direction on XOY is near zero. "
			"Geometry is degenerate for planar localization."
		)

	CD_xy = CD_xy.astype(np.float64)
	baseline = float(baseline)

	# Multi-start helps avoid poor local minima for noisy direction estimates.
	seeds = [
		initial_guess.astype(np.float64),
		np.array([CD_xy[0] - baseline, CD_xy[1], CD_xy[0], CD_xy[1]], dtype=np.float64),
		np.array([CD_xy[0], CD_xy[1], CD_xy[0] + baseline, CD_xy[1]], dtype=np.float64),
		np.array([CD_xy[0] - baseline * 0.5, CD_xy[1], CD_xy[0] + baseline * 0.5, CD_xy[1]], dtype=np.float64),
	]

	best_res = None
	best_cost = np.inf
	for x0 in seeds:
		res = least_squares(
			planar_equations,
			x0=x0,
			args=(tL_xy, tR_xy, CD_xy, baseline),
			method="trf",
			loss="huber",
			f_scale=0.05,
			max_nfev=300,
		)
		cost = float(np.linalg.norm(res.fun))
		if not res.success:
			continue

		xL, _, xR, _ = res.x
		# Keep semantic consistency: right camera should be on +X side of left camera.
		if xR <= xL:
			continue

		if cost < best_cost:
			best_cost = cost
			best_res = res

	if best_res is None:
		raise RuntimeError(
			"Solver failed for all initial guesses (or all fell into mirrored branch)."
		)

	xL, yL, xR, yR = best_res.x
	center = np.array([(xL + xR) * 0.5, (yL + yR) * 0.5], dtype=np.float64)
	return best_res.x, center, best_cost


def parse_k_matrix(k_text: str) -> np.ndarray:
	vals = [float(x) for x in k_text.split(",")]
	if len(vals) != 9:
		raise ValueError("K must contain 9 comma-separated values")
	K = np.array(vals, dtype=np.float64).reshape(3, 3)
	return K


def main() -> None:
	parser = argparse.ArgumentParser(
		description="Minimal stereo planar localization from left/right/query images"
	)
	parser.add_argument("--left", help="Path to left image")
	parser.add_argument("--right", help="Path to right image")
	parser.add_argument("--query", help="Path to query image")
	parser.add_argument(
		"--K",
		help="Camera intrinsics K as 9 comma-separated values (row-major)",
	)
	parser.add_argument("--xD", type=float, help="Known query camera x")
	parser.add_argument("--yD", type=float, help="Known query camera y")
	parser.add_argument("--B", type=float, help="Stereo baseline length")
	parser.add_argument(
		"--init",
		default="0,0,1,0",
		help="Initial guess xL,yL,xR,yR (default: 0,0,1,0)",
	)
	parser.add_argument(
		"--demo",
		action="store_true",
		help="Run self-contained demo using generated synthetic images",
	)
	parser.add_argument(
		"--demo_out",
		default="demo_data",
		help="Directory to save generated demo images",
	)
	parser.add_argument("--seed", type=int, default=42, help="Random seed for demo")
	args = parser.parse_args()

	if args.demo:
		meta = generate_synthetic_demo_data(args.demo_out, seed=args.seed)
		left_path = meta["left_path"]
		right_path = meta["right_path"]
		query_path = meta["query_path"]
		K = meta["K"]
		CD_xy = np.array([meta["xD"], meta["yD"]], dtype=np.float64)
		B = float(meta["B"])
		gt = meta["gt"]
	else:
		required = [args.left, args.right, args.query, args.K, args.xD, args.yD, args.B]
		if any(v is None for v in required):
			raise ValueError(
				"Non-demo mode requires --left --right --query --K --xD --yD --B"
			)
		left_path = args.left
		right_path = args.right
		query_path = args.query
		K = parse_k_matrix(args.K)
		CD_xy = np.array([args.xD, args.yD], dtype=np.float64)
		B = float(args.B)
		gt = None

	init = np.array([float(v) for v in args.init.split(",")], dtype=np.float64)
	if init.size != 4:
		raise ValueError("--init must be 4 comma-separated values")

	left_img = load_image(left_path)
	right_img = load_image(right_path)
	query_img = load_image(query_path)

	pts_l, pts_q1 = orb_match_points(left_img, query_img)
	tL = recover_translation_direction(pts_l, pts_q1, K)

	pts_r, pts_q2 = orb_match_points(right_img, query_img)
	tR = recover_translation_direction(pts_r, pts_q2, K)

	solution, center, residual_norm = solve_planar_positions(tL, tR, CD_xy, B, init)
	xL, yL, xR, yR = solution

	print("tL (direction) =", tL)
	print("tR (direction) =", tR)
	print("xL, yL, xR, yR =", solution)
	print("Stereo center (x, y) =", center)
	print("Residual norm =", residual_norm)

	if gt is not None:
		print("Ground truth [xL, yL, xR, yR] =", gt)
		print("Abs error =", np.abs(solution - gt))

		xL_gt, yL_gt, xR_gt, yR_gt = gt
		tL_oracle = np.array([CD_xy[0] - xL_gt, CD_xy[1] - yL_gt, 0.0], dtype=np.float64)
		tR_oracle = np.array([CD_xy[0] - xR_gt, CD_xy[1] - yR_gt, 0.0], dtype=np.float64)
		tL_oracle /= np.linalg.norm(tL_oracle)
		tR_oracle /= np.linalg.norm(tR_oracle)

		oracle_sol, oracle_center, oracle_res = solve_planar_positions(
			tL_oracle, tR_oracle, CD_xy, B, init
		)
		print("Oracle solve [xL, yL, xR, yR] =", oracle_sol)
		print("Oracle center (x, y) =", oracle_center)
		print("Oracle residual norm =", oracle_res)
		print("Oracle abs error =", np.abs(oracle_sol - gt))
		print("Demo images saved to:")
		print("  ", left_path)
		print("  ", right_path)
		print("  ", query_path)


if __name__ == "__main__":
	main()
