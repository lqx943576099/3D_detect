"""
实时展示：将色彩图与深度着色图叠加并不断更新，方便观察对齐效果。
- 依赖：已编译 astra_py（astra_py.so）及 OpenCV（cv2）。
- 在相机打开失败或帧无效时会自动退出并打印错误码。
用法：
  python3 test.py [--mirror]
  --mirror    : 在启动相机前通过 wrapper 请求底层启用流镜像（若后端支持 OpenNI2）
"""
import sys
import time
import numpy as np

try:
    import astra_py
except Exception as e:
    print("ERROR: 无法导入 astra_py 模块（请先编译并确保 astra_py.so 在 PYTHONPATH 或当前目录）")
    print(e)
    sys.exit(2)

try:
    import cv2
except Exception as e:
    print("ERROR: OpenCV (cv2) 未安装")
    print(e)
    sys.exit(2)

# 可选：Open3D / Matplotlib 用于点云可视化（默认优先 Matplotlib，避免某些平台 Open3D 崩溃）
import os
backend_pref = os.environ.get('ASTRA_PC_BACKEND', 'mpl').lower()  # 'open3d' 或 'mpl'
o3d = None
if backend_pref == 'open3d':
    try:
        import open3d as o3d
    except Exception:
        o3d = None
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
except Exception:
    plt = None


def _set_view_open3d(vis):
    try:
        ctr = vis.get_view_control()
        ctr.set_front([0.0, 0.0, 1.0])   # 朝 +Z 方向
        ctr.set_up([0.0, -1.0, 0.0])     # Y 向下为上方向
        ctr.set_lookat([0.0, 0.0, 1000.0])  # 注视 Z 正方向上的一点
        ctr.set_zoom(0.5)
    except Exception:
        pass


def _set_view_mpl(ax, pts):
    try:
        # 视角方向：从原点看向 +Z
        ax.view_init(elev=0, azim=-90)
        # 设置范围并将 Y 轴倒置以匹配相机坐标 (Y 向下)
        if pts is not None and pts.shape[0] > 0:
            xmin = float(np.nanmin(pts[:, 0])); xmax = float(np.nanmax(pts[:, 0]))
            ymin = float(np.nanmin(pts[:, 1])); ymax = float(np.nanmax(pts[:, 1]))
            zmin = float(np.nanmin(pts[:, 2])); zmax = float(np.nanmax(pts[:, 2]))
            if not np.isfinite(xmin) or not np.isfinite(xmax): xmin, xmax = -1.0, 1.0
            if not np.isfinite(ymin) or not np.isfinite(ymax): ymin, ymax = -1.0, 1.0
            if not np.isfinite(zmin) or not np.isfinite(zmax) or zmax <= 0: zmin, zmax = 0.0, 1.0
            ax.set_xlim(xmin, xmax)
            ax.set_ylim(ymax, ymin)  # 反转 Y 轴
            ax.set_zlim(0.0, zmax)
    except Exception:
        pass


def parse_frames(out):
    color = depth = shaded = cloud = None
    try:
        if isinstance(out, (tuple, list)):
            idx = 0
            if len(out) > idx:
                color = np.array(out[idx]); idx += 1
            if len(out) > idx:
                maybe_depth = out[idx]; idx += 1
                if isinstance(maybe_depth, (tuple, list)) and len(maybe_depth) >= 1:
                    depth = np.array(maybe_depth[0])
                    if len(maybe_depth) > 1:
                        shaded = np.array(maybe_depth[1])
                else:
                    depth = np.array(maybe_depth)
            if len(out) > idx:
                cloud = np.array(out[idx])
        else:
            color = np.array(out)
    except Exception:
        return None, None, None, None
    return color, depth, shaded, cloud


def depth_to_colormap(depth, shaded=None, clip_percentile=99.5):
    # 返回 BGR uint8 colormap (H,W,3)
    if shaded is not None:
        arr = np.array(shaded)
        if arr.ndim == 2:
            arr = np.stack([arr] * 3, axis=-1)
        if arr.dtype != np.uint8:
            arr = np.clip(arr, 0, 255).astype(np.uint8)
        if arr.shape[2] == 3:
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr
    if depth is None:
        return None
    d = np.array(depth)
    if d.ndim == 3 and d.shape[2] == 3:
        d = d[..., 2]
    valid = np.isfinite(d) & (d != 0)
    if not np.any(valid):
        return None
    vals = d[valid].astype(np.float64)
    vmax = np.percentile(vals, clip_percentile) if vals.size > 0 else np.nanmax(vals)
    if not np.isfinite(vmax) or vmax <= 0:
        vmax = np.nanmax(vals)
    if not np.isfinite(vmax) or vmax <= 0:
        return None
    depth_norm = np.zeros_like(d, dtype=np.uint8)
    depth_norm[valid] = np.clip((d[valid] / float(vmax) * 255.0), 0, 255).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)  # BGR
    return depth_color


def center_point_3d_from_cloud(cloud):
    if cloud is None:
        return None
    if cloud.ndim == 3 and cloud.shape[2] == 3:
        H, W, _ = cloud.shape
        cx = W // 4
        cy = H // 4
        v = cloud[cy, cx]
        x, y, z = float(v[0]), float(v[1]), float(v[2])
        if not np.isfinite(z) or z == 0:
            return None
        return (cx, cy, x, y, z)
    return None


def wait_for_valid_frame(cam, timeout=5.0, poll_delay=0.1):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            # 使用显式关键字参数以匹配新版 get_frames(want_color, want_depth, want_shaded, want_cloud)
            out = cam.get_frames(want_color=True, want_depth=True, want_shaded=True, want_cloud=True)
        except Exception:
            time.sleep(poll_delay)
            continue
        color, depth, shaded, cloud = parse_frames(out)
        if color is None or (depth is None and shaded is None and cloud is None):
            time.sleep(poll_delay)
            continue
        try:
            if color is None or color.size == 0:
                time.sleep(poll_delay)
                continue
            if cloud is not None and cloud.ndim == 3:
                zs = cloud[..., 2]
                if not np.isfinite(zs).any() or np.all(zs == 0):
                    time.sleep(poll_delay)
                    continue
            elif depth is not None:
                dd = depth
                if dd.ndim == 3 and dd.shape[2] == 3:
                    dd = dd[..., 2]
                if not (np.isfinite(dd).any() and not np.all(dd == 0)):
                    time.sleep(poll_delay)
                    continue
        except Exception:
            time.sleep(poll_delay)
            continue
        return color, depth, shaded, cloud
    return None


def main(interval=5.0, poll_delay=0.03, depth_alpha=0.6, upscale=1, mirror=False,
         hole_fill=True, hole_iters=1):
    cam = astra_py.AstraCamera()
    # 若需要通过 wrapper 请求底层镜像/空洞填充，先设置（必须在 start() 前）
    try:
        if mirror:
            try:
                cam.set_mirror(True)
            except Exception:
                # set_mirror 可能不存在（向后兼容），忽略异常
                pass
        if hole_fill:
            try:
                cam.set_hole_filling(True, int(hole_iters))
            except Exception:
                # 若后端不支持该接口，忽略
                pass

        cam.start()
    except Exception as e:
        print("启动相机失败：", e)
        try:
            cam.stop()
        except Exception:
            pass
        sys.exit(2)

    print("相机已 start，等待首帧验证（5s）...")
    first = wait_for_valid_frame(cam, timeout=5.0, poll_delay=0.1)
    if first is None:
        print("ERROR: 未在超时时间内收到有效帧，退出。")
        try:
            cam.stop()
        except Exception:
            pass
        sys.exit(3)

    print("开始实时展示窗口，按 'q' 或 Ctrl+C 退出。")
    last_print = 0.0
    # 点云可视化相关状态
    use_open3d = (backend_pref == 'open3d') and (o3d is not None)
    use_mpl = (not use_open3d) and (plt is not None)
    pc_vis = None  # Open3D Visualizer 实例或 Matplotlib Figure
    pc_geom = None # Open3D PointCloud 实例或 Matplotlib scatter
    sample_step = 4  # 下采样步长，减小渲染负担

    def build_pointcloud_from_arrays(cloud_arr, color_arr, step=4):
        if cloud_arr is None:
            return None, None
        arr = np.array(cloud_arr)
        if arr.ndim != 3 or arr.shape[2] != 3:
            return None, None
        H, W, _ = arr.shape
        z = arr[..., 2]
        valid = np.isfinite(z) & (z != 0)
        if not valid.any():
            return None, None
        # 下采样采样点
        ys = np.arange(0, H, step)
        xs = np.arange(0, W, step)
        grid_y, grid_x = np.meshgrid(ys, xs, indexing='ij')
        coords = np.stack([grid_y.ravel(), grid_x.ravel()], axis=1)
        mask = valid[coords[:, 0], coords[:, 1]]
        coords = coords[mask]
        pts = arr[coords[:, 0], coords[:, 1], :].reshape(-1, 3)
        cols = None
        if color_arr is not None:
            img = np.array(color_arr)
            if img.ndim == 2:
                img = np.stack([img] * 3, axis=-1)
            if img.dtype != np.uint8:
                img = np.clip(img, 0, 255).astype(np.uint8)
            cols = img[coords[:, 0], coords[:, 1], :].reshape(-1, 3).astype(np.float32) / 255.0
        return pts, cols

    try:
        while True:
            try:
                # 明确请求 shaded（深度着色图）和点云
                out = cam.get_frames(want_color=True, want_depth=True, want_shaded=False, want_cloud=True)
            except Exception as e:
                print("ERROR: get_frames 抛出异常，退出：", e)
                try:
                    cam.stop()
                except Exception:
                    pass
                sys.exit(4)

            color, depth, shaded, cloud = parse_frames(out)
            if color is None:
                print("ERROR: 未获取到 color 帧，退出。")
                try:
                    cam.stop()
                except Exception:
                    pass
                sys.exit(5)

            # prepare color image for cv2 (BGR uint8)
            img_color = np.array(color)
            if img_color.ndim == 2:
                img_color = np.stack([img_color] * 3, axis=-1)
            if img_color.dtype != np.uint8:
                img_color = np.clip(img_color, 0, 255).astype(np.uint8)
            if img_color.shape[2] == 3:
                # assume SDK gives RGB -> convert to BGR
                try:
                    img_color_bgr = cv2.cvtColor(img_color, cv2.COLOR_RGB2BGR)
                except Exception:
                    img_color_bgr = img_color
            else:
                img_color_bgr = img_color

            # get depth colormap (BGR)
            # 优先使用 shaded（如果 SDK 返回），否则尝试用 depth 或 cloud 的 Z 通道
            depth_vis = depth_to_colormap(depth, shaded=shaded)
            if depth_vis is None and cloud is not None:
                try:
                    depth_from_cloud = cloud[..., 2]
                    depth_vis = depth_to_colormap(depth_from_cloud, shaded=None)
                except Exception:
                    depth_vis = None

            # ensure depth_vis and img_color_bgr have same size for blending
            base = img_color_bgr
            if depth_vis is not None:
                if depth_vis.dtype != np.uint8:
                    depth_vis = np.clip(depth_vis, 0, 255).astype(np.uint8)
                # resize depth to match color if needed
                if depth_vis.shape[:2] != base.shape[:2]:
                    depth_vis = cv2.resize(depth_vis, (base.shape[1], base.shape[0]), interpolation=cv2.INTER_LINEAR)

                # alpha blend: depth semi-transparent over color
                alpha = float(depth_alpha)
                try:
                    overlay = cv2.addWeighted(base, 1.0 - alpha, depth_vis, alpha, 0)
                except Exception:
                    overlay = base.copy()
            else:
                overlay = base.copy()

            # 放大 overlay（长宽均等比例放大 upscale 倍）
            try:
                h, w = overlay.shape[:2]
                display = cv2.resize(overlay, (w * int(upscale), h * int(upscale)), interpolation=cv2.INTER_LINEAR)
            except Exception:
                display = overlay

            # 显示窗口
            cv2.imshow("astra_overlay (q to quit)", display)

            # every `interval` seconds also print center 3D if cloud available
            now = time.time()
            if now - last_print >= interval:
                last_print = now
                cp = center_point_3d_from_cloud(cloud)
                if cp is None:
                    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] WARNING: 中心像素对应点无效（z=0 或 非有限）")
                else:
                    cx, cy, X, Y, Z = cp
                    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] center(px={cx},{cy}) -> 3D (X={X:.4f}, Y={Y:.4f}, Z={Z:.4f})")

            key = cv2.waitKey(max(1, int(poll_delay * 1000))) & 0xFF
            if key == ord('q'):
                print("用户请求退出（q）")
                break
            elif key == ord('p'):
                # 切换点云可视化窗口
                if pc_vis is None:
                    print("开启点云可视化窗口（按 p 关闭）")
                    pts, cols = build_pointcloud_from_arrays(cloud, color, step=sample_step)
                    if pts is None or pts.shape[0] == 0:
                        print("WARNING: 当前没有可视化的点云数据")
                    else:
                        if use_open3d:
                            try:
                                pc_geom = o3d.geometry.PointCloud()
                                pc_geom.points = o3d.utility.Vector3dVector(pts)
                                if cols is not None:
                                    pc_geom.colors = o3d.utility.Vector3dVector(cols)
                                pc_vis = o3d.visualization.Visualizer()
                                if not pc_vis.create_window("Astra PointCloud"):
                                    raise RuntimeError("Open3D create_window failed")
                                pc_vis.add_geometry(pc_geom)
                                _set_view_open3d(pc_vis)
                            except Exception as _:
                                print("WARNING: Open3D 初始化失败，回退到 Matplotlib。可设置 ASTRA_PC_BACKEND=mpl 强制使用 Matplotlib。")
                                pc_vis = None
                                pc_geom = None
                                use_open3d = False
                                if plt is not None:
                                    use_mpl = True
                                else:
                                    use_mpl = False
                        if use_mpl and pc_vis is None:
                            try:
                                plt.ion()
                                pc_vis = plt.figure("Astra PointCloud")
                                ax = pc_vis.add_subplot(111, projection='3d')
                                scat = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=(cols if cols is not None else None), s=0.5, depthshade=False)
                                ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                                pc_geom = (ax, scat)
                                _set_view_mpl(ax, pts)
                                plt.show()
                            except Exception:
                                print("WARNING: Matplotlib 初始化失败，无法显示点云。")
                        elif use_mpl:
                            plt.ion()
                            pc_vis = plt.figure("Astra PointCloud")
                            ax = pc_vis.add_subplot(111, projection='3d')
                            scat = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=(cols if cols is not None else None), s=0.5, depthshade=False)
                            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                            pc_geom = (ax, scat)
                            _set_view_mpl(ax, pts)
                            plt.show()
                        else:
                            print("WARNING: 未检测到可用的3D可视化库（Open3D / Matplotlib）")
                else:
                    print("关闭点云可视化窗口")
                    if use_open3d:
                        try:
                            pc_vis.destroy_window()
                        except Exception:
                            pass
                    elif use_mpl:
                        try:
                            plt.close(pc_vis)
                        except Exception:
                            pass
                    pc_vis = None
                    pc_geom = None

            # 若点云窗口开启，则实时更新
            if pc_vis is not None and cloud is not None:
                pts, cols = build_pointcloud_from_arrays(cloud, color, step=sample_step)
                if pts is not None and pts.shape[0] > 0:
                    if use_open3d and o3d is not None and isinstance(pc_vis, o3d.visualization.Visualizer):
                        try:
                            pc_geom.points = o3d.utility.Vector3dVector(pts)
                            if cols is not None:
                                pc_geom.colors = o3d.utility.Vector3dVector(cols)
                            pc_vis.update_geometry(pc_geom)
                            pc_vis.poll_events()
                            pc_vis.update_renderer()
                        except Exception:
                            # 如果更新失败，关闭窗口，避免崩溃
                            try:
                                pc_vis.destroy_window()
                            except Exception:
                                pass
                            pc_vis = None
                            pc_geom = None
                    elif use_mpl and pc_geom is not None:
                        try:
                            ax, scat = pc_geom
                            # 增量更新而非重建，降低崩溃概率和开销
                            xs = pts[:, 0]; ys = pts[:, 1]; zs = pts[:, 2]
                            scat._offsets3d = (xs, ys, zs)
                            if cols is not None:
                                scat.set_facecolors(cols)
                            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                            _set_view_mpl(ax, pts)
                            pc_geom = (ax, scat)
                            # 以非阻塞方式刷新
                            pc_vis.canvas.draw_idle()
                            pc_vis.canvas.flush_events()
                        except Exception:
                            pass
    except KeyboardInterrupt:
        print("\n手动中断，正在停止相机...")
    finally:
        try:
            cam.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        # 确保 Matplotlib 清理，避免 Tk 回调报错
        try:
            if plt is not None:
                plt.ioff()
                plt.close('all')
        except Exception:
            pass
        print("相机已停止，窗口已关闭。")
    return 0


if __name__ == "__main__":
    rc = main(mirror=False, hole_fill=True, hole_iters=1)
    sys.exit(rc)