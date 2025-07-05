import asyncio
import threading
import json
import logging
import cv2
import numpy as np
import time

# 確定事項: pip install eclipse-zenoh でインストールし、import zenoh で利用する
import zenoh

from fastapi import FastAPI
from fastapi.websockets import WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from scipy.spatial.transform import Rotation
from starlette.websockets import WebSocketState

from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

# --- 基本設定 ---
logging.basicConfig(level=logging.INFO)
LOG = logging.getLogger(__name__)
ZENOH_ROUTER_ADDRESS = "tcp/localhost:7447"

# --- ROSトピック定義 ---
MAP_TOPIC = "map"
TF_TOPIC = "tf"
TF_STATIC_TOPIC = "tf_static"

# --- メッセージ型とトピックキーの対応辞書 ---
ROS_MSG_TYPES = {
    MAP_TOPIC: OccupancyGrid,
    TF_TOPIC: TFMessage,
    TF_STATIC_TOPIC: TFMessage
}

# --- グローバル変数 ---
connected_clients: list[WebSocket] = []
app_data = None

class AppData:
    """アプリケーションのデータをまとめて管理するクラス"""
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_map_img = None
        self.map_info = None
        self.transforms = {}
        self.robot_pose_on_map = None

def make_transform_matrix(translation, rotation_matrix):
    """並進ベクトルと回転行列から4x4の同次変換行列を作成する"""
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation
    return T

def update_robot_pose():
    """TFツリーからロボットのmap座標系での姿勢を計算する"""
    global app_data
    with app_data.lock:
        required_keys = ["base_footprint->odom", "odom->map"]
        if not all(key in app_data.transforms for key in required_keys):
            return
        try:
            trans1, rot1 = app_data.transforms[required_keys[0]]
            T_footprint_odom = make_transform_matrix(trans1, rot1)
            trans2, rot2 = app_data.transforms[required_keys[1]]
            T_odom_map = make_transform_matrix(trans2, rot2)
            T_footprint_map = T_odom_map @ T_footprint_odom
            x, y = T_footprint_map[0, 3], T_footprint_map[1, 3]
            r = Rotation.from_matrix(T_footprint_map[:3, :3])
            yaw = r.as_euler('zyx', degrees=False)[0]
            app_data.robot_pose_on_map = [x, y, yaw]
        except Exception as e:
            LOG.error(f"Error calculating robot pose: {e}", exc_info=True)

def zenoh_thread(loop: asyncio.AbstractEventLoop):
    """Zenohのセッションと購読を管理するスレッド"""

    def on_message(msg: zenoh.Sample):
        """Zenohサブスクライバのコールバック関数"""
        key_expr = str(msg.key_expr)
        LOG.info(f"Received message on topic: {key_expr}")
        try:
            msg_type = ROS_MSG_TYPES.get(key_expr)
            if not msg_type: return
            
            data = deserialize_message(bytes(msg.payload), msg_type)

            if key_expr == MAP_TOPIC:
                with app_data.lock:
                    app_data.map_info = data.info
                    grid_data = np.array(data.data, dtype=np.int8).reshape((data.info.height, data.info.width))
                    
                    #
                    # ★★★ ここからが今回の主要な修正箇所です ★★★
                    #
                    # 1. まず、未探索(-1)領域の色で画像全体を初期化
                    map_img = np.full((data.info.height, data.info.width, 3), 128, dtype=np.uint8)

                    # 2. 通行可能(0)領域を白に設定
                    map_img[grid_data == 0] = [255, 255, 255]

                    # 3. 障害物(100)領域を黒に設定
                    map_img[grid_data == 100] = [0, 0, 0]

                    # 4. 障害物の確率(1-99)領域をグレースケールで設定
                    prob_mask = (grid_data > 0) & (grid_data < 100)
                    if np.any(prob_mask):
                        prob_values = grid_data[prob_mask]
                        # 確率が高いほど黒(0)に近くなるように色を計算
                        # 色 = 255 * (1 - 確率/100)
                        gray_values = (255 * (1 - prob_values / 100.0)).astype(np.uint8)
                        # 計算したグレースケール値を画像の該当箇所に適用
                        map_img[prob_mask] = np.stack([gray_values]*3, axis=-1)
                    
                    # ★★★ ここまでが今回の主要な修正箇所です ★★★

                    # ユーザーリクエストにより画像を左右反転
                    #flipped_map_img = cv2.flip(map_img, 1)
                    app_data.latest_map_img = map_img
                
                # メインスレッドのイベントループでコルーチンを実行
                asyncio.run_coroutine_threadsafe(broadcast_map(), loop)

            elif key_expr in [TF_TOPIC, TF_STATIC_TOPIC]:
                with app_data.lock:
                    for tf in data.transforms:
                        app_data.transforms[f"{tf.child_frame_id}->{tf.header.frame_id}"] = (
                            np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]),
                            Rotation.from_quat([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]).as_matrix()
                        )
                update_robot_pose()
        except Exception as e:
            LOG.error(f"Error processing message from {key_expr}: {e}", exc_info=True)

    LOG.info("Zenoh thread started.")
    try:
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", json.dumps([ZENOH_ROUTER_ADDRESS]))
        session = zenoh.open(conf)
        LOG.info("Zenoh session opened successfully.")

        for topic in ROS_MSG_TYPES.keys():
            session.declare_subscriber(topic, on_message)
        LOG.info("Zenoh subscribers declared.")
        
        while True: time.sleep(1)
    except Exception as e:
        LOG.error(f"An error occurred in zenoh_thread: {e}", exc_info=True)


app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.on_event("startup")
async def startup_event():
    global app_data
    app_data = AppData()
    loop = asyncio.get_running_loop()
    threading.Thread(target=zenoh_thread, args=(loop,), daemon=True).start()


@app.get("/")
async def read_root():
    return FileResponse('index.html')

async def broadcast_map():
    with app_data.lock:
        if app_data.latest_map_img is None or app_data.map_info is None: return
        _, buffer = cv2.imencode('.png', app_data.latest_map_img)
        img_bytes = buffer.tobytes()
        map_metadata = { "type": "map", "info": { "resolution": app_data.map_info.resolution, "width": app_data.map_info.width, "height": app_data.map_info.height, "origin": { "position": { "x": app_data.map_info.origin.position.x, "y": app_data.map_info.origin.position.y, }}}}
    
    clients_to_send = [client for client in connected_clients if client.client_state == WebSocketState.CONNECTED]
    if clients_to_send:
        await asyncio.gather(*[client.send_json(map_metadata) for client in clients_to_send], *[client.send_bytes(img_bytes) for client in clients_to_send])

@app.websocket("/ws/visualization")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    LOG.info("Client connected.")
    if app_data and app_data.latest_map_img is not None: await broadcast_map()
    try:
        while True:
            if app_data and app_data.robot_pose_on_map:
                with app_data.lock: pose = app_data.robot_pose_on_map
                if pose: await websocket.send_json({ "type": "pose", "pose": { "x": pose[0], "y": pose[1], "theta": pose[2] } })
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        LOG.info("Client disconnected.")
    finally:
        if websocket in connected_clients: connected_clients.remove(websocket)