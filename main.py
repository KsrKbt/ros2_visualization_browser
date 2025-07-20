import asyncio
import threading
import json
import logging
import cv2
import numpy as np
import time
import zenoh

from fastapi import FastAPI
from fastapi.websockets import WebSocket, WebSocketDisconnect
from starlette.websockets import WebSocketState
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from scipy.spatial.transform import Rotation # 自己位置データの座標変換に使用
from rclpy.serialization import deserialize_message # ROSメッセージをバイナリからpythonオブジェクトに変換
from nav_msgs.msg import OccupancyGrid # 地図データのメッセージ型定義
from tf2_msgs.msg import TFMessage # 座標変換データの型定義

logging.basicConfig(level=logging.INFO)
LOG = logging.getLogger(__name__)
ZENOH_ROUTER_ADDRESS = "tcp/localhost:7447" # 接続先zenohルーターのアドレス指定

# ROSトピック定義（名前空間としてrobot1を指定）
# ロボットが増えた際にzenohネットワーク内でデータを判別可能にする
# ただし今回の実行環境ではロボットはネットワーク内に1台のみ存在
MAP_TOPIC = "robot1/map"
TF_TOPIC = "robot1/tf"
TF_STATIC_TOPIC = "robot1/tf_static"

# トピック名とメッセージ型定義の紐付け
ROS_MSG_TYPES = {
    MAP_TOPIC: OccupancyGrid,
    TF_TOPIC: TFMessage,
    TF_STATIC_TOPIC: TFMessage
}

# 接続中のwebsocketクライアントリスト
connected_clients: list[WebSocket] = []
app_data = None

class AppData:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_map_img = None # 地図画像の保持用
        self.map_info = None # 地図のメタデータ保持用
        self.transforms = {} # 座標変換データの保持用
        self.robot_pose_on_map = None # ロボットの最終的な姿勢の保持用

def make_transformation_matrix(translation, rotation):
    # 位置と回転行列から4x4の同次変換行列を作成
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T

def update_robot_pose():
    # tfツリーからロボットのmap座標系での姿勢を計算
    # tfツリー：base_footprint→odom→odom→map
    global app_data
    with app_data.lock:
        # 変換データの存在確認
        required_keys = ["base_footprint->odom", "odom->map"]
        if not all(key in app_data.transforms for key in required_keys):
            return
        try:
            # 同時変換行列に変換
            trans1, rot1 = app_data.transforms[required_keys[0]]
            T_footprint_odom = make_transformation_matrix(trans1, rot1)

            trans2, rot2 = app_data.transforms[required_keys[1]]
            T_odom_map = make_transformation_matrix(trans2, rot2)
            
            T_footprint_map = T_odom_map @ T_footprint_odom
            x, y = T_footprint_map[0, 3], T_footprint_map[1, 3]
            r = Rotation.from_matrix(T_footprint_map[:3, :3])
            yaw = r.as_euler('zyx', degrees=False)[0]
            app_data.robot_pose_on_map = [x, y, yaw] # 最終的な位置と向き
        except Exception as e:
            LOG.error(f"Error from update_robot_pose: {e}", exc_info=True)

def zenoh_thread(loop: asyncio.AbstractEventLoop):
    # Zenohのセッションと購読を管理するスレッド
    def on_message(msg: zenoh.Sample):
        # Zenohサブスクライバのコールバック関数
        key_expr = str(msg.key_expr)
        # LOG.info(f"Received message on topic: {key_expr}") # 受信確認用ログ
        try:
            msg_type = ROS_MSG_TYPES.get(key_expr)
            if not msg_type: 
                return
            # 受信したバイナリデータをpythonオブジェクトに変換
            data = deserialize_message(bytes(msg.payload), msg_type)

            # mapトピック（地図データ）の処理
            if key_expr == MAP_TOPIC:
                with app_data.lock:
                    app_data.map_info = data.info # 地図のメタデータをAppDataクラスの変数に格納
                    # OccupancyGridオブジェクトからNumPy配列に変換
                    grid_data = np.array(data.data, dtype=np.int8).reshape((data.info.height, data.info.width))
                    
                    # 地図の色はROS2に付属する3Dビジュアライザーであるrviz2の仕様を参考にした
                    # 未探索(-1)領域の色である灰色で画像全体を初期化
                    map_img = np.full((data.info.height, data.info.width, 3), 128, dtype=np.uint8)

                    # 通行可能(0)領域を白に設定
                    map_img[grid_data == 0] = [255, 255, 255]

                    # 障害物(100)領域を黒に設定
                    map_img[grid_data == 100] = [0, 0, 0]

                    # 障害物かもしれない(1-99)領域を特定
                    prob_obstacle = (grid_data > 0) & (grid_data < 100)
                    if np.any(prob_obstacle):
                        prob_values = grid_data[prob_obstacle]
                        # 確率が高いほど黒(0)に近くなるように色を計算
                        gray_values = (255 * (1 - prob_values / 100.0)).astype(np.uint8)
                        # 計算したグレースケール値を画像の該当箇所に適用
                        map_img[prob_obstacle] = np.stack([gray_values]*3, axis=-1)

                    #flipped_map_img = cv2.flip(map_img, 1) # 試験的に画像を左右反転
                    # 最終的な地図画像
                    app_data.latest_map_img = map_img
                
                # メインスレッドのイベントループでコルーチンを実行
                # 地図の更新をFastAPIスレッドに通知し、broadcast_map関数を呼び出し
                asyncio.run_coroutine_threadsafe(broadcast_map(), loop)

            # tfデータ処理
            elif key_expr in [TF_TOPIC, TF_STATIC_TOPIC]:
                with app_data.lock:
                    # 受信したtfデータを辞書に保存
                    for tf in data.transforms:
                        app_data.transforms[f"{tf.child_frame_id}->{tf.header.frame_id}"] = (
                            np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]),
                            Rotation.from_quat([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]).as_matrix()
                        )
                # ロボットの姿勢を計算
                update_robot_pose()
        except Exception as e:
            LOG.error(f"Error from {key_expr}: {e}", exc_info=True)

    LOG.info("Zenoh thread started.")
    session = None
    try:
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", json.dumps([ZENOH_ROUTER_ADDRESS]))
        # zenohルーターとセッションを確立
        session = zenoh.open(conf)
        LOG.info("Zenoh session opened successfully.")
        # 各トピックに対してon_messageコールバックを登録
        for topic in ROS_MSG_TYPES.keys():
            session.declare_subscriber(topic, on_message)

        while True: 
            time.sleep(1) # 待機
    except Exception as e:
        LOG.error(f"Error from zenoh_thread: {e}", exc_info=True)
    finally:
        if session:
            session.close()
            LOG.info("Zenoh session closed.")

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.on_event("startup")
async def startup_event():
    global app_data
    app_data = AppData()
    loop = asyncio.get_running_loop()
    # zenoh_threadをバックグラウンドスレッドとして開始
    threading.Thread(target=zenoh_thread, args=(loop,), daemon=True).start()


@app.get("/")
async def read_root():
    return FileResponse('index.html')

# 接続中のクライアントに最新の地図を送信
async def broadcast_map():
    with app_data.lock:
        # 地図データとメタデータの存在確認
        if app_data.latest_map_img is None or app_data.map_info is None: 
            return
        # NumPy配列の地図画像をpng形式のデータにエンコード
        _, buffer = cv2.imencode('.png', app_data.latest_map_img)
        img_bytes = buffer.tobytes()
        # 地図のメタデータをjson形式で準備
        map_metadata = { 
            "type": "map", 
            "info": { 
                "resolution": app_data.map_info.resolution, 
                "width": app_data.map_info.width, 
                "height": app_data.map_info.height, 
                "origin": { 
                    "position": { 
                        "x": app_data.map_info.origin.position.x, 
                        "y": app_data.map_info.origin.position.y, 
                    }
                }
            }
        }

    # 送信対象のクライアントの抽出
    clients_to_send = [client for client in connected_clients if client.client_state == WebSocketState.CONNECTED]
    if not clients_to_send:
        return
    # 各クライアントへの送信を一つのコルーチンとして定義
    async def send_to_client(client, metadata, image):
        try:
            await client.send_json(metadata)
            await client.send_bytes(image)
            return client, True  # 成功したらTrue
        except Exception:
            return client, False # 失敗したらFalse

    # 全クライアントへの送信タスクを作成
    tasks = [send_to_client(client, map_metadata, img_bytes) for client in clients_to_send]
    
    # タスクを並行実行
    results = await asyncio.gather(*tasks)

    # 失敗したクライアントを特定し、リストから除去
    for client, success in results:
        if not success:
            if client in connected_clients:
                connected_clients.remove(client)

@app.websocket("/ws/visualization")
async def websocket_endpoint(websocket: WebSocket):
    # 新規クライアントからの接続受け入れ
    await websocket.accept()
    connected_clients.append(websocket)
    LOG.info("Client connected.")
    # 地図がすでに存在していればbroadcast_map関数を呼び出す
    if app_data and app_data.latest_map_img is not None: await broadcast_map()
    try:
        while True:
            if app_data and app_data.robot_pose_on_map: # 計算済みの姿勢があれば
                with app_data.lock: pose = app_data.robot_pose_on_map
                # json形式でクライアントに送信
                if pose: await websocket.send_json({ "type": "pose", "pose": { "x": pose[0], "y": pose[1], "theta": pose[2] } })
            await asyncio.sleep(0.1) # 過度なデータ送信を抑制
    except WebSocketDisconnect:
        LOG.info("Client disconnected.")
    finally:
        if websocket in connected_clients: 
            connected_clients.remove(websocket) # 切断されたクライアントをリストから削除