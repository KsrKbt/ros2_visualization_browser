# zenohを介したロボットデータ購読
# 購読データを用いて地図画像生成処理とロボット位置の計算処理を行う
# FastAPIを用いて、処理結果をWebSocket経由で接続中の全クライアントに配信。また、Webページの提供

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

# ROS2トピック定義
MAP_TOPIC = "map" # 地図データ
TF_TOPIC = "tf" # 自己位置データ
TF_STATIC_TOPIC = "tf_static" # 自己位置データ

# トピック名とメッセージ型定義の紐付け
ROS_MSG_TYPES = {
    MAP_TOPIC: OccupancyGrid,
    TF_TOPIC: TFMessage,
    TF_STATIC_TOPIC: TFMessage
}

# 接続中のWebSocketクライアントリスト
connected_clients: list[WebSocket] = []
app_data = None

class AppData:
    def __init__(self): # 各データ保持用の変数定義
        self.lock = threading.Lock()
        self.latest_map_img = None # 地図画像
        self.map_info = None # 地図のメタデータ
        self.transforms = {} # 座標変換データ（辞書変数）
        self.robot_pose_on_map = None # ロボットの最終的な姿勢

def make_transformation_matrix(translation, rotation):
    # 位置と回転行列から4x4の同次変換行列を作成
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T

# 地図（map座標系）におけるロボットの姿勢（位置と向き）の算出処理
def update_robot_pose():
    # tfツリー（ロボットの姿勢の座標関係を木構造で表したもの）からロボットのmap座標系での姿勢を計算
    # tfツリーの内base_footprint→odom→mapの部分を変換
    global app_data
    with app_data.lock:
        # 変換データの存在確認
        required_keys = ["base_footprint->odom", "odom->map"]
        if not all(key in app_data.transforms for key in required_keys):
            return
        try:
            # 同次変換行列に変換
            trans1, rot1 = app_data.transforms[required_keys[0]]
            T_footprint_odom = make_transformation_matrix(trans1, rot1)

            trans2, rot2 = app_data.transforms[required_keys[1]]
            T_odom_map = make_transformation_matrix(trans2, rot2)
            
            T_footprint_map = T_odom_map @ T_footprint_odom # 行列の乗算で変換を1つに合成
            x, y = T_footprint_map[0, 3], T_footprint_map[1, 3] # x座標とy座標を抽出
            r = Rotation.from_matrix(T_footprint_map[:3, :3])
            yaw = r.as_euler('zyx', degrees=False)[0] # 向き（ヨー角）を抽出
            app_data.robot_pose_on_map = [x, y, yaw] # 最終的な姿勢
        except Exception as e:
            LOG.error(f"Error from update_robot_pose: {e}", exc_info=True)

# Zenohのセッションと購読を管理するスレッド
def zenoh_thread(loop: asyncio.AbstractEventLoop):
    # Zenohサブスクライバのコールバック関数
    def on_message(msg: zenoh.Sample):
        # トピック名の判別用に文字列に変換
        key_expr = str(msg.key_expr)
        #LOG.info(f"Received message on topic: {key_expr}") # 受信確認用ログ
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
                    
                    # 地図の色はROS2標準のビジュアライザーであるRviz2の仕様を参考にした
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
                # ロボットの位置を計算
                update_robot_pose()
        except Exception as e:
            LOG.error(f"Error from {key_expr}: {e}", exc_info=True)

    LOG.info("Zenoh thread started.")
    session = None
    try:
        conf = zenoh.Config()
        # zenohセッションを開始
        # zenohのScouting機能で同一ネットワーク内で動作しているzenoh-bridge-ros2ddsを自動検出し,通信開始
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

# FastAPIの初期化及び静的ファイル配信
app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.on_event("startup")
async def startup_event():
    global app_data
    app_data = AppData()
    loop = asyncio.get_running_loop()
    # zenoh_threadをバックグラウンドスレッドとして開始
    # （zenohからの受信処理と画像処理を別スレッドで実行）
    threading.Thread(target=zenoh_thread, args=(loop,), daemon=True).start()

# ユーザがトップページにアクセスした際にindex.htmlを返すAPIを定義
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

# ブラウザと通信を行うためのWebSocket接続に関する定義
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