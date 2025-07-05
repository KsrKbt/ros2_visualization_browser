import zenoh
import time
import json

# メッセージを受信するたびに呼び出される関数 (コールバック関数)
def listener(sample: zenoh.Sample):
    """
    受信したデータのキー(トピック名)とペイロード(値)を画面に表示する
    """
    topic = sample.key_expr
    
    try:
        # --- 変更箇所 ---
        # ZBytes を標準の bytes に変換してからデコードする
        payload_str = bytes(sample.payload).decode("utf-8")
        
        # 見やすいようにJSONとして整形しようと試みる
        payload_formatted = json.dumps(json.loads(payload_str), indent=2)
        print(f">> [Topic: {topic}]\n{payload_formatted}\n")

    except (UnicodeDecodeError, json.JSONDecodeError):
        # デコードやJSON解析に失敗した場合は、そのまま表示
        print(f">> [Topic: {topic}] Received (raw): {sample.payload}\n")


# --- メイン処理 ---
if __name__ == "__main__":
    # Zenohセッションを開始
    conf = zenoh.Config()
    session = zenoh.open(conf)

    print("Subscribing to all topics ('**')...")
    
    # "**" トピックを購読し、メッセージが来たら listener 関数を呼び出す
    sub = session.declare_subscriber("map", listener)

    print("Listening for messages... Press Ctrl+C to quit.")

    try:
        # プログラムが終了しないように無限ループで待機
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # Ctrl+C が押されたら、セッションを閉じて終了
        print("Stopping subscriber.")
        session.close()