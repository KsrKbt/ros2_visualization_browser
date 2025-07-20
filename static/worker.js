let websocket;
let flag = false; // 地図のメタデータを受信した「後」に地図画像を受信するためのフラグ

function connect() {
    const ws_url = `http://${self.location.host}/ws/visualization`;
    // サーバへの接続開始
    websocket = new WebSocket(ws_url);
    
    // main.jsへ接続ステータスを送信
    self.postMessage({ type: 'status', message: 'Connecting...' });
    websocket.onopen = () => { // 接続成功
        self.postMessage({ type: 'status', message: 'Connected' });
    };

    websocket.onmessage = async (event) => {
        if (typeof event.data === 'string') {
            const data = JSON.parse(event.data); // jsonをjava scriptオブジェクトに変換
            if (data.type === 'map') { // 地図のメタデータだった場合
                flag = true; // 次は地図画像の受信を待機
                self.postMessage({ type: 'mapInfo', info: data.info }); // メインスレッドに送信
            } else if (data.type === 'pose') { // 姿勢データだった場合
                self.postMessage({ type: 'pose', pose: data.pose }); // メインスレッドに送信
            }
        } else if (event.data instanceof Blob && flag) { // 地図画像を受信したとき、地図画像の受信を待機していた場合
            // 画像バイナリデータを画像データにデコード
            const imageBitmap = await createImageBitmap(event.data);
            flag = false; // 画像の処理を行ったので、次はメタデータの受信を待機
            self.postMessage({ type: 'mapImage', image: imageBitmap }, [imageBitmap]); // デコードした画像をメインスレッドに送信（所有権の移動）
        }
    };

    websocket.onclose = () => {
        self.postMessage({ type: 'status', message: 'Disconnected. Retrying...' });
        setTimeout(connect, 3000);
    };

    websocket.onerror = (error) => {
        self.postMessage({ type: 'status', message: 'Connection Error' });
        websocket.close();
    };
}

connect();