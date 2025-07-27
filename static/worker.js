// クライアントにおけるデータ受信処理（バックグラウンド）
// 受信したデータはメインスレッド（main.js）に送信する

let websocket;
let flag = false; // 地図のメタデータ（解像度やサイズ）を受信した後に、地図画像の受信を待機していることを表すためのフラグ

function connect() {
    const ws_url = `http://${self.location.host}/ws/visualization`;
    // サーバへのWebSocket接続開始
    websocket = new WebSocket(ws_url);
    
    // main.jsへ接続ステータスを送信
    // postMessageでメインスレッド（main.js）に報告
    self.postMessage({ type: 'status', message: 'Connecting...' });
    websocket.onopen = () => { // 接続成功
        self.postMessage({ type: 'status', message: 'Connected' });
    };

    websocket.onmessage = async (event) => {
        if (typeof event.data === 'string') { // 文字列を受信した場合、地図のメタデータかロボットの姿勢データとして判断
            const data = JSON.parse(event.data); // jsonをjava scriptオブジェクトに変換
            if (data.type === 'map') { // 地図のメタデータだった場合
                flag = true; // 次は地図画像の受信を待機
                self.postMessage({ type: 'mapInfo', info: data.info }); // メインスレッドに送信
            } else if (data.type === 'pose') { // 姿勢データだった場合
                self.postMessage({ type: 'pose', pose: data.pose }); // メインスレッドに送信
            }
        } else if (event.data instanceof Blob && flag) { // 地図画像（バイナリデータ）を受信した際に、地図画像の受信を待機していた（flag=true）場合
            // 画像バイナリデータを画像データにデコード
            const imageBitmap = await createImageBitmap(event.data);
            flag = false; // 画像の処理を行ったので、次はメタデータの受信を待機
            self.postMessage({ type: 'mapImage', image: imageBitmap }, [imageBitmap]); // デコードした画像をメインスレッドに送信（所有権の移動）
        }
    };

    websocket.onclose = () => { // 接続切断
        self.postMessage({ type: 'status', message: 'Disconnected. Retrying...' });
        setTimeout(connect, 3000);
    };

    websocket.onerror = (error) => { // 接続エラー
        self.postMessage({ type: 'status', message: 'Connection Error' });
        websocket.close();
    };
}

connect();