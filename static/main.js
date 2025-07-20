const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const statusElement = document.getElementById('status');

const map_scale = 7; // 地図の表示倍率

// データ管理のためのオブジェクト定義
const state = {
    map: { image: null, info: null },
    robot: { pose: null }
};

// サーバとの通信を担うworker.jsをバックグラウンドで起動
const worker = new Worker('/static/worker.js');

// workerからのメッセージ待機
worker.onmessage = (event) => {
    const { type, ...data } = event.data; //受信したデータからtypeプロパティを分離
    switch (type) {
        case 'status': // サーバとの接続状態表示
            statusElement.textContent = data.message;
            break;
        case 'mapInfo': // 受信した地図を設定した表示倍率と掛け合わせる
            state.map.info = data.info;
            canvas.width = data.info.width * map_scale;
            canvas.height = data.info.height * map_scale;
            break;
        case 'mapImage':
            state.map.image = data.image;
            break;
        case 'pose':
            state.robot.pose = data.pose;
            break;
    }
};

// 描画ループ
function draw() {
    requestAnimationFrame(draw);

    const { width, height } = canvas;
    const localCtx = canvas.getContext('2d'); // getContextはループ内で一度だけ取得するのが効率的
    localCtx.clearRect(0, 0, width, height); // 前回の描画を削除
    
    // 地図が届いていない場合
    if (!state.map.image) {
        localCtx.fillStyle = '#eee';
        localCtx.fillRect(0, 0, width, height);
        return;
    }

    // 画像のぼかしを無効化
    localCtx.imageSmoothingEnabled = false;

    // 地図画像をCanvas全体に拡大して描画
    localCtx.drawImage(state.map.image, 0, 0, width, height);

    // ロボットの描画
    if (state.robot.pose && state.map.info) {
        const { info } = state.map;
        const { pose } = state.robot;

        // Ros座標からピクセル座標への変換 (表示倍率を考慮)
        const px = ((pose.x - info.origin.position.x) / info.resolution) * map_scale;
        const py = ((pose.y - info.origin.position.y) / info.resolution) * map_scale;

        localCtx.save(); // 座標系の初期状態（左上が原点）を保存
        // ロボットの位置情報をもとに座標系を移動
        localCtx.translate(px, py);
        localCtx.rotate(pose.theta);

        // ロボットを表す矢印の描画
        const arrowLength = 12;
        const arrowWidth = 8;
        localCtx.fillStyle = 'rgba(255, 0, 0, 0.9)';
        localCtx.beginPath();
        localCtx.moveTo(arrowLength, 0);
        localCtx.lineTo(-arrowLength / 2, -arrowWidth / 2);
        localCtx.lineTo(-arrowLength / 2, arrowWidth / 2);
        localCtx.closePath();
        localCtx.fill();
        
        localCtx.restore(); // 座標系の初期状態にリセット
    }
}

draw();