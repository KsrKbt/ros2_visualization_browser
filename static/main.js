// static/main.js

const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const statusElement = document.getElementById('status');

// マップの表示倍率
const SCALE_FACTOR = 10; // 10倍で表示（この数値は自由に変更してください）

// アプリケーションの状態を管理
const state = {
    map: { image: null, info: null },
    robot: { pose: null }
};

// Web Workerを起動
const worker = new Worker('/static/worker.js');

// Workerからのメッセージを待機
worker.onmessage = (event) => {
    const { type, ...data } = event.data;
    switch (type) {
        case 'status':
            statusElement.textContent = data.message;
            break;
        case 'mapInfo':
            state.map.info = data.info;
            // Canvasの描画領域サイズを、元のマップサイズ x 倍率に設定
            canvas.width = data.info.width * SCALE_FACTOR;
            canvas.height = data.info.height * SCALE_FACTOR;
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
    // getContextはループ内で一度だけ取得するのが効率的
    const localCtx = canvas.getContext('2d');
    localCtx.clearRect(0, 0, width, height);

    if (!state.map.image) {
        localCtx.fillStyle = '#eee';
        localCtx.fillRect(0, 0, width, height);
        return;
    }

    // ★★★ 重要な設定 ★★★
    // JavaScript側でもスムージング無効化を念のため設定
    localCtx.imageSmoothingEnabled = false;

    // マップ画像をCanvas全体に拡大して描画
    localCtx.drawImage(state.map.image, 0, 0, width, height);

    // ロボットを描画
    if (state.robot.pose && state.map.info) {
        const { info } = state.map;
        const { pose } = state.robot;

        // ROS座標からピクセル座標への変換 (倍率を考慮)
        const px = ((pose.x - info.origin.position.x) / info.resolution) * SCALE_FACTOR;
        const py = ((pose.y - info.origin.position.y) / info.resolution) * SCALE_FACTOR;

        localCtx.save();
        localCtx.translate(px, py);
        localCtx.rotate(pose.theta);

        const arrowLength = 12;
        const arrowWidth = 8;
        localCtx.fillStyle = 'rgba(255, 0, 0, 0.9)';
        localCtx.beginPath();
        localCtx.moveTo(arrowLength, 0);
        localCtx.lineTo(-arrowLength / 2, -arrowWidth / 2);
        localCtx.lineTo(-arrowLength / 2, arrowWidth / 2);
        localCtx.closePath();
        localCtx.fill();
        
        localCtx.restore();
    }
}

// 描画ループを開始
draw();