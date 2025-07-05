// worker.js - バックグラウンドで通信を処理

let websocket;
let waitingForMapImage = false;

function connect() {
    const ws_protocol = self.location.protocol === 'https:' ? 'wss' : 'ws';
    const ws_url = `${ws_protocol}://${self.location.host}/ws/visualization`;
    
    websocket = new WebSocket(ws_url);
    
    self.postMessage({ type: 'status', message: 'Connecting...' });

    websocket.onopen = () => {
        self.postMessage({ type: 'status', message: 'Connected' });
    };

    websocket.onmessage = async (event) => {
        if (typeof event.data === 'string') {
            const data = JSON.parse(event.data);
            if (data.type === 'map') {
                waitingForMapImage = true;
                self.postMessage({ type: 'mapInfo', info: data.info });
            } else if (data.type === 'pose') {
                self.postMessage({ type: 'pose', pose: data.pose });
            }
        } else if (event.data instanceof Blob && waitingForMapImage) {
            const imageBitmap = await createImageBitmap(event.data);
            waitingForMapImage = false;
            self.postMessage({ type: 'mapImage', image: imageBitmap }, [imageBitmap]);
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