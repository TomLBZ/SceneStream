const videoElement = document.getElementById('video');
const ws = new WebSocket('ws://localhost:8765'); // Connect to the WebSocket server

// Access webcam and stream video
navigator.mediaDevices.getUserMedia({ video: true })
    .then(stream => {
        videoElement.srcObject = stream;

        const mediaRecorder = new MediaRecorder(stream, { mimeType: 'video/webm' });

        mediaRecorder.ondataavailable = event => {
            if (event.data.size > 0 && ws.readyState === WebSocket.OPEN) {
                ws.send(event.data);
            }
        };

        mediaRecorder.start(100); // Send data in chunks every 100 milliseconds

    })
    .catch(error => {
        console.error('Error accessing webcam:', error);
    });

ws.onopen = () => {
    console.log('WebSocket connection established');
};

ws.onclose = () => {
    console.log('WebSocket connection closed');
};

ws.onerror = error => {
    console.error('WebSocket error:', error);
};
