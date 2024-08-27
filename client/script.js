const videoElement = document.getElementById('webcam');
const canvas = document.createElement('canvas');
const context = canvas.getContext('2d');
let socket = null;

// Access the webcam stream
navigator.mediaDevices.getUserMedia({ video: true })
    .then((stream) => {
        videoElement.srcObject = stream;
        startStreaming(stream);
    })
    .catch((err) => console.error('Error accessing webcam:', err));

function startStreaming(stream) {
    socket = new WebSocket('ws://localhost:8765');
    
    socket.onopen = () => {
        const videoTrack = stream.getVideoTracks()[0];
        const imageCapture = new ImageCapture(videoTrack);

        setInterval(async () => {
            try {
                const bitmap = await imageCapture.grabFrame();
                canvas.width = bitmap.width;
                canvas.height = bitmap.height;
                context.drawImage(bitmap, 0, 0);

                canvas.toBlob((blob) => {
                    if (socket.readyState === WebSocket.OPEN) {
                        socket.send(blob);
                    }
                    else {
                        alert('HERE.');
                    }
                }, 'image/jpeg');
            } catch (error) {
                console.error('Error capturing frame:', error);
            }
        }, 100); // Sending frames every 100 ms
    };

    socket.onclose = (event) => {
        console.log('WebSocket closed:', event);
        alert('Connection closed. Please refresh the page to restart the stream.');
    };

    socket.onerror = (error) => {
        console.error('WebSocket error:', error);
        alert('WebSocket encountered an error. Please check the console for more details.');
    };
}
