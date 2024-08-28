const videoElement = document.getElementById('webcam');
const canvas = document.getElementById('canvas');
const context = canvas.getContext('2d');
const startBtn = document.getElementById('start');
const stopBtn = document.getElementById('stop');
const textboxes = document.getElementsByClassName('textbox');

let isEditing = true;
let address = 'ws://localhost:8765';
let width = 640;
let height = 480;
let fps = 30;
let socket;
let interval;

function updateElementsEnabledState() {
    for (let textbox of textboxes) {
        textbox.disabled = !isEditing; // Disable textboxes when not editing
    }
    startBtn.disabled = !isEditing; // Disable start button when not editing
    stopBtn.disabled = isEditing; // Disable stop button when editing
    console.log('isEditing:', isEditing);
}

function applySettings() {
    address = textboxes[0].value;
    width = parseInt(textboxes[1].value);
    height = parseInt(textboxes[2].value);
    fps = parseInt(textboxes[3].value);
    isEditing = false;
}

async function startStreaming() {
    let stream;
    try {
        stream = await navigator.mediaDevices.getUserMedia({ video: true });
        videoElement.srcObject = stream;
    } catch (error) {
        console.error('Error accessing webcam:', error);
        return;   
    }
    try {
        socket = new WebSocket(address);
    } catch (error) {
        console.error('Error creating WebSocket:', error);
        return;
    }

    socket.onopen = () => {
        const videoTrack = stream.getVideoTracks()[0];
        const imageCapture = new ImageCapture(videoTrack);

        interval = setInterval(async () => {
            try {
                const bitmap = await imageCapture.grabFrame();
                canvas.width = width;
                canvas.height = height;
                context.drawImage(bitmap, 0, 0, width, height);

                canvas.toBlob((blob) => {
                    if (socket.readyState === WebSocket.OPEN) {
                        socket.send(blob);
                    }
                }, 'image/jpeg');
            } catch (error) {
                console.error('Error capturing frame:', error);
            }
        }, 1000 / fps);
    };

    socket.onclose = (event) => {
        console.log('WebSocket closed:', event);
        alert('Connection closed. Please restart streaming.');
    };

    socket.onerror = (error) => {
        console.error('WebSocket error:', error);
        alert('WebSocket encountered an error. Please check the console for more details.');
    };
}

function stopStreaming() {
    if (interval) {
        clearInterval(interval);
    }
    if (socket) {
        socket.close();
    }
    if (videoElement.srcObject) {
        videoElement.srcObject.getTracks().forEach((track) => {
            track.stop();
        });
        videoElement.srcObject = null;
    }
}

function init() {
    isEditing = true;
    updateElementsEnabledState();
}

startBtn.addEventListener('click', () => {
    applySettings();
    updateElementsEnabledState();
    startStreaming();
});

stopBtn.addEventListener('click', () => {
    isEditing = true;
    updateElementsEnabledState();
    stopStreaming();
});

init();