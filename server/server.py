import asyncio
import websockets
import numpy as np
import cv2

async def video_stream_handler(websocket, path):
    while True:
        try:
            frame_data = await websocket.recv()
            # Convert bytes data to a numpy array
            np_arr = np.frombuffer(frame_data, np.uint8)
            # Decode numpy array to an image
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is not None:
                # Display the image using OpenCV
                cv2.imshow("Video Stream", image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Failed to decode image")
        except websockets.exceptions.ConnectionClosed:
            print("Connection closed")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")

    cv2.destroyAllWindows()

start_server = websockets.serve(video_stream_handler, "localhost", 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
