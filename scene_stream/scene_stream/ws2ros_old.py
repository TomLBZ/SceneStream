import rclpy
import asyncio
import websockets
import numpy as np
import cv2

from rclpy.node import Node

from sensor_msgs.msg import Image

class Ws2Ros(Node):
    def __init__(self):
        super().__init__('ws2ros')
        self.publisher = self.create_publisher(Image, 'image', 10)

    async def receive(self, websocket, path):
        print("Callback started")
        while True:
            print("Waiting for frame")
            frame_data = await websocket.recv()
            np_arr = np.frombuffer(frame_data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is not None:
                msg = Image()
                msg.data = image.tobytes()
                msg.width = image.shape[1]
                msg.height = image.shape[0]
                msg.encoding = 'bgr8'
                self.publisher.publish(msg)
                print("Image published")
            else:
                print("Failed to decode image")

# async def video_stream_handler(websocket, path):
#     while True:
#         try:
#             frame_data = await websocket.recv()
#             # Convert bytes data to a numpy array
#             np_arr = np.frombuffer(frame_data, np.uint8)
#             # Decode numpy array to an image
#             image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#             if image is not None:
#                 # Display the image using OpenCV
#                 cv2.imshow("Video Stream", image)
#                 if cv2.waitKey(1) & 0xFF == ord('q'):
#                     break
#             else:
#                 print("Failed to decode image")
#         except websockets.exceptions.ConnectionClosed:
#             print("Connection closed")
#             break
#         except Exception as e:
#             print(f"Unexpected error: {e}")

#     cv2.destroyAllWindows()

def stop_all_tasks(loop) -> None:
    """Stop all tasks in the given `loop`"""
    tasks = asyncio.all_tasks(loop)
    for task in tasks:
        if task.done():
            continue
        if task.cancelling():
            while task.cancelling() > 1:
                task.uncancel()
            continue
        if task.cancelled():
            continue
        task.cancel()
    loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))

async def manual_spin(node, sleep_time=0.0, loop=None):
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            if sleep_time >= 0:
                await asyncio.sleep(sleep_time)
    except KeyboardInterrupt:
        if loop is not None:
            stop_all_tasks(loop)
    except Exception as e:
        print(f"Unexpected error: {e}")

def main():
    try:
        rclpy.init()
        ws2ros = Ws2Ros()
        server = websockets.serve(ws2ros.receive, "localhost", 8765)
        loop = asyncio.get_event_loop()
        loop.run_until_complete(server)
        print("Server started")
        loop.create_task(manual_spin(ws2ros, loop=loop))
        print("Manual spin started")
        loop.run_forever()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        exit(0)
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        if loop.is_running():
            loop.stop()
            loop.close()

if __name__ == '__main__':
    main()
