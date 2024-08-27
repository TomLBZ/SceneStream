import asyncio
import websockets
import os

async def video_stream_handler(websocket, path):
    count = 0
    os.makedirs("received_frames", exist_ok=True)
    
    try:
        while True:
            frame_data = await websocket.recv()
            with open(f"received_frames/frame_{count}.jpg", "wb") as f:
                f.write(frame_data)
            count += 1
            print(f"Received frame {count}")
    except websockets.exceptions.ConnectionClosedOK:
        print("Connection closed gracefully.")
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Connection closed with error: {e}")
    except Exception as e:
        readable_exception = e.__class__.__name__ + ": " + str(e)
        print(f"Unexpected error: {readable_exception}")

start_server = websockets.serve(video_stream_handler, "localhost", 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
