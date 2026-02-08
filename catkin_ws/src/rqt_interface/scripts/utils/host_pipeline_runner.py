from pipeline_controller import PipelineController
import os
import socket
import threading

SOCKET_PATH = "/home/nexus/ipc/pipeline_controller.sock"

if os.path.exists(SOCKET_PATH):
    os.remove(SOCKET_PATH)

sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.bind(SOCKET_PATH)
os.chmod(SOCKET_PATH, 0o666)
sock.listen(1)

pipeline = PipelineController()

def run_pipeline_async(name):
    try:
        pipeline.start_pipeline(name)
    except Exception as e:
        print(f"[HOST] Error starting pipeline: {e}")

def stop_pipeline_async():
    try:
        pipeline.stop()
    except Exception as e:
        print(f"[HOST] Error stopping pipeline: {e}")

while True:
    print("[HOST] Waiting for pipeline controller connection...")
    conn, _ = sock.accept()
    print("[HOST] Pipeline controller connected")

    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            command = data.decode().strip()
            print(f"[HOST] Received command: {command}")

            if command == "stop":
                threading.Thread(
                    target=stop_pipeline_async, 
                    daemon=True
                    ).start()
                conn.sendall(b"Stop command received\n")
                continue
            else:
                threading.Thread(
                    target=run_pipeline_async, 
                    args=(command,),
                    daemon=True
                    ).start()
                
                conn.sendall(f"Pipeline '{command}' started".encode())
            
    except Exception as e:
        print(f"[HOST] Connection error: {e}")
        conn.sendall(f"Error: {e}".encode())
    finally:
        conn.close()
        print("[HOST] Pipeline controller disconnected")
