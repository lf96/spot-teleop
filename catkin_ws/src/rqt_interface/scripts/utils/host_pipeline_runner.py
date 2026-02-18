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
        print(f"[HOST] Error starting pipeline: {e}", flush=True)

def stop_pipeline_async():
    try:
        pipeline.stop()
    except Exception as e:
        print(f"[HOST] Error stopping pipeline: {e}", flush=True)


while True:
    print("[HOST] Waiting for pipeline controller connection...", flush=True)
    conn, _ = sock.accept()
    print("[HOST] Pipeline controller connected", flush=True)

    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            command = data.decode().strip()
            print(f"[HOST] Received command: {command}", flush=True)

            if command == "stop":
                threading.Thread(
                    target=stop_pipeline_async, 
                    daemon=True
                    ).start()
                conn.sendall(b"Stop command received\n")
                continue
            if command == "open_gripper":
                threading.Thread(
                    target=pipeline.openGripper, 
                    daemon=True
                    ).start()
                conn.sendall(b"Open gripper command received\n")
                continue
            if command == "close_gripper":
                threading.Thread(
                    target=pipeline.closeGripper, 
                    daemon=True
                    ).start()
                conn.sendall(b"Close gripper command received\n")
                continue
            else:
                threading.Thread(
                    target=run_pipeline_async, 
                    args=(command,),
                    daemon=True
                    ).start()
                
                conn.sendall(f"Pipeline '{command}' started".encode())
            
    except Exception as e:
        print(f"[HOST] Connection error: {e}", flush=True)
        conn.sendall(f"Error: {e}".encode())
    finally:
        conn.close()
        print("[HOST] Pipeline controller disconnected", flush=True)