import socket

HOST = '127.0.0.1'  # 또는 cmd_trio_motion_tcp 실행되는 IP
PORT = 5000

def run_client():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            s.connect((HOST, PORT))
            print(f"[CONNECTED] to {HOST}:{PORT}")

            while True:
                try:
                    data = s.recv(1024)
                    if not data:
                        print("[DISCONNECTED] Server closed the connection.")
                        break
                    print(f"[RECEIVED] {data}")
                except socket.timeout:
                    continue
    except Exception as e:
        print(f"[ERROR] {e}")

if __name__ == "__main__":
    run_client()

