import socket
import time

def send_fsm_command(command_char, target_number):
    HOST = '127.0.0.1'   # FSM 서버가 실행 중인 IP (로컬)
    PORT = 6000          # FSM 서버 포트 (MNG_TRIO_PORT)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"[CONNECTING] to {HOST}:{PORT}")
        s.connect((HOST, PORT))

        # 명령 구성
        target_str = f"{target_number:02d}"
        packet = bytearray()
        packet.extend(b'7766')          # Header
        packet.append(ord('1'))         # Command Mode
        packet.append(ord(command_char)) # Command: '1' for move, '0' for stop
        packet.append(ord(target_str[0]))
        packet.append(ord(target_str[1]))
        packet.extend(b'55')            # Footer
        packet.append(ord('\n'))        # End

        print(f"[SENDING] {packet}")
        s.sendall(packet)

        # 응답 수신
        response = s.recv(32)
        print(f"[RESPONSE] {response}")

if __name__ == "__main__":
    print("=== FSM Trio Mock Sender ===")
    send_fsm_command('1', 12)  # 예: 이동 명령, target=12
    time.sleep(1)
    send_fsm_command('0', 0)   # 예: 정지 명령

