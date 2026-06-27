import socket

HOST = "10.51.9.67"  # Replace with your IP
PORT = 3883             # Replace with your port

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))

    while True:
        data = s.recv(1024)
        if not data:
            break

        print("Received:", data.decode("utf-8"))