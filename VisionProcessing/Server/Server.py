import socket
import cv2
import numpy as np

sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.bind(('',5810))
sock.listen(5)

def main():
  client, addr = sock.accept()
	while True:
		img = client.recv(300000)
		string = np.fromstring(img,np.uint8)
		im = cv2.imdecode(string, 1)
		try:
			cv2.imshow('Frame',im)
		except Exception as e:
			continue
			
		if cv2.waitKey(10) & 0xFF == 113:
			break
if __name__ == '__main__':
	main()	
