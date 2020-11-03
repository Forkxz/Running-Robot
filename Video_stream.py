import cv2
from threading import Thread
import numpy as np
import time
class LoadStreams:  # multiple IP or RTSP cameras
    def __init__(self, sources=0):
        self.mode = 'images'

        self.imgs = None
        self.sources = sources
        # Start the thread to read frames from the video stream
        cap = cv2.VideoCapture(sources)
        assert cap.isOpened(), 'Failed to open %s' % sources
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS) % 100
        _, self.imgs = cap.read()  # guarantee first frame
        thread = Thread(target=self.update,args=([cap]), daemon=True)
        print(' success (%gx%g at %.2f FPS).' % (w, h, fps))
        thread.start()
        print('')  # newline

    def update(self, cap):
        # Read next stream frame in a daemon thread
        n = 0
        while cap.isOpened():
            n += 1
            cap.grab()
            if n == 4:  # read every 4th frame
                _, self.imgs = cap.retrieve()
                n = 0
            time.sleep(0.01)  # wait time

    def __iter__(self):
        self.count = -1
        return self

    def __next__(self):
        self.count += 1
        img0 = self.imgs.copy()
        if cv2.waitKey(1) == ord('q'):  # q to quit
            cv2.destroyAllWindows()
            raise StopIteration

        return img0

    def __len__(self):
        return 0  # 1E12 frames = 32 streams at 30 FPS for 30 years

if __name__ == '__main__':
    # source = "http://192.168.1.111:8082/?action=stream?dummy=param.mjpg"
    dataset = LoadStreams(2)

    while True:
        cv2.imshow('img0', dataset.imgs)
        if cv2.waitKey(1) == ord('q'):  # q to quit
            raise StopIteration