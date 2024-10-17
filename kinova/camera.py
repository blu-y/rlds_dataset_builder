import threading
from threading import Lock
import cv2


class Camera2:
    last_frame = None
    last_ready = None
    lock = Lock()
    capture=None
    def __init__(self, rtsp_link=0, w=320, h=240):
        self.w = w
        self.h = h
        self.capture = cv2.VideoCapture(rtsp_link)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        thread = threading.Thread(target=self.rtsp_cam_buffer, args=(), name="rtsp_read_thread")
        thread.daemon = True
        thread.start()
    def rtsp_cam_buffer(self):
        while True:
            with self.lock:
                self.last_ready = self.capture.grab()
    def getFrame(self):
        if (self.last_ready is not None):
            self.last_ready,self.last_frame=self.capture.retrieve()
            return self.last_frame.copy()
        else:
            return -1

class Camera1:
    last_frame = None
    last_ready = None
    lock = Lock()
    capture=None
    def __init__(self, rtsp_link=6, w=320, h=240):
        self.w = w
        self.h = h
        self.capture = cv2.VideoCapture(rtsp_link)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        thread = threading.Thread(target=self.rtsp_cam_buffer, args=(), name="rtsp_read_thread")
        thread.daemon = True
        thread.start()
    def rtsp_cam_buffer(self):
        while True:
            with self.lock:
                self.last_ready = self.capture.grab()
    def getFrame(self):
        if (self.last_ready is not None):
            self.last_ready,self.last_frame=self.capture.retrieve()
            return self.last_frame.copy()
        else:
            return None
        
if __name__ == "__main__":
    # check working camera index
    working_camera = []
    for i in range(10):  # Test for the first 10 possible camera indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera found at index {i}")
            cap.release()
            working_camera.append(i)
        else:
            print(f"No camera found at index {i}")
    print("Working camera index:", working_camera)
    # cam = []
    # for i in working_camera:
    #     cam.append(Camera(int(i)))
    # cam = input
    # while True:
    #     for i, c in enumerate(cam):
    #         frame = c.getFrame()
    #         if frame is not None:
    #             window_name = f"camera {working_camera[i]}"
    #             cv2.imshow(window_name, frame)
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    # cv2.destroyAllWindows()
    
    # run 2 cameras with threading

    cam1 = Camera1()
    cam2 = Camera2()
    while True:
        frame1 = cam1.getFrame()
        frame2 = cam2.getFrame()
        if frame1 is not None:
            cv2.imshow("camera", frame1)
        if frame2 is not None:
            cv2.imshow("camera2", frame2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break