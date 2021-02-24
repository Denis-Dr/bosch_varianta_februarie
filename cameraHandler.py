from threading import Thread
import cv2
import numpy as np

class VideoStream:
    """Camera object"""
    def __init__(self, resolution=(640,480),framerate=30,PiOrUSB=1,src=0,driverWindows=None):

        # Create a variable to indicate if it's a USB camera or PiCamera.
        # PiOrUSB = 1 will use PiCamera. PiOrUSB = 2 will use USB camera.
        self.PiOrUSB = PiOrUSB

        if self.PiOrUSB == 1: # PiCamera
            # Import packages from picamera library
            from picamera.array import PiRGBArray
            from picamera import PiCamera

            # Initialize the PiCamera and the camera image stream
            self.camera = PiCamera()
            self.camera.resolution = resolution
            self.camera.framerate = framerate
            self.rawCapture = PiRGBArray(self.camera,size=resolution)
            self.stream = self.camera.capture_continuous(
                self.rawCapture, format = "bgr", use_video_port = True)

            # Initialize variable to store the camera frame
            self.frame = np.zeros(resolution,3)

        if self.PiOrUSB == 2: # USB camera
            # Initialize the USB camera and the camera image stream
            self.driverWindows = driverWindows
            self.stream = cv2.VideoCapture(src, self.driverWindows)
            self.stream.set(3,resolution[0])
            self.stream.set(4,resolution[1])
            self.stream.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
            #ret = self.stream.set(5,framerate) #Doesn't seem to do anything so it's commented out

            # Read first frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    # Create a variable to control when the camera is stopped
        self.stopped = False

    def start(self):
    # Start the thread to read frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):

        if self.PiOrUSB == 1: # PiCamera

            # Keep looping indefinitely until the thread is stopped
            for f in self.stream:
                # Grab the frame from the stream and clear the stream
                # in preparation for the next frame
                self.frame = f.array
                self.rawCapture.truncate(0)

                if self.stopped:
                    # Close camera resources
                    self.stream.close()
                    self.rawCapture.close()
                    self.camera.close()

        if self.PiOrUSB == 2: # USB camera

            # Keep looping indefinitely until the thread is stopped
            while True:
                # If the camera is stopped, stop the thread
                if self.stopped:
                    # Close camera resources
                    self.stream.release()
                    return

                # Otherwise, grab the next frame from the stream
                (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # Return the most recent frame
        return self.frame

    def stop(self):
        # Indicate that the camera and thread should be stopped
        self.stopped = True


class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src=0, driver=None):
        self.stream = cv2.VideoCapture(src, driver)
        self.stream.set(3, 640)
        self.stream.set(4, 480)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
                self.stream.release()

            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True


class VideoShow:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, frame=None, name="Video"):
        self.frame = frame
        self.stopped = False
        self.name = name

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow(self.name, self.frame)
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True
            if self.stopped == True:
                cv2.destroyAllWindows()

    def stop(self):
        self.stopped = True


'''
%%%%% EXEMPLU DE UTILIZARE  -->> A NU SE STERGE COMENTARIUL  !!!  %%%%

def threadBoth(source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Dedicated thread for showing video frames with VideoShow object.
    Main thread serves only to pass frames between VideoGet and
    VideoShow objects/threads.
    """

    video_getter = VideoGet(source).start()
    video_shower = VideoShow(video_getter.frame).start()
    cps = CountsPerSec().start()  # -> o clasa care pune nr de iteratii in colt

    while True:
        if video_getter.stopped or video_shower.stopped:
            video_shower.stop()
            video_getter.stop()
            break

        frame = video_getter.frame
        frame = putIterationsPerSec(frame, cps.countsPerSec())
        video_shower.frame = frame
        cps.increment()
'''