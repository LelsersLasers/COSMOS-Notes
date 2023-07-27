import cv2
import face_detect
import time

SCREEN_SIZE = (640, 480)


#------------------------------------------------------------------------------#
class ModeFSM:
    TRACKING = 0
    WAITING = 1
    SCANNING = 2

    MAX_WAIT_TIME = 10.0

    def __init__(self):
        self.current_state = ModeFSM.SCANNING
        self.time_in_state = 0.0

    def update(self, face, delta):
        self.time_in_state += delta
        if face is not None:
            self.current_state = ModeFSM.TRACKING
        elif self.current_state == ModeFSM.TRACKING:
            self.current_state = ModeFSM.WAITING
            self.time_in_state = 0.0
        elif self.current_state == ModeFSM.WAITING:
            if self.time_in_state >= ModeFSM.MAX_WAIT_TIME:
                self.current_state = ModeFSM.SCANNING
                self.time_in_state = 0.0

    def should_track(self):
        return self.current_state in [ModeFSM.TRACKING, ModeFSM.WAITING]
#------------------------------------------------------------------------------#

cap = cv2.VideoCapture(0)

time.sleep(1.0)

t0 = time.time()
t1 = time.time()
delta = 1 / 100

try:
    while True:   

        #----------------------------------------------------------------------#
        t1 = time.time()
        delta = t1 - t0
        t0 = t1
        #----------------------------------------------------------------------#

        ret, image = cap.read() 
    
        grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        face = face_detect.detect_largest_face(grey_image, SCREEN_SIZE)
        if face is not None:
            face.draw(image)
    
        cv2.imshow('image', image)
    
        # Wait for Esc key to stop
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
finally:
    cap.release()

    print("Done.\n")