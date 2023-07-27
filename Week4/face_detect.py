import cv2

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

class FaceDetected:
    def __init__(self, face, screen_size):
        self.x = face[0]
        self.y = face[1]
        self.w = face[2]
        self.h = face[3]

        self.center_x = self.x + self.w/2
        self.center_y = self.y + self.h/2

        self.prop_x = self.x / screen_size[0]
        self.prop_y = self.y / screen_size[1]
        self.prop_w = self.w / screen_size[0]
        self.prop_h = self.h / screen_size[1]

        self.prop_center_x = self.center_x / screen_size[0]
        self.prop_center_y = self.center_y / screen_size[1]

    def calc_area(self):
        return self.w * self.h

    def __str__(self):
        return f'Face at {self.center}'
    
    def draw(self, image, color = (255, 0, 0)):
        # rect + center dot
        cv2.rectangle(image, (self.x, self.y), (self.x+self.w, self.y+self.h), color, 2)
        cv2.circle(image, (int(self.center_x), int(self.center_y)), 5, (255, 255, 255), -1)

def detect_largest_face(grey_image, screen_size):
    faces = detect_faces(grey_image, screen_size)
    if len(faces) == 0:
        return None
    else:
        return max(faces, key=FaceDetected.calc_area)

def detect_faces(grey_image, screen_size):
    faces = face_cascade.detectMultiScale(grey_image, 1.3, 5)
    return [FaceDetected(face, screen_size) for face in faces]