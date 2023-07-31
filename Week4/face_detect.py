import cv2

frontal_face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
profile_face_cascade = cv2.CascadeClassifier('haarcascade_profileface.xml')

class FaceDetected:
    def __init__(self, face, screen_size, flip = False):
        self.x = face[0] if not flip else screen_size[0] - face[0] - face[2]
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
    frontal_faces = frontal_face_cascade.detectMultiScale(grey_image, scaleFactor=1.2, minNeighbors=6, minSize=(30, 30))

    profile_faces_1 = profile_face_cascade.detectMultiScale(grey_image,  scaleFactor=1.2, minNeighbors=6, minSize=(30, 30))
    profile_faces_2 = profile_face_cascade.detectMultiScale(cv2.flip(grey_image, 1),  scaleFactor=1.2, minNeighbors=6, minSize=(30, 30))

    face_results = [(frontal_faces, False), (profile_faces_1, False), (profile_faces_2, True)]
    
    detected_faces = []
    # for faces in [(frontal_faces, False)]:
    for faces in face_results:
        faces, flip = faces
        for face in faces:
            detected_faces.append(FaceDetected(face, screen_size, flip))

    return detected_faces