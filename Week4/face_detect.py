import cv2

CONF_THRESHOLD = 0.8

frontal_face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
profile_face_cascade = cv2.CascadeClassifier('haarcascade_profileface.xml')

modelFile = "models/res10_300x300_ssd_iter_140000_fp16.caffemodel"
configFile = "models/deploy.prototxt"
net = cv2.dnn.readNetFromCaffe(configFile, modelFile)

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
    
def detect_largest_face_dnn(image, screen_size):
    faces = detect_faces_dnn(image, screen_size)
    if len(faces) == 0:
        return None
    else:
        return max(faces, key=FaceDetected.calc_area)

def detect_faces(grey_image, screen_size):
    # haar cascades
    frontal_faces = frontal_face_cascade.detectMultiScale(grey_image, scaleFactor=1.3, minNeighbors=5)

    profile_faces_1 = profile_face_cascade.detectMultiScale(grey_image,  scaleFactor=1.3, minNeighbors=5)
    profile_faces_2 = profile_face_cascade.detectMultiScale(cv2.flip(grey_image, 1),  scaleFactor=1.3, minNeighbors=5)

    face_results = [(frontal_faces, False), (profile_faces_1, False), (profile_faces_2, True)]
    
    detected_faces = []
    # for faces in [(frontal_faces, False)]:
    for faces in face_results:
        faces, flip = faces
        for face in faces:
            detected_faces.append(FaceDetected(face, screen_size, flip))

    return detected_faces

def detect_faces_dnn(image, screen_size):
    # dnn

    blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), [104, 117, 123], False, False,)

    net.setInput(blob)
    detections = net.forward()

    detected_faces = []

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > CONF_THRESHOLD:
            x1 = int(detections[0, 0, i, 3] * screen_size[0])
            y1 = int(detections[0, 0, i, 4] * screen_size[1])
            x2 = int(detections[0, 0, i, 5] * screen_size[0])
            y2 = int(detections[0, 0, i, 6] * screen_size[1])
            detected_faces.append(FaceDetected([x1, y1, x2-x1, y2-y1], screen_size))
    
    return detected_faces

# def detectFaceOpenCVDnn(net, frame, conf_threshold=0.7):
    
#     frameHeight = frame.shape[0]
#     frameWidth = frame.shape[1]
#     blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], False, False,)

#     net.setInput(blob)
#     detections = net.forward()
#     bboxes = []
#     for i in range(detections.shape[2]):
#         confidence = detections[0, 0, i, 2]
#         if confidence > conf_threshold:
#             x1 = int(detections[0, 0, i, 3] * frameWidth)
#             y1 = int(detections[0, 0, i, 4] * frameHeight)
#             x2 = int(detections[0, 0, i, 5] * frameWidth)
#             y2 = int(detections[0, 0, i, 6] * frameHeight)
#             bboxes.append([x1, y1, x2, y2])
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), int(round(frameHeight / 150)), 8,)
            
#             top=x1
#             right=y1
#             bottom=x2-x1
#             left=y2-y1

#             #  blurry rectangle to the detected face
#             face = frame[right:right+left, top:top+bottom]
#             face = cv2.GaussianBlur(face,(23, 23), 30)
#             frame[right:right+face.shape[0], top:top+face.shape[1]] = face

#     return frame, bboxes