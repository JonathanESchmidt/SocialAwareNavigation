import tensorflow as tf
from tf2_yolov4.anchors import YOLOV4_ANCHORS
from tf2_yolov4.model import YOLOv4
import matplotlib.pyplot as plt
import time
 

class tfYOLOv4():
    def __init__(self, width = 1024, height = 768):

        self.WIDTH, self.HEIGHT = (width, height)

        self.CLASSES = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
            'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli',
            'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
            'bed', 'dining table', 'toilet', 'tv', 'laptop',  'mouse', 'remote', 'keyboard',
            'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
            'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        self.model = YOLOv4(
            input_shape=(self.HEIGHT, self.WIDTH, 3),
            anchors=YOLOV4_ANCHORS,
            num_classes=80,
            training=False,
            yolo_max_boxes=50,
            yolo_iou_threshold=0.5,
            yolo_score_threshold=0.5,
        )

        self.model.load_weights('./yolov4.h5')

    def predict(self, img_path):
        self.image = tf.io.read_file(img_path)
        self.image = tf.io.decode_image(self.image)
        self.image = tf.image.resize(self.image, (self.HEIGHT, self.WIDTH))
        self.images = tf.expand_dims(self.image, axis=0) / 255

        
        self.boxes, self.scores, self.classes, self.detections = self.model.predict(self.images)
 
        self.boxes = self.boxes[0] * [self.WIDTH, self.HEIGHT, self.WIDTH, self.HEIGHT]
        self.scores = self.scores[0]
        self.classes = self.classes[0].astype(int)
        self.detections = self.detections[0]
 
    def show(self, objectType = 0):
 
        plt.imshow(self.images[0])
        ax = plt.gca()
        
        for (xmin, ymin, xmax, ymax), score, class_idx in zip(self.boxes, self.scores, self.classes):
            if score > 0 and class_idx == objectType:
                rect = plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                                    fill=False, color='green')
                ax.add_patch(rect)
        
                text = self.CLASSES[class_idx] + ': {0:.2f}'.format(score)
                ax.text(xmin, ymin, text, fontsize=9, bbox=dict(facecolor='yellow', alpha=0.6))
                #print(score)
        
        plt.title('Objects detected: {}'.format(self.detections))
        plt.axis('off')
        plt.show()

if __name__ == "__main__":
    start = time.time()
    testModel = tfYOLOv4()
    modelDone = time.time()
    delta = modelDone - start
    print(f"Intilisation of model took: {delta}")    
    
    testModel.predict('Testimage.jpeg')
    predictDone = time.time()
    delta = predictDone - modelDone
    print(f"Prediciton with model took: {delta}") 
    

    testModel.show()
    start = time.time()

    testModel.predict('Test2.jpeg')
    predict2Done = time.time()
    delta = predict2Done - start
    print(f"Prediciton with model took: {delta}")

    testModel.show()