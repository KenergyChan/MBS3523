import cv2 # torch<=1.11.0
import numpy as np
import pyrealsense2
from realsense_depth import *
import time
import serial
"Servo parts-----------------------------------------------------------------"
ser = serial.Serial('COM3',baudrate=115200,timeout=1)
time.sleep(0.5)
poslr = 90
posud = 40
error = 0
pre_lr=90
pre_ud=40

"-----------------------------------------------------------------"

"Real sense parts-----------------------------------------------------------------"
point = (400, 300)

prev_depth=0
prev_x = 0
prev_y = 0

global boxes
global scores
global class_ids

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

# Initialize Camera Intel Realsense
dc = DepthCamera()

# Create mouse event
cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

"-----------------------------------------------------------------"

"Yolov5 model parts-----------------------------------------------------------------"

path = "best.onnx"


# Initialize model
net = cv2.dnn.readNet(path)

input_shape = (640, 640) # 640 for YOLOv5s

input_height = int(input_shape[0])
input_width = int(input_shape[1])
img_height = 0
img_width = 0

output_names = net.getUnconnectedOutLayersNames()
has_postprocess = 'score' in output_names

conf_threshold = 0.6 #0.7
iou_threshold = 0.5


def prepare_input(image):
    input_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Resize input image
    input_img = cv2.resize(input_img, (input_width, input_height))

    # Scale input pixel values to 0 to 1
    return input_img


def rescale_boxes(boxes):
    # Rescale boxes to original image dimensions
    input_shape = np.array([input_width, input_height, input_width, input_height])
    boxes = np.divide(boxes, input_shape, dtype=np.float32)
    boxes *= np.array([img_width, img_height, img_width, img_height])
    return boxes


def extract_boxes(predictions):
    # Extract boxes from predictions
    boxes = predictions[:, :4]

    # Scale boxes to original image dimensions
    boxes = rescale_boxes(boxes)

    # Convert boxes to xyxy format
    boxes_ = np.copy(boxes)
    boxes_[..., 0] = boxes[..., 0] - boxes[..., 2] * 0.5
    boxes_[..., 1] = boxes[..., 1] - boxes[..., 3] * 0.5
    boxes_[..., 2] = boxes[..., 0] + boxes[..., 2] * 0.5
    boxes_[..., 3] = boxes[..., 1] + boxes[..., 3] * 0.5

    return boxes_


def process_output(output):
    predictions = np.squeeze(output[0])

    # Filter out object confidence scores below threshold
    obj_conf = predictions[:, 4]
    predictions = predictions[obj_conf > conf_threshold]
    obj_conf = obj_conf[obj_conf > conf_threshold]

    # Multiply class confidence with bounding box confidence
    predictions[:, 5:] *= obj_conf[:, np.newaxis]

    # Get the scores
    scores = np.max(predictions[:, 5:], axis=1)

    # Filter out the objects with a low score
    valid_scores = scores > conf_threshold
    predictions = predictions[valid_scores]
    scores = scores[valid_scores]

    # Get the class with the highest confidence
    class_ids = np.argmax(predictions[:, 5:], axis=1)

    # Get bounding boxes for each object
    boxes = extract_boxes(predictions)

    # Apply non-maxima suppression to suppress weak, overlapping bounding boxes
    # indices = nms(boxes, scores, self.iou_threshold)
    try:
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), conf_threshold, iou_threshold).flatten()
        return boxes[indices], scores[indices], class_ids[indices]

    except:
        return [np.zeros(4)], [0],[0]


def parse_processed_output(outputs):
    scores = np.squeeze(outputs[output_names.index('score')])
    predictions = outputs[output_names.index('batchno_classid_x1y1x2y2')]

    # Filter out object scores below threshold
    valid_scores = scores > conf_threshold
    predictions = predictions[valid_scores, :]
    scores = scores[valid_scores]

    # Extract the boxes and class ids
    # TODO: Separate based on batch number
    batch_number = predictions[:, 0]
    class_ids = predictions[:, 1]
    boxes = predictions[:, 2:]

    # In postprocess, the x,y are the y,x
    boxes = boxes[:, [1, 0, 3, 2]]

    # Rescale boxes to original image dimensions
    boxes = rescale_boxes(boxes)

    return boxes, scores, class_ids


def draw_detections(image, boxes, scores, class_ids):
    for box, score, class_id in zip(boxes, scores, class_ids):
        x1, y1, x2, y2 = box.astype(int)

        # Draw rectangle
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), thickness=2)
        if class_id == 0:
            label = "[{}]".format("Hoop")
        else:
            label = "[{}]".format("Other")
        label = f'{label} {int(score * 100)}%'
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        # top = max(y1, labelSize[1])
        # cv.rectangle(frame, (left, top - round(1.5 * labelSize[1])), (left + round(1.5 * labelSize[0]), top + baseLine), (255,255,255), cv.FILLED)
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), thickness=2)
    return image

def detect(src_img):
  input_img = prepare_input(src_img)
  blob = cv2.dnn.blobFromImage(input_img, 1 / 255.0)

  # Perform inference on the image
  net.setInput(blob)

  # Runs the forward pass to get output of the output layers
  outputs = net.forward(output_names)

  if has_postprocess:
      boxes, scores, class_ids = parse_processed_output(outputs)

  else:
      # Process output data
      boxes, scores, class_ids = process_output(outputs)

  # print("outputs:", outputs)
  # print("Boxes:", boxes)
  # print("scores:", scores)
  print("class_ids:", class_ids)

  return boxes, scores, class_ids

"--------------------------------------------------------"

while True:

    ret, depth_frame, color_frame = dc.get_frame()
    img_height, img_width = color_frame.shape[:2]
    boxes, scores, class_ids = detect(color_frame)
    output_img = color_frame

    if boxes[0][0] > 0:
        if len(boxes) > 1:
            x_list = []
            y_list = []
            print(type(x_list))
            for box in boxes:
                x_list.append(boxes[0][0])
                y_list.append(boxes[0][1])
            combined_box = [min(x_list), min(y_list), max(x_list), max(y_list)]

        else:
            combined_box = boxes[0]

        print(combined_box)
        x = int(combined_box[0])
        y = int(combined_box[1])
        w = int(combined_box[2]-combined_box[0])
        h = int(combined_box[3]-combined_box[1])
        print(len(combined_box))

        errorPan_lr = (x + w / 2) - 640 / 2
        errorPan_ud = (y + h / 2) - 640 / 2
        print('errorPanlr', errorPan_lr)
        print('errorPanud', errorPan_ud)

        if abs(errorPan_lr) > 80:
            poslr = int(poslr - errorPan_lr / 30)
        if poslr > 160:
            poslr = 160
            print("Out of range")
        if poslr < 5:
            poslr = 5
            print("out of range")

        if abs(errorPan_ud) > 80:
            posud = int(posud - errorPan_ud / 30)
        if posud > 160:
            posud = 160
            print("Out of range")
        if posud < 5:
            posud = 5
            print("out of range")

        depths=[]

        for i in range(int(combined_box[0]),int(combined_box[2])):

            for j in range(int(combined_box[1]),int(combined_box[3])):

                try:
                    if depth_frame[j, i] > 0:
                        depths.append(depth_frame[j, i])

                except:

                    pass

        if len(depths) != 0:
            current_depth = min(depths)
            output_img = draw_detections(color_frame.copy(), [combined_box], [max(scores)], class_ids)
            cv2.putText(output_img, "{}mm".format(current_depth), (int(combined_box[0]), int(combined_box[1] - 20)), cv2.FONT_HERSHEY_PLAIN, 2,(0, 0, 0), 2)
            print(current_depth)

    try:
        servoPos_lr = 'a' + str(poslr) + '\r'
        ser.write(servoPos_lr.encode())
        time.sleep(0.05)
        error=0
    except:
        servoPos_lr = 'a' + str(pre_lr) + '\r'
        ser.write(servoPos_lr.encode())
        time.sleep(0.05)
        error=1

    try:
        servoPos_ud = 'c' + str(posud) + '\r'
        ser.write(servoPos_ud.encode())
        time.sleep(0.05)
        error=0
    except:
        servoPos_ud = 'c' + str(pre_ud) + '\r'
        ser.write(servoPos_ud.encode())
        time.sleep(0.05)
        error=1

    if error == 0:
        pre_lr = poslr
        pre_ud = posud


    cv2.imshow('Detected', output_img)
    time.sleep(0.05)
    if cv2.waitKey(1) & 0xff == 27:
        break

ser.close()