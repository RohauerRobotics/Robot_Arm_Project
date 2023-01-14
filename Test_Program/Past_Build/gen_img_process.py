# general image processing, primarily for overhead images
# uses resnet and pytorch to process image data and output 
# image and box coordinates

from turtle import back
from unicodedata import decimal
import torch
from torchvision.transforms import ToTensor
from PIL import Image
from torchvision.models import detection
import cv2
import numpy as np

class GenImgProcess(object):
	def __init__(self) -> None:
		# load model parameters
		self.model_names = [
                    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
                    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
                    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
                    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
                    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
                    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
                    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
                    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
                    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
                ]
				
		# set the device we will be using to run the model
		self.DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

		self.PATH = 'resnet.pth'

		# define model version, load saved parameters, declare compute device
		# and declare evaluation mode
		self.model = detection.fasterrcnn_resnet50_fpn()
		self.model.load_state_dict(torch.load(self.PATH))
		self.model.to(self.DEVICE)
		self.model.eval()
		print("Successfully loaded pretrained resnet!")

		self.weights = detection.FasterRCNN_ResNet50_FPN_Weights.COCO_V1
		self.weights = self.weights.meta["categories"]

	def execute_model(self, image):
		black = (0, 0, 0)
		image = self.mod_image(image)
		object_lib = {}
		# send the input to the device and pass the it through the network to
		# get the detections and predictions
		image = image.to(self.DEVICE)
		with torch.inference_mode():
			detections = self.model(image)[0]
			for i in range(0, len(detections["boxes"])):
				# extract the confidence (i.e., probability) associated with the
				# # prediction
				# print("Confidence: ", np.round(detections["scores"][i].detach().cpu().numpy(), decimals=3))
				confidence = np.round(detections["scores"][i].detach().cpu().numpy(), decimals=3)
				if confidence > 0.75:
					idx = int(detections['labels'][i])
					box = detections["boxes"][i].detach().cpu().numpy()
					(startX, startY, endX, endY) = box.astype("int")
					img_label = f"{self.model_names[idx]}: {round(confidence*100, 3)}%"
					print(f"Info: {img_label}")
					# draw box around image
					cv2.rectangle(self.orig, (startX, startY), (endX,endY), black, 3)
					text_Y = startY - 15 if (startY-15 >15) else startY + 15
					cv2.putText(self.orig, img_label, (startX, text_Y), 
									cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1, black, 3)
					object_lib[self.model_names[idx]] = [startX, startY, endX, endY]

		return self.orig, object_lib

	def mod_image(self, image):
		print(f"Image Shape: {image.shape}")
		# cv2.resize(image, (1280, 1280))
		# copy original for adding bounding boxes to detected objects
		self.orig = image.copy()
		# convert the image from BGR to RGB channel ordering and change the
		# image from channels last to channels first ordering
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		image = image.transpose((2, 0, 1))
		# add the batch dimension, scale the raw pixel intensities to the
		# range [0, 1], and convert the image to a floating point tensor
		image = np.expand_dims(image, axis=0)
		image = image / 255.0
		image = torch.FloatTensor(image)
		
		return image

# test class version of resnet test
# test = GenImgProces()
# frame = cv2.imread("remote_test.jpg")
# frame = cv2.resize(frame, (1920, 1080))
# out_img, object_lib = test.execute_model(frame)
# print(f"Object Lib: \n{object_lib}")

# out_img = cv2.resize(out_img, (1280, 720))
# cv2.imshow("Display Image", out_img)
# cv2.waitKey(0)
