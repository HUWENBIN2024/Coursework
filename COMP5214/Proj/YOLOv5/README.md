# Facial Mask Recognition
Background: Covid-19 is getting serious and wearing masks can be benificial to the public hygiene. As CCTV is used commonly today, we can apply a real-time detection method to a camera to recognize the mask wearing situation.

We use object detection method to do mask recognition. It can distinguish 5 classes: 'no mask', 'improper', 'proper surgical', 'proper N95', 'proper cloth'. 

No mask: detect faces without a mask  
Improper: detect someone wearing a mask but in an improper way  
Proper surgical/N95/Cloth: recognize the class of masks  

We use yolo so that it can apply to a camera and do a real time detection. Demo video can be seen via a link below.

## Model: YOLOv5

PyTorch implementation of YOLOv5: Real-Time Facial Mask Detection. Forked from: [https://github.com/ultralytics/yolov5](https://github.com/ultralytics/yolov5). It is shown below: 
![model](/yolov5.png "model")

## Custom setting

We saved 5 classes name into data/data.yaml.

```python
yaml_text = """train: data/train/images
val: data/train/images

nc: 5
names: ['no mask', 'improper', 'proper surgical', 'proper N95', 'proper cloth']"""
```

We modified models/custom_yolov5s.yaml to configurate the classes number. Remaining configurations are the same as default

```python
nc: 5  # number of classes
```

## Usage

### Train

`python train.py --img 640 --batch 32--epochs 50 --data data/data.yaml --cfg models/custom_yolov5s.yaml --weights yolov5s.pt --name yolov5s_results  --cache`

### Detect
We have already train the model for 100 epoch and you can use "best.pt" directly for detection.

Detect some images: `python detect.py --source data/test --weight best.pt --name expTestImage --conf 0.4`

Detect using camera: `python detect.py --source 0 --weight best.pt --name expTestImage --conf 0.4`

## DataSet link
https://drive.google.com/drive/folders/1erKN4l5_LU3mZrY0T8nV-r01RyHVGWgY?usp=sharing  
Download the dataset, copy the 3 folds(train, test, val) and paste to yolov5/data.

## Result
### Demo Video
https://youtu.be/Pxdrc_3d6ss

### Quantitive Result
![result](/results.png "result")

### Example
![demo](/demo.png "demo")
