# VGG16 Faster-RCNN
Implementation of Faster-RCNN.
Based on https://github.com/kbardool/keras-frcnn

This branch is the training and testing for the normal VGG16 model as feature extractor.

### Train
To train, open the Jupyter Notebook `TrainVGG16.ipynb` using Google Colab and execute the command lines in its code cells. The training outputs were reserved in the file.

The pre-trained weights can be accessed at https://github.com/fchollet/deep-learning-models/releases/download/v0.1/vgg16_weights_tf_dim_ordering_tf_kernels.h5
The pre-trained weights need to be put under the current directory directly.

### Test
To test, open the Jupyter Notebook `TestVGG16.ipynb` using Google Colab and execute the command lines in its code cells. The testing outputs were reserved in the file. Note that the due to my mistakes, the output FPS is not valid, the actual FPS can be calculated using 1/average elapsed time.

Due to the inconsistency of our manual labeling process, when testing, the following modification needs to be done in line 25 of `simple_parser.py`:
```python
# During Training,
(filename,class_name,x1,y1,x2,y2) = line_split
# During Testing
(class_name,x1,y1,x2,y2,filename) = line_split
```
Also in line 43 of `simple_parser.py`:
```python
# During Training
img = cv2.imread("trainImgs/"+filename)
# During Testing
img = cv2.imread("testdataset/"+filename)
```

### DataSet:
dataset can be accessed from https://drive.google.com/drive/folders/1x-dTkgSeUCE29mpxbOvpYbs6F6XEsy1p?usp=sharing. Directly put the folder `trainImgs` and `testdataset` under current directory.
