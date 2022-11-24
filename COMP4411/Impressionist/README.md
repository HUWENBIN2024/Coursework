## 项目介绍：
**impressionist是一个互动程序，创造的图片看起来像印象派绘画。它基于Paul Haeberli的一篇论文和一个程序。这是“数字绘画”网站。要创建印象主义的图像，用户加载现有图像并在空白像素画布上绘制一系列“笔触”。这些笔触取自原始图像的颜色，给人一种绘画的感觉**

## 项目来源：CS457 in University of Washington
```
https://courses.cs.washington.edu/courses/cse457/21au/src/impressionist/impressionist.php
```

**proj tasks:**  
[0] 五种Brush: 线，圆，散点，散线，散圆					完成  
[0] 线的粗细、角度								完成  
[0] 线的方向控制：调参、右键定方向、光标轨迹切向、与图像梯度垂直	完成  
[0] 透明度（alpha blending）							完成  
[0.5] 设计新Brush (三角形，etc.)		(完成)  
[0.5] 边缘修剪					    (完成)  
[0.5] 在原图上显示对应位置			完成  
[0.5] 左右图像交换				完成  
[1] 调色刷						完成   
[1] 模糊刷 						完成  
[1] 返回上一步					 (完成)  
[1] 两图像融合					 (完成)  
[1.5] 在画布上添加原图背景，以便对照。			完成  
[1.5] 画到一半换原图						完成  
[1.5] 提前设置好画布上不同位置的透明度			完成  
[1.5] 用User指定的Brush自动作画				    (完成)  
[1.5] Brush方向受另一图像的梯度影响			           (完成)  
[1.5] User手动定义Image的Edge; 刷子在Edge上时被其分为两部分					        (完成)  
[2] User可随意设计Filter Kernel; 该Kernel可被自动Normalize; 用该Kernel处理原图		完成  
[2] 自动生成 Image的Edge; User可选择让刷子在Edge上时被其分为两部分			    (完成)  
[2] 用几种特定的风格自动作画											
[2] 设计一个Brush，用于局部拉伸扭曲图像				    放弃  
[2] Curved Brush Following Image Gradient								  (完成)  
[4] 用大量合适的小图以合适的方式排列成给定的大图 (马赛克照片拼图)					完成  
[4] 给定原视频，自动生成Impressionist版视频							 
[8] Bayesian Matting												 

**some notes for usage:**
Images in folder "Thumbnail8" and "Mosaic" are used for mosaic bonus task.
When checking the functionality of "mosaic", please load images in Mosaic folder.
