###Project implements a basic realtime lane and vehicle tracking using OpenCV.###

##### Screenshots: #####
<img src="img/screen1.png" width="400">
<img src="img/screen2.png" width="400">

##### Implemented with: #####

* OpenCV 2.3
* C/C++ using Microsoft Visual Studio 2010 IDE.

##### OpenCV features used & used techniques: #####

* Gaussian smoothing for image noise removal
* Canny edge detection [1]
* Hough transform for line detection
* Haar features for vehicle detection (hypothesis generation) [2]
* Vehicle hypothesis verification using horizontal edges and symmetry [3]

##### Possible improvements: #####

* k-Means clustering for Hough lines
* Kalman/Gabor/RANSAC filtering of sampled data
* KLT (Kanade-Lucas-Tomasi) feature tracker for vehicle tracking
* Vanishing point detection using Gaussian probability model
* Better lane tracking(probability methods), stability & accuracy
* More accurate vehicle hypothesis checking
* Alternative IPM(inverse perspective mapping) lane detection method
* Road area extraction & detection for roads without lanes
* Ability to process night vision situations
* Include road shadow removal
* Speed upgrades
* Road sign and traffic lights detection

##### References: #####

1. Canny, J., "A Computational Approach To Edge Detection", IEEE Trans. Pattern Analysis and Machine Intelligence, 1986
2. Viola and Jones, "Rapid object detection using a boosted cascade of simple features", Computer Vision and Pattern Recognition, 2001
3. King Hann Lim et al. "Lane-Vehicle Detection and Tracking", IMECS, 2009

Training data used from: California Institute of Technology SURF project

##### Projects using our source code base & samples (YouTube video): #####
[![Example video](http://img.youtube.com/vi/NG3POm989fU/0.jpg)](https://www.youtube.com/watch?v=NG3POm989fU)

[Example project homepage](http://www.prodigyproductionsllc.com/articles/programming/lane-detection-with-opencv-and-c/)
