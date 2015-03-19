Project implements a **basic** realtime lane and vehicle tracking using OpenCV.

**Screenshots:**

<img src='http://opencv-lane-vehicle-track.googlecode.com/svn/trunk/img/screen1.png' width='300' height='200' />
<img src='http://opencv-lane-vehicle-track.googlecode.com/svn/trunk/img/screen2.png' width='300' height='200' />

**Implemented with:**
  * OpenCV 2.3
  * C/C++ using Microsoft Visual Studio 2010 IDE.

**OpenCV features used & implemented techniques:**
  * Gaussian smoothing for image noise removal
  * Canny edge detection `[1]`
  * Hough transform for line detection
  * Haar features for vehicle detection (hypothesis generation) `[2]`
  * Vehicle hypothesis verification using horizontal edges and symmetry `[3]`

**Possible improvements:**
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

**References:**
  * `[1]` Canny, J., "A Computational Approach To Edge Detection", IEEE Trans. Pattern Analysis and Machine Intelligence, 1986
  * `[2]` Viola and Jones, "Rapid object detection using a boosted cascade of simple features", Computer Vision and Pattern Recognition, 2001
  * `[3]` King Hann Lim et al. "Lane-Vehicle Detection and Tracking", IMECS, 2009

**Training data used from:** California Institute of Technology SURF project

Projects using our source code base & samples:

<a href='http://www.youtube.com/watch?feature=player_embedded&v=NG3POm989fU' target='_blank'><img src='http://img.youtube.com/vi/NG3POm989fU/0.jpg' width='425' height=344 /></a>

[Project home page](http://www.prodigyproductionsllc.com/articles/programming/lane-detection-with-opencv-and-c/).


