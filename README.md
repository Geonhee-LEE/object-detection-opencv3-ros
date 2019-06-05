# opencv-ros
Image processing using OpenCV for Labeling and HSV, Filtering

I want to extract the object(e.g., enclosure) from image at the ROS kinectic version.

After the hsv convesion and filtering and labeling, I extracted the ROI for detecting the object(black color) using OpenCV3.

On the ROI area, I did again the hsv convesion for extracing the black color. 

As a result, I can extract the center of x, y about detected object.

<pre><code>roslaunch opencv_pkg conveyer_object_detection.launch  </code></pre>


![HSV & Labeling](img/result.png)
