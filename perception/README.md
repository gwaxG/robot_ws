# Perception

This ROS package provides feature extractor. Features are extracted along central horizotnal and verical lines. 
First of all, the package calculates postions of features, then it averages values of non-nan pixels around those positions at the square of the size`band_width**2`.
 

### Parameters
* Depth image topic  
`<param name="depth_image_topic" type="str" value="/camera/depth/image_raw" />`
* Image height   
`<param name="image_height" type="int" value="480" />`
* Image width  
`<param name="image_width" type="int" value="640" />`
* Number of vertical features  
`<param name="feature_height" type="int" value="30" />`
* Number of horizontal features  
`<param name="feature_width" type="int" value="12" />`
* Width of feature band  
`<param name="band_width" type="int" value="10" />`
  

### Subscribers
1. One channel depth image topic `/depth_image_topic`.

### Publishers
Calculated features are published at `/features` as "BeamFeatures.msg".

```
Header header
std_msgs/Float32MultiArray horizontal
std_msgs/Float32MultiArray vertical
```

### Launch
`roslaunch perception perception.launch`
