# Image_Filtering

This assignment is consists of 2 main codes to evaluate the Joint Bailateral Upsampling and the Iterative upsampling algorithms to enhance the disparity map from the added noise as well as getting the information from a high resolution image which is the RGB image to make the low resolution image which is desparity map to be upsampled to a high resolution one.

* A GUI is created to examine different sigma spatial and sigma spectral and as shown from the picture of the GUI; it is required to input 3 data which are the 
window size that affect the sigma spatial as sigma spatial is a function of window size. Also, it requires sigma spectral and output file name to save the produced images and the point clouds of JBU and IU.

* In order to run the first program to execute the two algorithms:
```
./HelloWorld_Exec (image_1 path) (image_2 path) 
```

* If you need to enter new window size and sigma spectral, press "Esc" to repeat the program again.

* To run the code of computing the normals:
```
./surface_normal.py (radius of KDTreeSearch to compute normals)(number of neighboors)
(path of the point cloud generated) (name of the output file with extension xyzn)
```
## **Documentation**

In order to see the full results of the task; you can access the **Assig2_Documentation.pdf**
