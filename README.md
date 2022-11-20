ITGT521_3D Graphics and Rendering_Assignment 4 The-Rise-of-Skybox

Name: Yifan Li(EVAN)
Student ID: 6537394

For this assignment：

------------------------------------------------------------------------------> OpenGL Utility Toolkit OpenGL的应用工具包

    Running this code requires GLUT and GMTL
    运行需要GLUT和GMTL
    GLUT download link: https://www.opengl.org/resources/libraries/glut/glut_downloads.php
    GMTL download link: https://sourceforge.net/projects/ggt/files/GMTL%20Documentation/0.6.1/
 

Keyboard inputs for plane and propeller (subpart):

 ------------------------------------------------------> Plane 2 飞机2
 
     i   = moves the plane forward    i 移动飞机向前
     k   = moves the plane backward    k 移动飞机向后
     q,e = rolls the plane     q,e 旋转飞机
     a,d = yaws the plane     a,d 飞机偏航
     x,s = pitches the plane     x,s 飞机俯角
     
 ------------------------------------------------------> Plane 1 飞机1
 
     I   = moves the plane forward    I 移动飞机向前
     K   = moves the plane backward    K 移动飞机向后
     Q,E = rolls the plane     Q,E 旋转飞机
     A,D = yaws the plane     A,D 飞机偏航
     X,S = pitches the plane     X,S 飞机俯角
     
 ------------------------------------------------------> Plane Propeller 飞机配件
 
     r   = Rotates right propeller    r 旋转右边螺旋桨
     y   = Rotates left propeller    y 旋转左边螺旋桨
     t   = Rotates Front propeller    t 旋转前边螺旋桨
     g   = Rotates Subsubpropeller1    g 旋转螺旋桨配件1
     h   = Rotates Subsubpropeller2    h 旋转螺旋桨配件2
     H   = Rotates Subsubpropeller3    H 旋转螺旋桨配件3
     
 ------------------------------------------------------> Select Camera 选择相机
 
     v   = Select camera to view     v 选择相机视角
     b   = Select camera to control    b 选择相机控制
     
 ------------------------------------------------------> Light Control 光源控制
 
     1,3 = translates light up/down (+y/-y)    1,3  调整光源上下
     2,4 = translates light (+z/-z)     2,4  调整光源前后
     5,6 = translates light (+x/-x)     2,4  调整光源左右
     9   = toggles diffuse light on/off      9  打开/关闭漫射光
     8   = toggles specular light on/off      8  打开/关闭镜面反射光
     7   = toggles ambient light on/off      7打开/关闭环境光
     
 ------------------------------------------------------> Mouse Control 鼠标控制
 
     Mouse inputs for world-relative camera:
     Hold left button and drag  = controls azimuth and elevation   
     按住鼠标左键控制相机的方位角和仰角
 
     (Press CTRL (and hold) before left button to restrict to azimuth control only,
      Press SHIFT (and hold) before left button to restrict to elevation control only)  
     (按住 CTRL 并 拖动鼠标左键 可以实现相机方位角的固定旋转
      按住 SHIFT 并 拖动鼠标左键 可以实现相机仰角的固定旋转)
 
      Hold right button and drag = controls distance                
      按住鼠标右键控制相机的缩放距离            
