## Webcam Tuner

#### tuner.py

Its a python3 application to facilitate the webcam tuning of Robotino.

#### Running and Understanding the App

* Have Python3, OpenCV and Numpy pre installed;
* Execute the following command on your terminal:
    > python3 tuner.py
* The interface has these elements, from top to bottom, left to right:
    1. Sliders;
    2. Low color;
    3. High color;
    4. The original video input;
    5. The calibration output;

![interface](./webcam_tuner/examples/interface.jpg)

* The sliders are grouped in three sets of two. Each set has a \_L and a \_H,
representing **L**ow and **H**igh.
* Normally colors are represented in a **R**ed **G**reen **B**lue space, in this
application they are represented in **H**ue **S**aturation **B**rightness space.
Visit [this](http://www.colorizer.org) website for more information.
* The two colored spaces bellow the sliders and above the camera view are there
so you can see which colors are being selected.
* You want to calibrate the camera so that each color you want to identify stays
between the Low and High value.

---

#### Tuning

1. Plug the webcam into this computer;
2. Place all the colored objects in the camera's view;
3. Move the sliders until only one of the colors can be seem;
4. Repeat step 3 for all colors;

---

#### Examples

*These values will change depending on lighting conditions, so pay attention to
your environment.*

![exampleAll](./webcam_tuner/examples/all.jpg)

In this image almost every color was isolated, so the output looks the same as
the camera input.

![exampleGreen](./webcam_tuner/examples/green.jpg)

Example values to isolate the green color.

![exampleBlue](./webcam_tuner/examples/blue.jpg)

Example vales to isolate the blue color. Usually with natural light this color
is harder to isolate.

![exampleRed](./webcam_tuner/examples/red.jpg)

Since this was recorded with artificial lights the red is harder to isolate, the
example is a terrible isolation.

## Have Fun!

Created by: guilherme dot vieira dot leite at gmail dot com
