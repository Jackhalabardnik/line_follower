# Line-follower

This is a custom line-following robot project, created with love as part of a bachelor's thesis.

This is still a work in progress, mainly due to hardware limitations (a homemade sensor board with homemade sensors). I am trying to use some math tricks to enable faster line tracking. Also, I am still working on PID.

`maths1.ods` is a spreadsheet containing data from sensors, which allowed me to develop a good IIR filter that greatly helped to reduce sensor and ADC noise and enabled self-calibration of each sensor.
As a result, this robot is able to follow any line that is 19 mm wide and less reflective than the background.

Why not RTOS? I don't need threads, and this allows me to keep things as simple as possible.
