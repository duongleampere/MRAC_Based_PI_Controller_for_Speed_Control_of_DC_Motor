# MRAC Based PI Controller for Speed Control of DC Motor

Because github does not allow me to upload all files of my project so I upload cubemx file and core. I think that is sufficent for you to implement this controller on STM32F407

# Components:
- STM32F407 discovery
- DC Servo JGB37-545 DC Geared Motor
- HiLetgo BTS7960 43A High Power Motor Driver Module
- USB CP2102

# Hardware:

![image](https://user-images.githubusercontent.com/69660620/116704851-12f49580-a9f6-11eb-9324-38dbbc69a14d.png)

# Results:

- Square pulse:

![image](https://user-images.githubusercontent.com/69660620/116705055-55b66d80-a9f6-11eb-9b26-1b38e0973f50.png)

- Setpoint = 100 RPM:

![image](https://user-images.githubusercontent.com/69660620/116705158-6e268800-a9f6-11eb-8330-8daa136dff84.png)

=> Output response is fast
=> Although some leap appreared, it was trivial
