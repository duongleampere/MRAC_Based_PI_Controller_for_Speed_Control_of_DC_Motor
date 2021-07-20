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

With Small γ_p, γ_i:

![image](https://user-images.githubusercontent.com/69660620/126261778-3b1e7e3d-9830-4d50-867f-3202b00db6be.png)

Comment: 
• With a small gamma value, the controller responds slowly and sometimes has a little overshoot (as small as 0.05%).
• Once in the steady state, the controller is stable and get close to the setpoint value.

With big γ_p, γ_i:

![image](https://user-images.githubusercontent.com/69660620/126261810-742c7aa2-fa0c-4386-ab4b-4fd768b08510.png)

Comment:
• With a large gamma value, the controller responds quickly.
• When entering steady state, for small set value, controller oscillates continuously, with medium set value, controller oscillates less and with large set value, controller works best and hardly oscillated.

With flexible gamma value according to formula: 

γ_p, γ_i = (a*setpoint+b)/c (a=1, b=0, c=1700)
![image](https://user-images.githubusercontent.com/69660620/126262160-533fe810-9150-4bca-828e-a8136c0f63f6.png)

Comment:
With a suitable and flexible way of setting the gamma value according to the set value, the controller results in a fast, stable response, little overshoot and a small setting time.
