# Vision-Based Object Tracking Robot (Raspberry Pi + OpenCV)

  ## Overview
  This project implements an autonomous Raspberry-Pi robot capable of detecting a colored object using computer vision and maintaining a safe distance from it using an ultrasonic sensor.  
  The robot uses motor control, real-time camera input, and distance sensing to move forward or backward while triggering a buzzer when the object is detected.

  The system integrates:
  - Raspberry Pi GPIO motor control  
  - OpenCV color detection  
  - ultrasonic distance measurement  
  - PWM speed control  
  - real-time decision logic  

  This project demonstrates embedded systems programming, robotics control, and computer vision integration.

  ---

  ## Features
  - Real-time camera object detection (green color)
  - Autonomous forward/backward movement
  - Ultrasonic distance measurement
  - PWM motor speed control
  - Buzzer feedback
  - Safety stopping logic
  - Closed-loop tracking behavior

  ---

  ## Hardware Used
  - Raspberry Pi (with Picamera2)
  - L298N Motor Driver
  - 2 DC motors
  - Ultrasonic sensor (HC-SR04)
  - Buzzer
  - Camera module
  - Battery pack / power supply

  ---

  ## Pin Configuration

  ### Motor Driver (L298N)
  - IN1 → GPIO 20  
  - IN2 → GPIO 16  
  - ENA → GPIO 21  
  - IN3 → GPIO 27  
  - IN4 → GPIO 22  
  - ENB → GPIO 17  

  ### Ultrasonic Sensor
  - TRIG → GPIO 23  
  - ECHO → GPIO 24  

  ### Buzzer
  - BUZZER → GPIO 10  

  ---

  ## Software Stack
  - Python  
  - OpenCV  
  - NumPy  
  - RPi.GPIO  
  - Picamera2  

  ---

  ## How It Works

  1. Camera captures frame  
  2. OpenCV detects green object using HSV threshold  
  3. Ultrasonic sensor measures distance  
  4. Robot decides movement:

      If object detected:
        - too close → move forward
        - too far → move backward
        - within range → stop

      If no object:
        - stop motors
        - buzzer off

  5. Buzzer activates when object detected and robot moving.

  ---

  ## Distance Logic

      Move forward  if distance < 4.2 cm
      Move backward if distance > 10.9 cm
      Stop          if 4.2–10.9 cm

  ---

 
  ## Skills Demonstrated
  - Embedded systems programming  
  - Raspberry Pi GPIO control  
  - Motor driver interfacing  
  - Computer vision with OpenCV  
  - Sensor fusion  
  - Autonomous robotics logic  

  ---

  ## Future Improvements
  - object tracking instead of detection  
  - multi-color detection  
  - obstacle avoidance  
  - SLAM navigation  
  - web dashboard control  

  ---

  ## Author
  Shashank Reddy  
  B.Tech Computer Science
