# Autonomous Raspberry Pi Rover

This project is an autonomous rover I built in my mechatronics class. The rover uses a Raspberry Pi, camera, ultrasonic sensor, motor controller, and a custom electronic power system to drive autonomously and manually through a web server hosted on the Pi.

This was one of my favorite projects because it combined programming, electronics, mechanical design, and problem-solving into one working robot. I enjoyed being able to build something physical, test it, troubleshoot issues, and keep improving it until it worked.

## Features

- Autonomous driving using sensor input
- Manual control through a Raspberry Pi web server
- Live camera integration
- Ultrasonic sensor for obstacle detection
- Camera-based projects, including color detection
- Raspberry Pi-controlled system
- Custom electronic emergency stop system
- Separate power control for main system and motors

## How It Works

The rover is powered and controlled by a Raspberry Pi. The Pi runs a web server that allows the rover to be controlled manually from another device. This makes it possible to drive the rover without needing a traditional remote controller.

For autonomous driving, the rover uses an ultrasonic sensor to detect objects in front of it. This allows the rover to react to obstacles and make driving decisions based on distance readings.

The camera adds another layer of functionality. At first, it was used for visual feedback, but it also opened the door to new projects such as color detection and computer vision experiments.

## Safety System

One important part of this rover is the emergency stop system. I built the electronics with two separate power switches:

1. **Main System Power Switch**  
   Controls the Raspberry Pi and the main electronic system.

2. **Motor Controller Power Switch**  
   Controls power directly to the motor controller.

This setup allows the motors to be shut off quickly while keeping the Raspberry Pi and other systems powered. This makes testing safer because the rover can be stopped without fully shutting down the entire system.

## Skills Used

- Raspberry Pi
- Python
- Web server control
- Basic HTML/CSS interface
- Camera setup
- Ultrasonic sensor integration
- Motor controller wiring
- Basic electronics
- Circuit wiring
- Power management
- Troubleshooting
- Autonomous robotics
- Color detection
- Mechanical assembly

## Project Goals

The main goal of this project was to create a rover that could drive on its own while still allowing manual control when needed. I also wanted to make the rover safer and easier to test by adding a separate motor power switch as an emergency stop.

Another goal was to explore what else could be done with the camera system. After getting the camera working, I was able to experiment with color detection, which helped me understand how computer vision can be used in robotics.

## What I Learned

Through this project, I learned how important it is to combine hardware and software correctly. A small wiring issue, code error, or sensor problem can affect the entire rover. I also learned how to troubleshoot step by step instead of guessing.

This project helped me improve my understanding of robotics, Raspberry Pi programming, sensors, motor control, and electronic safety systems. It also made me more interested in building autonomous systems and using computer vision in future projects.

## Future Improvements

Some future improvements I would like to add include:

- Better obstacle avoidance
- Improved web control interface
- Live video streaming on the web server
- More advanced color tracking
- Line following
- Object detection
- Cleaner wiring and enclosure
- Battery level monitoring
- More reliable autonomous navigation

## Why I Built This

I built this rover because I enjoy learning how mechanical, electrical, and programming systems work together. This project gave me the chance to design, build, code, test, and improve a real robotic system. It was fun, challenging, and one of the projects that made me even more interested in engineering and robotics.
