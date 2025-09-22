# Potentia
This is a personal project aimed at self learning and letting me practice CAD design and general IK. I aim to create a bare-bones semi-autonomous robot dog that can path-find around in a simple space and compute a motion to reach a point in the room. The final version of the project is steadily coming along as I finalize more design decisions.

---

## 🦿 CAD & Body
The frame of the robot is made of Reapid PETG 3d print. All CADs of the frame are designed by me, primaily on OnShape. The general design of the leg has been inspired by the genral industry implementations but also very much by [Adam Beedle's design on youtube](https://www.youtube.com/watch?v=bDxItdyQ3jc)

<img width="814" height="650" alt="image" src="https://github.com/user-attachments/assets/72814e43-cc2b-4a8f-a39a-83e78d4c2be4" />

I feel the legs could use a re-work in the future to reduce weight. They were designed to be eary to model and put together for prototyping but also sturdy enough to support the full weight of the final project.

##  Hardware
* `ESP32` onboard is the dedicated processor for basic IK and SLAM. Might be replaced later for a beefier alternative
* Currently all legs are actuated by `MG996R servos` ran by a `PCA9865` servo driver board.
* An internal battery can be implemented later with a power distribution solution but for now the power is supplied from my PC and a seperate DC power supply for the servos (6V)

## ⚡Software
* [Cobalt](https://github.com/IDirkI/Cobalt) is used for general vectors, math and transform types it implements
* Software uses the `Adafurit PCA9865` library to talk to the board for now. This will be replaced once Cobalt has its PCA9865 driver implemented
* The IK equations are currently precomputed by hand beforehand. Once Cobalt's generalized IK is released, the project will switch to it for convinience

### Computation of IK equations
<img width="589" height="736" alt="image" src="https://github.com/user-attachments/assets/184c1621-800b-4aa1-bbbe-51efd9560c08" />
<img width="492" height="697" alt="image" src="https://github.com/user-attachments/assets/cc806489-ed88-4163-91f9-5d9bf6437085" />
<img width="488" height="695" alt="image" src="https://github.com/user-attachments/assets/ac5de58b-ef58-4a85-9806-b0bcd60c2b22" />

## Prototype Test
The first big prototype test with the single leg was a success.

The leg's IK equations work and allow the foot to be moved freely within the workspace of the leg.

The test consists of a series of motions: 5 circle, 3 horizontal line & 3 verticle line motions. Afterwards it repeats itself. The test works as expected and one addition in the future may be to smooth out transitions between two motions/frames with an s curve perhaps.

https://github.com/user-attachments/assets/b065dd97-b819-4b19-8fcd-185ff3592873


