# Potentia
This is a personal project aimed at self learning and letting me practice CAD design and general IK. I aim to create a bare-bones semi-autonomous robot dog that can path-find around in a simple space and compute a motion to reach a point in the room. The final version of the project is steadily coming along as I finalize more design decisions.

---

## CAD & Body
The frame of the robot is made of Reapid PETG 3d print. All CADs of the frame are designed by me, primaily on OnShape. The general design of the leg has been inspired by the genral industry implementations but also very much by [Adam Beedle's design on youtube](https://www.youtube.com/watch?v=bDxItdyQ3jc)

<p align="middle">
  <img width="814" height="650" alt="image" src="https://github.com/user-attachments/assets/72814e43-cc2b-4a8f-a39a-83e78d4c2be4" />
  <img width="1372" height="791" alt="image" src="https://github.com/user-attachments/assets/5df83fdb-6f44-43fa-9e25-6ee47f161eda" />
</p>


Both the current leg and body will need a redesign to reduce weight at some point.

## Hardware
* `ESP32` onboard is the dedicated processor for basic IK and SLAM. Might be replaced later for a beefier alternative
* Currently all legs are actuated by `MG996R servos` ran by a `PCA9865` servo driver board.
* An internal battery can be implemented later with a power distribution solution but for now the power is supplied from my PC and a seperate DC power supply for the servos (6V)

## Software
* [Cobalt](https://github.com/IDirkI/Cobalt) is used for general vectors, math and transform types it implements as well as its numerical IK implementation.
* Software uses the `Adafurit PCA9865` library to talk to the board for now. This will be replaced once Cobalt has its PCA9865 driver implemented
* The IK equations are currently precomputed by hand beforehand. Once Cobalt's generalized IK is released, the project will switch to it for convinience

### Computation of IK equations
<img width="589" height="736" alt="image" src="https://github.com/user-attachments/assets/184c1621-800b-4aa1-bbbe-51efd9560c08" />
<img width="492" height="697" alt="image" src="https://github.com/user-attachments/assets/cc806489-ed88-4163-91f9-5d9bf6437085" />
<img width="488" height="695" alt="image" src="https://github.com/user-attachments/assets/ac5de58b-ef58-4a85-9806-b0bcd60c2b22" />

## Prototype Test
### MK-1 Test (analytical IK, baseline)
The leg's analytic IK equations work and allow the foot to be moved freely within the workspace of the leg.

The test consists of a series of motions: 5 circle, 3 horizontal line & 3 verticle line motions. Afterwards it repeats itself. The test works as expected and one addition in the future may be to smooth out transitions between two motions/frames with an s curve perhaps.

https://github.com/user-attachments/assets/b065dd97-b819-4b19-8fcd-185ff3592873

### MK-2 Test (numerical IK)
Project now relies on (Cobalt)[https://github.com/IDirkI/Cobalt] for its IK calculations using a `.rob` file and Cobalt's jacobian psuedo-inverse with DLS method for numerical IK.

The end effector will follow the trajectories set by:
1. $(x, y) = (0, -0.17 + 0.065\sin t)  \longrightarrow$ vertical line
2. $(x, y) = (0.14\cos t, -0.17)\longrightarrow$ horizontal line
3. $(x, y) = (0.06\cos t, -0.15 + 0.06\sin t)\longrightarrow$ circle line
4. $(x, y) = \left(\frac{0.18\cos t}{\sin^2 t}, -0.16 + \frac{0.18\cos t\, \sin t}{\sin^2 t}\right)\longrightarrow$ infinity symbol (lemniscate)

https://github.com/user-attachments/assets/93de009b-6c64-46c1-8bfc-6bf6f99d2122

## Prototyping Cost
Thusfar, several failed prints & 2 broken servo's

![20251014_182952](https://github.com/user-attachments/assets/c2f517c8-d9ca-41b9-b29d-4948e94bbb23)
