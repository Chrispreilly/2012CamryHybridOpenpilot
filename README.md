# 2012 Toyota Camry Hybrid Openpilot


### Openpilot Repo

https://github.com/chrispreilly/openpilot   


### Physical Components

##### Purchased / Standard Compenents
- EON
- (Grey) Panda
- Motor Controller Board
  - See MotorControllerWiring
  
##### Customized Components Needed
- Motor Mount & Steering Wheel Mount
  - See MotorMount example, need to customize this for each car
  
### Ignition
The Panda is plugged into the OBD port, and on my Toyota the ignition pin on the panda matches with the 12V supply. As a work around I've changed in thermald to sense ignition at 13V. Battery voltage is ~12.5V, so it only senses igniton when the car is on and inverter brings the voltage up to ~13.5V.
  

### CAN Communication - Required Messages  
I have all messages being sent over the OBD CAN bus. As long as the steering control and safety check messages don't conflict with nything in your car, you should be able to use the OBD bus.

##### Inputs from car to openpilot:
- Blinker
- Brake Pedal
- Gas Pedal
- Wheel Speed
- Steering Angle
- Cruis Control On/Off

##### Inputs from car to Motor Controller:
- Steering Angle

##### Inputs from Openpilot to Motor Controller:
- Desired Angle

##### Inputs from Motor Controller to Openpilot:
- CAN Safety Check

### Safety
- Belt Tension should never exceed what the driver can override
- Arduino sends a CAN check message every 100ms. If openpilot doesn't hear the message for 250ms it will disengage
- If Arduino sees a gap betwwen recieving Desired Angle from Openpilot or Current Angle from the car it will turn off the motor
- If Openpilot sees a delta between current angle and desired angl of >5 degrees (meaning user override or motor lost power) it will disengage



