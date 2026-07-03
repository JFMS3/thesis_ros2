# Crazyflie Brushless 2.1 Assembly

## Unpacking the Module
Unboxing the module you will find the following components
<img width="784" height="470" alt="CrazyflieComponents" src="https://github.com/user-attachments/assets/3c48f837-582a-4669-a39f-8d5330496518" />

## Testing the Control Board
Take out the Control Board and plug in its MicroUSB cable. You should see the front two LEDs glow blue. The bottom right LED should blink green 5 times, before the bottom left periodically blinks red. If this happens, the control board is working as expected!

https://github.com/user-attachments/assets/d4b5383b-3ffd-417e-a5fd-422172104990

## Mounting the motors
Take a motor and 3 short black screws. Each motor should sit on top of the side with the MicroUSB cable. The motor cable should run _along_ the arm and attach to the connectors.

On the underside, screw in the motor, in the following screw holes. The kit comes with a screwdriver you can use for this.

<img width="1000" height="942" alt="PXL_20260703_055547629" src="https://github.com/user-attachments/assets/1e170109-779a-4c13-9a7e-0f6ab0c95004" />

Once all screws have been fastened, turn the control board over, _twist_ the cable 180 degrees and attach it to the connector.

https://github.com/user-attachments/assets/2b518bb4-9502-4d39-b37f-749377b52dca

At this point, the setup should look like this.
<img width="3398" height="3187" alt="PXL_20260703_061437937" src="https://github.com/user-attachments/assets/1ca18e8d-a315-4841-9033-d441b4b53bfe" />

## Attach the propeller guards
Take a propeller guard and hook it on the underside of the motor, at the end of the arm.
On the underside, use a screwdriver to push the clips apart until they rest on the side of the arm. Gently the guard to pull the clips on top of the arm.

https://github.com/user-attachments/assets/05b83ff2-0b25-4a7d-9d5a-c848dba8992d

The propeller arm should be attached like this at the end.

<img width="2281" height="1773" alt="PXL_20260703_062416858" src="https://github.com/user-attachments/assets/dddfeebb-3f99-4aaf-8e03-c8f8e8a956e7" />

> [!NOTE]
> It is possible to attach the legs instead of the motor guard, but this does not protect the propellers. It is *highly* recommended to use the guard if possible.
> In addition, once the guards have been attached, the guard screws can be used to secure them but this is not recommended.

## Attach the propellers
Take out the two sets of propellers. The clockwise (CW) propellers are labelled "5R", while the counter-clockwise are labelled "5L". Ensure that the propellers are connected as follows, with the top side being convex (there should be text on top). To verify the correct propeller is used, the higher side should be facing forward in the direction of rotation.

<img width="1357" height="1018" alt="bl_prop_direction" src="https://github.com/user-attachments/assets/a6ab38c0-1a3d-4022-9ad9-1f9466944ceb" />

The propeller can also be removed using the provided tool.

https://github.com/user-attachments/assets/98f7d253-76ef-4c6f-84b2-6cd948b4e609

## Attach the rubber pad
Place the rubber pad between the expansion pins. Make sure there are no gaps between the pins.

https://github.com/user-attachments/assets/5dfac46b-2643-454a-98bd-9a3178388228

## Attach the long pins
Take out the long pins (not the short ones). Turn the drone over and push the pins through the expansion connectors

https://github.com/user-attachments/assets/d686987a-2691-42af-b54b-c384b689802c

## Attach the battery
Take out the battery and place it on top of the rubber pad. The side with the QR code should be on the left side of the drone. To hold it in place, take the battery holder and push it down on top of the long pins. The yellow side should be face up.

<img width="1736" height="1584" alt="PXL_20260703_070344724" src="https://github.com/user-attachments/assets/8d6962bb-5942-4cd6-881f-197dfd7c580e" />

## Attach expansions
To conclude, attach any expansion decks you may have. For instance, the _Mocap Marker Deck_, used to attach M3 reflective markers to the drone.
Then, connect your battery to its connector.

<img width="3472" height="3188" alt="PXL_20260703_071244109" src="https://github.com/user-attachments/assets/9b9218eb-a0b7-4d53-93e7-a968f407fad7" />

Congratulations! Your CrazyFlie 2.1 Brushless should be fully assembled!

# Connecting to the CrazyFlie

## CrazyRadio setup
To communicate with the CrazyFlie, you will need a CrazyRadio 2.0. These have been ordered for each drone (in a seperate box). Connect the PCB to the antenna and plug into your computer.

When connecting your CrazyRadio for the first time, its LED should glow red.
<img width="3183" height="3463" alt="PXL_20260703_072216345" src="https://github.com/user-attachments/assets/480e97f0-6999-4930-8988-b2a5805a09f3" />
On your computer, the file explorer should also open, containing several files as shown below.
<img width="800" height="318" alt="Screenshot 2026-07-03 172352" src="https://github.com/user-attachments/assets/1a23d5de-09e1-4636-956e-e7a992022d20" />

Download the latest firmware for the CrazyRadio from CrazyFlie's [firmware releases repo](https://github.com/bitcraze/crazyradio2-firmware/releases). Download the latest crazyradio2-*.uf2 file and copy it to the CrazyRadio drive. If you see a Windows error, that is to be expected, just click skip.

Your CrazyRadio should now briefly glow white when connected, meaning it is operational.

# Controlling the CrazyFlie
## Manual Control
To manually control the drone, connect a game controller to your computer and open the UI client.
To do this, pip install the cf-client Python package (ideally in a virtual environment) and run the cf-client command.

```python
py -3.12 -m venv ~/cf-venv  # create virtual environment using Python 3.12 version (use your own version)
source ~/cf-venv/Scripts/activate  # or in Powershell:  & '~/cf-venv/Scripts/Activate.ps1'
pip install cflib  # install the cflib package

cfclient  # open the GUI
# alternatively after installing, you can do & 'C:\Users\Control Lab109\cf-venv\Scripts\cfclient.exe'
```



Once the GUI is open, press *Scan* to find your drone, and then *Connect* to connect to it.
Press the green *ARM* button to warm up the motors.

> [!NOTE]
> If some of your motors don't spin at first that is normal. Just try arming it a few times and letting it run for a few minutes.

https://github.com/user-attachments/assets/9bd1d93d-4ea2-409d-bf09-81a706965427



