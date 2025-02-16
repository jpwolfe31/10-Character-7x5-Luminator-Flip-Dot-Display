# 10-Character-7x5-Luminator-Flip-Dot-Display
10 Character 7x5 Luminator Flip Dot Display

Flip Dot Display Build Information

![IMG_2178](https://github.com/user-attachments/assets/93d4848d-9bd5-4d04-bb2a-334ed9793262)

These Luminator Flip Dot displays were acquired on eBay in 2023.  The upper display was the first acquisition and consists of two side by side five character boards with each character having a 7 by 5 dot matrix.  The lower display was acquired a few months later and consists of three boards each having a 14 by 21 dot matrix.  Both were used on transit busses.

There are two github repositories as the units are controlled quite differently.  The first is for the lower display and is called 14x63 Luminator Flip Dot Display.  The second is for the upper display and is called 10 Character 7x5 Luminator Flip Dot Display.

While having different arrangements of dots, the displays use the same flip dot mechanism and magnetic driver.  The upper display has the row drivers located on the board.  The lower display has no row or column drivers on the board.  

Each display has a small white motion sensor that is used to activate the display.  This saves wear and tear and some power when no one is around.  In time mode, the colon moves left or right each second.

10 Character 7x5 Display Boards (5 characters each) (top boards in photo).

These two display boards are each designated as a slave display and each has a jumper setting for center or end.  My obvious guess is that there is a master display board as well that would normally be at the front that controls the other two.

The electronics used to control the two displays are on a pcb board attached to the rear of the front display board with a 40 pin female pcb mounted connector utilizing the connector already on the display board.  The control board also feeds the end display board with the same signals through another 40 pin connector and ribbon cable to the 40 pin connector already on that board.

Each display board has row drivers installed (see the schematic for the display boards).  The control board attached to the front display has the column drivers needed to complete the drive of the flip dots.  The schematic includes details on the control board as well as the two display boards which are the same and only differ with respect to the jumper setting (center or end).  T
The boards are supported with a bolt and a couple of nuts.  The end of the bolt on the front is covered with an automotive vacuum plug.

The control board uses an Arduino MKR1010 along with the Blynk iPhone app to send commands to the MKR1010 over the internet.

Unfortunately, Blynk is not supporting new makers on its app right now, but I believe this program could be modified to work with the Aurduino Cloud IOT and its messenger widget.  Also, I would probably use a Nano-ESP32 if I were to build the board again as it is newer, cheaper and seems to have better wifi connectivity.  

See folders above for code, schematics, build information and printed circuit boards. Feel free to email me at jpwolfe31@yahoo.com if you have any questions.
