// ascii code
// control characters 0-31 plus 127
// printable characters 32-126
const static byte font_data[] = {
  0x55, 0x2A, 0x55, 0x2A, 0x55,  // 0x00 (checker 0) (was ascii nul)
  0x2A, 0x55, 0x2A, 0x55, 0x2A,  // 0x01 (checker 1) (was ascii soh)
  0x00, 0x14, 0x00, 0x00, 0x00,  // 0x02 (left :) (was ascii stx)
  0x00, 0x00, 0x00, 0x14, 0x00,  // 0x03 (right :) (was ascii etx)
  0x18, 0x3C, 0x7E, 0x3C, 0x18,  // 0x04 (eot)
  0x1C, 0x57, 0x7D, 0x57, 0x1C,  // 0x05 (enq)
  0x1C, 0x5E, 0x7F, 0x5E, 0x1C,  // 0x06 (ack)
  0x00, 0x18, 0x3C, 0x18, 0x00,  // 0x07 (bel)
  0xFF, 0xE7, 0xC3, 0xE7, 0xFF,  // 0x08 (bs)
  0x00, 0x18, 0x24, 0x18, 0x00,  // 0x09 (tab)
  0xFF, 0xE7, 0xDB, 0xE7, 0xFF,  // 0x0A (lf)
  0x30, 0x48, 0x3A, 0x06, 0x0E,  // 0x0B (vt)
  0x26, 0x29, 0x79, 0x29, 0x26,  // 0x0C (np)
  0x40, 0x7F, 0x05, 0x05, 0x07,  // 0x0D (cr)
  0x40, 0x7F, 0x05, 0x25, 0x3F,  // 0x0E (so)
  0x5A, 0x3C, 0xE7, 0x3C, 0x5A,  // 0x0F (si)
  0x7F, 0x3E, 0x1C, 0x1C, 0x08,  // 0x10 (dle)
  0x08, 0x1C, 0x1C, 0x3E, 0x7F,  // 0x11 (dc1)
  0x14, 0x22, 0x7F, 0x22, 0x14,  // 0x12 (dc2)
  0x5F, 0x5F, 0x00, 0x5F, 0x5F,  // 0x13 (dc3)
  0x06, 0x09, 0x7F, 0x01, 0x7F,  // 0x14 (dc4)
  0x00, 0x66, 0x89, 0x95, 0x6A,  // 0x15 (nak)
  0x60, 0x60, 0x60, 0x60, 0x60,  // 0x16 (syn)
  0x94, 0xA2, 0xFF, 0xA2, 0x94,  // 0x17 (etb)
  0x08, 0x04, 0x7E, 0x04, 0x08,  // 0x18 (can)
  0x10, 0x20, 0x7E, 0x20, 0x10,  // 0x19 (em)
  0x08, 0x08, 0x2A, 0x1C, 0x08,  // 0x1A (eof)
  0x08, 0x1C, 0x2A, 0x08, 0x08,  // 0x1B (esc)
  0x1E, 0x10, 0x10, 0x10, 0x10,  // 0x1C (fs)
  0x0C, 0x1E, 0x0C, 0x1E, 0x0C,  // 0x1D (gs)
  0x30, 0x38, 0x3E, 0x38, 0x30,  // 0x1E (rs)
  0x06, 0x0E, 0x3E, 0x0E, 0x06,  // 0x1F (us)
  0x00, 0x00, 0x00, 0x00, 0x00,  // 0x20 space
  0x00, 0x00, 0x5F, 0x00, 0x00,  // 0x21 !
  0x00, 0x07, 0x00, 0x07, 0x00,  // 0x22 "
  0x14, 0x7F, 0x14, 0x7F, 0x14,  // 0x23 #
  0x24, 0x2A, 0x7F, 0x2A, 0x12,  // 0x24 $
  0x23, 0x13, 0x08, 0x64, 0x62,  // 0x25 %
  0x36, 0x49, 0x56, 0x20, 0x50,  // 0x26 &
  0x00, 0x08, 0x07, 0x03, 0x00,  // 0x27 '
  0x00, 0x1C, 0x22, 0x41, 0x00,  // 0x28 (
  0x00, 0x41, 0x22, 0x1C, 0x00,  // 0x29 )
  0x2A, 0x1C, 0x7F, 0x1C, 0x2A,  // 0x2A *
  0x08, 0x08, 0x3E, 0x08, 0x08,  // 0x2B +
  0x00, 0x40, 0x20, 0x00, 0x00,  // 0x2C ,
  //0x00, 0x80, 0x70, 0x30, 0x00,  // 0x2C alt , font
  0x00, 0x08, 0x08, 0x08, 0x00,  // 0x2D -
  0x00, 0x00, 0x40, 0x00, 0x00,  // 0x2E .
  //0x00, 0x00, 0x60, 0x60, 0x00,  // 0x2E alt . font
  0x20, 0x10, 0x08, 0x04, 0x02,  // 0x2F /
  0x3E, 0x41, 0x41, 0x41, 0x3E,  // 0x30 0
  //0x3E, 0x51, 0x49, 0x45, 0x3E,  // 0x30 alt 0 font
  0x00, 0x42, 0x7F, 0x40, 0x00,  // 0x31 1
  0x42, 0x61, 0x51, 0x49, 0x46,  // 0x32 2
  0x22, 0x41, 0x49, 0x49, 0x36,  // 0x33 3
  //0x21, 0x41, 0x49, 0x4D, 0x33,  // 0x33 alt 3 font
  0x18, 0x14, 0x12, 0x7F, 0x10,  // 0x34 4
  0x27, 0x45, 0x45, 0x45, 0x39,  // 0x35 5
  0x3C, 0x4A, 0x49, 0x49, 0x30,  // 0x36 6
  0x01, 0x01, 0x79, 0x05, 0x03,  // 0x37 7
  0x36, 0x49, 0x49, 0x49, 0x36,  // 0x38 8
  0x06, 0x49, 0x49, 0x29, 0x1E,  // 0x39 9
  0x00, 0x00, 0x14, 0x00, 0x00,  // 0x3A :
  0x00, 0x40, 0x24, 0x00, 0x00,  // 0x3B ;
  0x00, 0x08, 0x14, 0x22, 0x41,  // 0x3C <
  0x00, 0x14, 0x14, 0x14, 0x00,  // 0x3D =
  0x00, 0x41, 0x22, 0x14, 0x08,  // 0x3E >
  0x02, 0x01, 0x59, 0x09, 0x06,  // 0x3F ?
  0x3E, 0x41, 0x5D, 0x59, 0x4E,  // 0x40 @
  0x7C, 0x12, 0x11, 0x12, 0x7C,  // 0x41 A
  0x7F, 0x49, 0x49, 0x49, 0x36,  // 0x42 B
  0x3E, 0x41, 0x41, 0x41, 0x22,  // 0x43 C
  0x7F, 0x41, 0x41, 0x41, 0x3E,  // 0x44 D
  0x7F, 0x49, 0x49, 0x49, 0x41,  // 0x45 E
  0x7F, 0x09, 0x09, 0x09, 0x01,  // 0x46 F
  0x3E, 0x41, 0x41, 0x51, 0x73,  // 0x47 G
  0x7F, 0x08, 0x08, 0x08, 0x7F,  // 0x48 H
  0x00, 0x41, 0x7F, 0x41, 0x00,  // 0x49 I
  0x20, 0x40, 0x41, 0x3F, 0x01,  // 0x4A J
  0x7F, 0x08, 0x14, 0x22, 0x41,  // 0x4B K
  0x7F, 0x40, 0x40, 0x40, 0x40,  // 0x4C L
  0x7F, 0x02, 0x1C, 0x02, 0x7F,  // 0x4D M
  0x7F, 0x04, 0x08, 0x10, 0x7F,  // 0x4E N
  0x3E, 0x41, 0x41, 0x41, 0x3E,  // 0x4F O
  0x7F, 0x09, 0x09, 0x09, 0x06,  // 0x50 P
  0x3E, 0x41, 0x51, 0x21, 0x5E,  // 0x51 Q
  0x7F, 0x09, 0x19, 0x29, 0x46,  // 0x52 R
  0x26, 0x49, 0x49, 0x49, 0x32,  // 0x53 S
  0x01, 0x01, 0x7F, 0x01, 0x01,  // 0x54 T
  0x3F, 0x40, 0x40, 0x40, 0x3F,  // 0x55 U
  0x1F, 0x20, 0x40, 0x20, 0x1F,  // 0x56 V
  0x3F, 0x40, 0x38, 0x40, 0x3F,  // 0x57 W
  0x63, 0x14, 0x08, 0x14, 0x63,  // 0x58 X
  0x03, 0x04, 0x78, 0x04, 0x03,  // 0x59 Y
  0x61, 0x51, 0x49, 0x45, 0x43,  // 0x5A Z 90
  //0x61, 0x59, 0x49, 0x4D, 0x43,  // 0x5A alt Z font
  0x00, 0x7F, 0x41, 0x41, 0x41,  // 0x5B [
  0x02, 0x04, 0x08, 0x10, 0x20,  // 0x5C backslash
  // (do not use backslash char here as it commented out following line) 
  0x00, 0x41, 0x41, 0x41, 0x7F,  // 0x5D ]
  0x04, 0x02, 0x01, 0x02, 0x04,  // 0x5E ^
  0x40, 0x40, 0x40, 0x40, 0x40,  // 0x5F _
  0x00, 0x03, 0x07, 0x08, 0x00,  // 0x60 ` (grave accent)
  0x20, 0x54, 0x54, 0x78, 0x40,  // 0x61 a 97
  0x7F, 0x28, 0x44, 0x44, 0x38,  // 0x62 b
  0x38, 0x44, 0x44, 0x44, 0x28,  // 0x63 c
  0x38, 0x44, 0x44, 0x28, 0x7F,  // 0x64 d
  0x38, 0x54, 0x54, 0x54, 0x18,  // 0x65 e
  0x00, 0x08, 0x7E, 0x09, 0x02,  // 0x66 f
  0x18, 0xA4, 0xA4, 0x9C, 0x78,  // 0x67 g
  0x7F, 0x08, 0x04, 0x04, 0x78,  // 0x68 h
  0x00, 0x44, 0x7D, 0x40, 0x00,  // 0x69 i
  0x20, 0x40, 0x40, 0x3D, 0x00,  // 0x6A j
  0x7F, 0x10, 0x28, 0x44, 0x00,  // 0x6B k
  0x00, 0x41, 0x7F, 0x40, 0x00,  // 0x6C l
  0x7C, 0x04, 0x78, 0x04, 0x78,  // 0x6D m
  0x7C, 0x08, 0x04, 0x04, 0x78,  // 0x6E n
  0x38, 0x44, 0x44, 0x44, 0x38,  // 0x6F o
  0xFC, 0x18, 0x24, 0x24, 0x18,  // 0x70 p
  0x18, 0x24, 0x24, 0x18, 0xFC,  // 0x71 q
  0x7C, 0x08, 0x04, 0x04, 0x08,  // 0x72 r
  0x48, 0x54, 0x54, 0x54, 0x24,  // 0x73 s
  0x04, 0x04, 0x3F, 0x44, 0x24,  // 0x74 t
  0x3C, 0x40, 0x40, 0x20, 0x7C,  // 0x75 u
  0x1C, 0x20, 0x40, 0x20, 0x1C,  // 0x76 v
  0x3C, 0x40, 0x30, 0x40, 0x3C,  // 0x77 w
  0x44, 0x28, 0x10, 0x28, 0x44,  // 0x78 x
  0x0C, 0x10, 0x10, 0x10, 0x7C,  // 0x79 y
  0x44, 0x64, 0x54, 0x4C, 0x44,  // 0x7A z
  0x00, 0x08, 0x36, 0x41, 0x00,  // 0x7B {
  0x00, 0x00, 0x77, 0x00, 0x00,  // 0x7C |
  0x00, 0x41, 0x36, 0x08, 0x00,  // 0x7D }
  0x02, 0x01, 0x02, 0x04, 0x02};  // 0x7E ~
  // not included 
  // 0x7F (127) would be ascii DEL (delete)
  // 0s80 (128) to 0xFF (255) are extended ascii used in 1981 IBM PC.

void get_char_fd_array(char char_out)
{
byte char_base_addr;

//Serial.begin(9600); // for serial monitor
//while (!Serial) {}; // wait for serial port to connect.
//Serial.print("Character is: ");
//Serial.println(char_out);

char_base_addr = byte(char_out);
//Serial.print("Hex address is: ");
//Serial.println(char_base_addr, HEX);
//Serial.println();
//Serial.println("Flip disc display shows: ");
//Serial.println();

for (int c = 0; c <= 4; c++){
  for (int r = 0; r <= 6; r++){
    fd_array[c][r] = ((font_data[(char_base_addr * 5) + c]) >> r) & 0x01; //bitwise and
    }  
  }

//for (int r = 0; r <= 6; r++){
//  for (int c = 0; c <=4; c++){
//    Serial.print(fd_array[c][r]);
//    Serial.print("  ");
//    } 
//  Serial.println();
//  }
}



// !"#$%&'()*+,-./:;<=>?@[\]^_`{|}~
