/*
 * Logger for TAURIS system.
 * FW Ver: 1.09
 * HW Ver: 3_4
 * Date: 30-Dec-2017
 * 
 * Reference: 
 * https://www.hackster.io/rayburne/sd-card-serial-logger-8a6700
 * http://www.embedds.com/programming-avr-usart-with-avr-gcc-part-2/
 * http://files.support.epson.com/pdf/lx300_/lx300_u1.pdf
 * 
 * https://www.camiresearch.com/Data_Com_Basics/RS232_standard.html
 * 
 * NOTE:
 * DTE Ready (DTR) This signal is asserted (logic '0', positive voltage) by the DTE device when it wishes to open a communications channel. 
 * If the DCE device is a modem, the assertion of DTE Ready prepares the modem to be connected to the telephone circuit, and, once connected, maintains the connection. 
 * When DTE Ready is deasserted (logic '1', negative voltage), the modem is switched to "on-hook" to terminate the connection.
 * IMPORTANT: If the DCE device is not a modem, it may require DTE Ready to be asserted before the device can be used, or it may ignore DTE Ready altogether. 
 * If the DCE device (for example, a printer) is not responding, confirm that DTE Ready is asserted before you search for other explanations.
 */

// include the SD library:
#include <SPI.h>
#include <SD.h>
#include <util/delay.h>

// Define log file name depends on board No.
#define LOG_FILE_NAME "akmT0109.akm"

// Firmware and hardware version
#define FW_VER "109"
#define HW_VER "3_04"

// BuzzerPinOut is shared with TX line for serial.
//#define BUZZER_ENABLE 0
#define DTR_PRINTER_READY LOW
#define DTR_PRINTER_NOT_READY HIGH
//#define DTR_PRINTER_READY 1
//#define DTR_PRINTER_NOT_READY 0
/*
DTE Ready (DTR) This signal is asserted (logic '0', positive voltage) by the DTE device when it wishes to open a communications channel. 
If the DCE device is a modem, the assertion of DTE Ready prepares the modem to be connected to the telephone circuit, and, once connected, maintains the connection. 
When DTE Ready is deasserted (logic '1', negative voltage), the modem is switched to "on-hook" to terminate the connection.
IMPORTANT: If the DCE device is not a modem, it may require DTE Ready to be asserted before the device can be used, or it may ignore DTE Ready altogether. 
If the DCE device (for example, a printer) is not responding, confirm that DTE Ready is asserted before you search for other explanations.
*/

// Received data.
volatile byte Data = 0;

volatile byte IdleState = 0;
volatile byte HandshakeDone = 0;

volatile char DataCharacter; // Variable to store received data converted to ASCII.
volatile byte DataReadyFlag = 0; // Received data ready flag

volatile unsigned long SimpleChecksum = 0; // Accumulate received bytes values as a simple checksum.

// Port pins declaration.
//#ifdef BUZZER_ENABLE
  const int BuzzerPinOut = 8; //PB0 PIN_8/CLK0
//#endif
const int LedGreenOut = 9; //PB1 PIN_9/OC1A
const int chipSelect = 10; // SPI SD CARD chip select PIN
const int DtrFromPrinter = 2; // PD2 INT0 - input - check if prnter can accept new data
const int DtrToTauris = 4; // PD4 - output - set high when data are write to SD to hold on sending data from Tauris
const int CardDetectIn = 3; //PD3 PIN_1
#define BUFFSIZE 128

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;

File myFile;

volatile char bufferChar[BUFFSIZE];
volatile char c; 
volatile byte b;
volatile int bufferidx;

// Timer1 is used to count interval time between following print requests. If timer expire, flush data to SD, close file and reopen it to be ready for next printing w/o power cycle whole device.
void SetupTimer1(void);

void(* resetFunc) (void) = 0; //declare reset function @ address 0

//*******************************************************************
// Timer1 configuration part.
void SetupTimer1()
{
  TCCR1A = 0;  // Clear Timer register 1A
  TCCR1B = 0;  // Clear Timer register 1B
  //OCR1A = 468750;
  OCR1A = 15623;  // Stop counting when this value is reached
  TCCR1B |= (1 << WGM12);  // Set Mode 4, CTC mode on OCR1A is used
  TIMSK1 |= (1 << OCIE1A); // Set interrupt on compare match, interrupt is done every 1 second
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set prescaler to 1024 
}

//*******************************************************************
// Timer1 ISR
ISR (TIMER1_COMPA_vect) { 
  if(IdleState) {
    IdleState++; 
  }
}

//*******************************************************************


// Interrupt service routine for INT1. SD Card Detect routine.
// This IRQ can be fired only if SD card is ejected in runtime.
ISR(INT1_vect) {
  // Disable INT0 interrupt to hold printing - strobe to printer can't be generated.
  cli();
  byte CheckCardDetectIn = 0;
  //EIMSK &= ~(1 << INT0);   // disable INT0 interrupt  
  PORTB &= ~(1<<PB1); //Turn OFF LED
  // Disable IdleState to prevent generating IRQ from Timer.
  IdleState = 0;
  DataReadyFlag = 0;
  SimpleChecksum = 0;
  // Turn ON buzzer.
  #ifdef BUZZER_ENABLE
    PORTB |= (1<<PB0);
  #endif
  
  // Wait until card is injected back into the socket - LOW on CardDetectIn.
  // Initialize SD card from the beggining.
  // Poke CardDetectIn. If it is HIGH then SD card is not inserted. Wait here until SD card will be inserted.
  CheckCardDetectIn = (PIND & B00001000);
  while(CheckCardDetectIn == B00001000){
    // TurnOFF/ON LED
    for (byte loop_cnt = 0 ; loop_cnt < 5 ; loop_cnt++) {
      PORTB &= ~(1<<PB1); 
      #ifdef BUZZER_ENABLE
        PORTB &= ~(1<<PB0);
      #endif
      _delay_ms(200);
      
      PORTB |= (1<<PB1); 
      #ifdef BUZZER_ENABLE
        PORTB |= (1<<PB0);
      #endif
      _delay_ms(200);
    }
    _delay_ms(500);
    CheckCardDetectIn = (PIND & B00001000); 
  }
  
  PORTB &= ~(1<<PB1); //Turn OFF LED
  software_Reset();
}


//*******************************************************************
// Setup of interrupt INT1.
void SetupINT1 () {
  // -----------------------------------------
  // Enable INT1 IRQ for CardDetectIn at the end of setup.
  // -----------------------------------------
  EICRA |= (1 << ISC10);  //01 -> Any logical change on INT1 generates an interrupt request.
  EICRA &= ~(1 << ISC11);  //01 -> Any logical change on INT1 generates an interrupt request.
  EIMSK |= (1 << INT1);   // enable INT1 interrupt
  EIFR |= (1 << INTF1);
  
}

//*******************************************************************
// Restarts program from beginning but does not reset 
// the peripherals and registers.
inline void software_Reset(){
  cli();
  // Disable IdleState to prevent generating IRQ from Timer.
  IdleState = 0;
  DataReadyFlag = 0;
  SimpleChecksum = 0;
  PORTB &= ~(1<<PB1); // Turn OFF LED
  // Turn ON buzzer.
  #ifdef BUZZER_ENABLE
    //digitalWrite(BuzzerPinOut, HIGH);
    PORTB |= (1<<PB0);
  #endif
  
  asm volatile ("  jmp 0");
  //sei();
}   

//*******************************************************************
void setup() {

  pinMode(chipSelect, OUTPUT);  // standard convention required for SD library
  pinMode(LedGreenOut, OUTPUT);
  pinMode(DtrToTauris, OUTPUT);
  pinMode(DtrFromPrinter, INPUT);
  pinMode(CardDetectIn, INPUT); // PULLUP might be not needed as PCB has pullup.
  
  digitalWrite(DtrToTauris, DTR_PRINTER_NOT_READY); // Hold-on sending new data from Tauris until setup() complete.
  
  #ifdef BUZZER_ENABLE
    pinMode(BuzzerPinOut, OUTPUT);
    // Turn ON buzzer until all setup operation will be completed. This is information to outside to 
    // not start printing before setup complete.
    digitalWrite(BuzzerPinOut, HIGH);
  #endif

  
  Serial.begin(9600);  // Use pins 3 Tx and 2 Rx (I/O view: pin 2 receives data to SD write)
  while (!Serial) {
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(200);
    digitalWrite(LedGreenOut, LOW);   // sets the LED off
    delay(200);
    //; // wait for serial port to connect. Needed for native USB port only
  }
  delay(100);
  Serial.print("Serial init at 9600 BAUD\r\n");
  
  
  // -----------------------------------------
  // SD CARD initialization part.
  // -----------------------------------------

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
    // FATAL ERROR
  while(!card.init(SPI_HALF_SPEED, chipSelect)) {
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, HIGH);
    #endif      
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(500);                  // waits for a number of miliseconds      
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, LOW);
    #endif
    digitalWrite(LedGreenOut, LOW);    // sets the LED off
    delay(500);                  // waits for a number of miliseconds
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  // FATAL ERROR
  while(!volume.init(card)) {
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, HIGH);
    #endif      
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(500);                  // waits for a number of miliseconds      
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, LOW);
    #endif
    digitalWrite(LedGreenOut, LOW);    // sets the LED off
    delay(500);                  // waits for a number of miliseconds
  }

  // -----------------------------------------
  // End of SD CARD initialization part.
  // -----------------------------------------

  // -----------------------------------------
  // Create new or open existed file test.txt
  // -----------------------------------------
  // FATAL ERROR
  while(!SD.begin(chipSelect)) {
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, HIGH);
    #endif      
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(500);                  // waits for a number of miliseconds      
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, LOW);
    #endif
    digitalWrite(LedGreenOut, LOW);    // sets the LED off
    delay(500);                  // waits for a number of miliseconds
  }
  
  myFile = SD.open(LOG_FILE_NAME, FILE_WRITE);
  
  
  // if the file opened okay, write to it:
  // if the file didn't open, print an error:
  // FATAL ERROR
  while(!myFile) {
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, HIGH);
    #endif      
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(500);                  // waits for a number of miliseconds      
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, LOW);
    #endif
    digitalWrite(LedGreenOut, LOW);    // sets the LED off
    delay(500);                  // waits for a number of miliseconds
  }

  // -----------------------------------------
  // End Create new or open existed file test.txt
  // -----------------------------------------

  // discard any random input
  while (Serial.read() >= 0) {}

  
  // Write start of LOG indicator at a TOP of logged record. It need to be added to differentiate case for SD card hot plug out.
  myFile.println("");  
  myFile.print("!-!-md5="); myFile.print(SimpleChecksum); 
  myFile.print("-!-!FW="); myFile.print(FW_VER);
  myFile.print("-!-!HW="); myFile.print(HW_VER); 
  //myFile.print("-!-!SD="); myFile.print(CardWasMovedOut);
  myFile.print("-!-!START_LOG");
  myFile.print("-!-!");
  myFile.println("");
  myFile.flush();
  _delay_ms(200);
  
  // Turn ON LED to indicate that device starts correctly.
  digitalWrite(LedGreenOut, HIGH);   // sets the LED on

  // Turn OFF buzzer. Setup is completed and printing can be started.
  digitalWrite(BuzzerPinOut, LOW);
  // Make possible sending data from Tauris to printer
  digitalWrite(DtrToTauris, DTR_PRINTER_READY);

  Serial.println("SETUP COMPLETE. START LOOP"); // DEBUG PRINT
  Serial.println(""); // DEBUG PRINT
  
  SetupTimer1();
  SetupINT1();


  sei();     // Enable global interrupts by setting global interrupt enable bit in SREG
  
} // setup

void loop() {
  // Idle state timer expired. Printing is over. Add suffix data to the file.
  if(IdleState >= 10) {
     myFile.println("");
     
     myFile.print("!-!-md5="); myFile.print(SimpleChecksum); 
     myFile.print("-!-!FW="); myFile.print(FW_VER);
     myFile.print("-!-!HW="); myFile.print(HW_VER); 
     myFile.print("-!-!");
     myFile.println("");
     myFile.flush();
     
     myFile.close();
     
     #ifdef BUZZER_ENABLE
       digitalWrite(BuzzerPinOut, HIGH);
       digitalWrite(LedGreenOut, LOW);   // sets the LED on
       delay (500);
       digitalWrite(BuzzerPinOut, LOW);
       digitalWrite(LedGreenOut, HIGH);   // sets the LED off     
     #endif
     
     digitalWrite(LedGreenOut, LOW);   // sets the LED on
     delay (500);
     digitalWrite(LedGreenOut, HIGH);   // sets the LED off 
     
     //myFile = SD.open("test.txt", FILE_WRITE);
     myFile = SD.open(LOG_FILE_NAME, FILE_WRITE);

     // Reinitialize checksum variable for new data set.
     SimpleChecksum = 0;
     
     if (!myFile) 
     {
       // Serial.println("Error opening test.txt Data");
       
       // FATAL ERROR
       while(1) {
         #ifdef BUZZER_ENABLE
           digitalWrite(BuzzerPinOut, HIGH);
         #endif
         
         digitalWrite(LedGreenOut, HIGH);   // sets the LED on
         delay(300);                  // waits for a number of miliseconds
         digitalWrite(LedGreenOut, LOW);    // sets the LED off
         delay(300);                  // waits for a number of miliseconds
       } 
     }
     // Stop idle timer. It will be enabled with next received data.
     IdleState = 0;
  }

  // read into program buffer, increment index
  if (Serial.available()) {
      digitalWrite(LedGreenOut, LOW);    // sets the LED off
      b = Serial.read();
      DataCharacter = char(b);
      myFile.write(DataCharacter); 
      //myFile.flush();
      Serial.write(b);
      
      digitalWrite(LedGreenOut, HIGH);   // sets the LED on
      IdleState = 1;
  }


} // loop()
