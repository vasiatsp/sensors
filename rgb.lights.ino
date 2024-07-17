const
int redPin = 11;  // R petal on RGB LED
//module connected to digital pin 11 

const
int greenPin = 10;  // G petal on RGB LED
//module connected to digital pin 10 

const
int bluePin = 9;  // B petal on RGB LED
//module connected to digital pin 9 

/**************************************************************************/

void setup()

{

  pinMode(redPin, OUTPUT); // sets the redPin
//to be an output 

  pinMode(greenPin, OUTPUT); // sets thegreenPin to be an output 

  pinMode(bluePin, OUTPUT); // sets the bluePinto be an output 

}    

/***************************************************************************/

void
loop()  // run over and over again  

{    

  // Basic colors:  

  color(255, 0, 0); // turn the RGB LED red 

  delay(1000); // delay for 1 second  

  color(0,255, 0); // turn the RGB LEDgreen  

  delay(1000); // delay for 1 second  

  color(0, 0, 255); // turn the RGB LEDblue  

  delay(1000); // delay for 1 second 

  // Example blended colors:  

  color(255,0,252); // turn the RGB LEDred  

  delay(1000); // delay for 1 second  

  color(237,109,0); // turn the RGB LEDorange  

  delay(1000); // delay for 1 second  

  color(255,215,0); // turn the RGB LEDyellow  

  delay(1000); // delay for 1 second  

  color(34,139,34); // turn the RGB LEDgreen  

  delay(1000); // delay for 1 second 

  color(0,112,255); // turn the RGB LED blue  

  delay(1000); // delay for 1 second

  color(0,46,90); // turn the RGB LED  indigo 

  delay(1000); // delay for 1 second

  color(128,0,128); // turn the RGB LEDpurple  

  delay(1000); // delay for 1 second

}     

/******************************************************/

void
color (unsigned char red, unsigned char green, unsigned char blue)// the colorgenerating function  

{    

  analogWrite(redPin, red);   

  analogWrite(greenPin, green); 

  analogWrite(bluePin, blue); 

}