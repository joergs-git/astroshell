#define SMOOTH 30 //Motor smoothness 0-no soft start, 254-max. soft start, 30-default

#define MAX_MOT1_OPEN  20197  //timeouts, to be found experimentally, depend on gear oil temperature
#define MAX_MOT1_CLOSE 21722
#define MAX_MOT1_VCC   21884
#define MAX_MOT2_OPEN  29837
#define MAX_MOT2_CLOSE 31912
#define MAX_MOT2_VCC   34231

#define IP_ADR0 192
#define IP_ADR1 168
#define IP_ADR2 1
#define IP_ADR3 177

//#define IP_ADR0 161
//#define IP_ADR1 72
//#define IP_ADR2 192
//#define IP_ADR3 163

#define VCC_RAW_MAX 580 //if VCC shows more, increase this parameter

#include <SPI.h>
#include <Ethernet.h>

#define OPEN       1 //motor direction
#define CLOSE      2 //if 0, switched off

#define motor1a    6 //All PWMs here
#define motor1b    9 //
#define motor2a    5 //
#define motor2b    3 // 3,5,6,9
#define lim1open   7 //Limit switch
#define lim1closed 2 //Limit switch
#define lim2open   1 //Limit switch
#define lim2closed 0 //Limit switch
#define SW1up     A3 //Button
#define SW1down   A2 //Button
#define SW2up     A5 //Button
#define SW2down   A4 //Button
#define SWSTOP    8  //Button
#define VCC1      A1
#define VCC2      A0
#define CLOUD     4
#define SWVIBRO   20
#define motVIBRO  20

boolean newInfo,sw1up,sw1down,sw2up,sw2down,vcc1close,vcc2close;
byte cnt,mot1dir=0,mot2dir=0,mot1speed=0,mot2speed=0,stop1reason=0,stop2reason=0,vccerr,stoppressed,closesignal,cloudsensortimer;
unsigned long long int vibrotimer;
word mot1timer,mot2timer;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(IP_ADR0,IP_ADR1,IP_ADR2,IP_ADR3);
EthernetServer server(80);

void setup(){
  pinMode(motor1a,OUTPUT);
  pinMode(motor1b,OUTPUT);
  pinMode(motor2a,OUTPUT);
  pinMode(motor2b,OUTPUT);
  pinMode(motVIBRO,OUTPUT);
  pinMode(lim1open,INPUT_PULLUP);
  pinMode(lim1closed,INPUT_PULLUP);
  pinMode(lim2open,INPUT_PULLUP);
  pinMode(lim2closed,INPUT_PULLUP);
  pinMode(SW1up,INPUT_PULLUP);
  pinMode(SW1down,INPUT_PULLUP);
  pinMode(SW2up,INPUT_PULLUP);
  pinMode(SW2down,INPUT_PULLUP);
  pinMode(SWSTOP,INPUT_PULLUP);
  pinMode(CLOUD,INPUT_PULLUP);
  pinMode(SWVIBRO,INPUT_PULLUP);
 

//  Serial.begin(115200); while (!Serial);
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.println(Ethernet.localIP());

  //Executing interruptions 60 times per second(via registers)
  cli();
  OCR2A = 255;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  sei();
}

//main cycle
void loop(){
//  Serial.println(".");

  //web server
  EthernetClient client = server.available();
  if (client){
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (newInfo && c == ' ') {
          newInfo = 0;
        }
        if (c == '$')newInfo = 1;
        if (newInfo){
          //start stop motors from webpage
          if (c == '1'){if(!mot1dir && !digitalRead(lim1open)  ){mot1dir=OPEN; mot1timer=MAX_MOT1_OPEN; }else{mot1dir=0;stop1reason=2;vcc1close=0;}}//stop1reason=2; - stop reason - web
          if (c == '2'){if(!mot1dir && !digitalRead(lim1closed)){mot1dir=CLOSE;mot1timer=MAX_MOT1_CLOSE;}else{mot1dir=0;stop1reason=2;vcc1close=0;}}
          if (c == '3'){if(!mot2dir && !digitalRead(lim2open)  ){mot2dir=OPEN; mot2timer=MAX_MOT2_OPEN; }else{mot2dir=0;stop2reason=2;vcc2close=0;}}
          if (c == '4'){if(!mot2dir && !digitalRead(lim2closed)){mot2dir=CLOSE;mot2timer=MAX_MOT2_CLOSE;}else{mot2dir=0;stop2reason=2;vcc2close=0;}}
          if (c == '5'){mot1dir=0;mot2dir=0;stop1reason=2;stop2reason=2;vcc1close=0;vcc2close=0;}
          if (c == '6'){if(vibrotimer>millis())vibrotimer=millis();else vibrotimer=millis()+10000;}
        }
        if      (c == '\n')currentLineIsBlank = true;
        else if (c != '\r')currentLineIsBlank = false;
        if (c == '\n' && currentLineIsBlank) {

          /*---------------- HTML ----------------*/
          client.println ("HTTP/1.1 200 OK");
          client.println ("Content-Type: text/html");
          client.println ("Connection: close");
          client.println ();
          client.println ("<!DOCTYPE HTML>");
          client.println ("<html>");
          client.println ("<head> ");
          client.println ("<meta http-equiv='Content-Type' content='text/html ; charset=utf-8'/> ");
          client.print ("<title>Astroshell Dome</title>");
          //styles (page design)
          client.print ("<style>");
          client.print ("td{text-align:center;font-size: 200%;}");
          client.print ("button{font-size: 200%;}");
          client.print ("button:active{color: red;}");
          client.print("</style>");
          client.println("</head>");
          //web page body 
          client.println("<body>");
          client.println("<table border=1 cellpadding=4 cellspacing=0 align=center>");
          client.println("<tr>");
          client.println("<td colspan=3 style='font-weight: bold; font-size: 400%'>");
          client.println("Astroshell Dome");          
          client.println("<tr>");
          client.println("<td width=35%>");client.print ("1");
          client.println("<td width=30%>");
          client.println("<td width=35%>");client.print ("2");
          
          client.println("<tr>");
          client.println("<td>");
          client.print("<button onmousedown=\"document.location.href='$1'\">Open</button><br><br>");
          client.println("<button onmousedown=\"document.location.href='$2'\">Close</button>");

          client.println("<td>");
          client.print("<button onmousedown=\"document.location.href='$5'\" style='font-size: 400%'>Stop</button>"); //Big button STOP
          
          client.println("<td>");
          client.print("<button onmousedown=\"document.location.href='$3'\">Open</button>");
          client.println("<br><br><button onmousedown=\"document.location.href='$4'\">Close</button>");


          client.println("<tr>");
          client.println("<td>");
          if(digitalRead(lim1closed))                        client.print("Closed<br>");
          if(digitalRead(lim1open))                          client.print("Opened<br>");
          if(!digitalRead(lim1closed) && !digitalRead(lim1open))client.print("Middle position<br>");
          if(mot1dir==OPEN)client.print("Opening<br>");
          if(mot1dir==CLOSE){
            client.print("Closing<br>");
            if(vccerr)client.print("By Capacitor<br>");
          if(vcc1close)client.print("By Cloud Sensor<br>");            
          }

            if(mot1dir==0){
              client.println("Stopped<br>");
              if(digitalRead(lim1open))  client.print("by “Opened” limit switch");
         else if(digitalRead(lim1closed))client.print("by “Closed” limit switch");
         else if(stop1reason==1)client.println("by button");
         else if(stop1reason==2)client.println("by remote command");
            }
          client.println("<td>");
         client.println("<button onmousedown=\"document.location.href='$6'\"");
         if(vibrotimer>millis())client.print(" style='color: red;'");
         client.println("></button>");  // Vibrate button removed

          client.println("<td>");
          if(digitalRead(lim2closed))                        client.print("Closed<br>");
          if(digitalRead(lim2open))                          client.print("Opened<br>");
          if(!digitalRead(lim2closed) && !digitalRead(lim2open))client.print("Middle position<br>");
          if(mot2dir==OPEN)client.print("Opening<br>");
          if(mot2dir==CLOSE){
            client.print("Closing<br>");
            if(vccerr)client.print("By Capacitor<br>");
          if(vcc2close)client.print("By Cloud Sensor<br>");            
          }
            if(mot2dir==0){
              client.println("Stopped<br>");
              if(digitalRead(lim2open))  client.println("by “Opened” limit switch");
         else if(digitalRead(lim2closed))client.println("by “Closed” limit switch");
         else if(stop2reason==1)client.print("by button");
         else if(stop2reason==2)client.print("by remote command");
            }
          //printing sensor and limit switch state
          client.println("<tr>");
          client.println("<td>");
          client.print("<br>lim1opened: ");client.println(digitalRead(lim1open));
          client.println("<br>lim1closed: ");client.println(digitalRead(lim1closed));          
          client.println("<td>");
          client.print("VCC1: ");    client.print((float)analogRead(VCC1)*24.0f/1023.0f*(1023.0f/VCC_RAW_MAX),1);client.println("V");
      // client.print("<br>VCC2: ");client.print((float)analogRead(VCC2)*24.0f/1023.0f*(1023.0f/VCC_RAW_MAX),1);client.println("V");
          client.println("<br>Stop button: ");   client.println(!digitalRead(SWSTOP));
          client.println("<br>Cloud sensor: ");   client.println(!digitalRead(CLOUD));
          client.println("<td>");
          client.print("<br>lim2opened: ");client.println(digitalRead(lim2open));
          client.println("<br>lim2closed: ");client.println(digitalRead(lim2closed));
          client.println ("</table><br>");
          client.println ("</body>");
          client.println ("</html>");
          client.print ("<script>setTimeout(\"document.location.href='http://");
          client.print (IP_ADR0);client.print (".");
          client.print (IP_ADR1);client.print (".");
          client.print (IP_ADR2);client.print (".");
          client.print (IP_ADR3);
          client.println ("/'\", 1000);</script>");
          break;
        }
      }
    }
    delay(1);
    client.stop();
  }
}

//Processing interruptions
ISR(TIMER2_COMPA_vect){
  if(cnt<100)cnt++;
  //Processing limit switches
  if(digitalRead(lim1open)||digitalRead(lim1closed)){stop1reason=0;}
  if(digitalRead(lim2open)||digitalRead(lim2closed)){stop2reason=0;}
  if(digitalRead(lim1open  ) && mot1dir==OPEN){mot1dir=0;stop1reason=0;vcc1close=0;}
  if(digitalRead(lim1closed) && mot1dir==CLOSE){mot1dir=0;stop1reason=0;vcc1close=0;}
  if(digitalRead(lim2open  ) && mot2dir==OPEN){mot2dir=0;stop2reason=0;vcc2close=0;}
  if(digitalRead(lim2closed) && mot2dir==CLOSE){mot2dir=0;stop2reason=0;vcc2close=0;}

  //Closing, if no VCC1 voltage
  if(analogRead(VCC1)<100 && vccerr<6){
    vccerr++;
    if(vccerr==6){    
      if(!digitalRead(lim1closed)){mot1dir=CLOSE;mot1timer=MAX_MOT1_VCC;}
      if(!digitalRead(lim2closed)){mot2dir=CLOSE;mot2timer=MAX_MOT2_VCC;}
    }
  }
  //If power back, stopping
  if(analogRead(VCC1)>200 && vccerr>0){
    vccerr--;  
    if(!vccerr){
      mot1dir=0;vcc1close=0;
      mot2dir=0;vcc2close=0;     
    }
  }

  //if capacitor voltage too low, stopping motors
  if(vccerr>5 && vccerr<12 && analogRead(VCC2)<92){
    vccerr++;
    if(vccerr==12){
      mot1dir=0;vcc1close=0;
      mot2dir=0;vcc2close=0;
    }
  }


//  Cloud sensor
  if(!digitalRead(CLOUD)){if(cloudsensortimer<200)cloudsensortimer++;}else if(cloudsensortimer)cloudsensortimer--;
  if(cloudsensortimer>60 && !closesignal){ //1 second delay
    if(!digitalRead(lim1closed)){mot1dir=CLOSE;mot1timer=MAX_MOT1_CLOSE;} //closing(if not already closed)
    if(!digitalRead(lim2closed)){mot2dir=CLOSE;mot2timer=MAX_MOT2_CLOSE;}
    vcc1close=1;
    vcc2close=1;
    closesignal=1; //signal received not to turn on the motors(in case if they are stopped by operator)
  }
  if(!cloudsensortimer && closesignal){ // we see there was a signal, now not anymore
 //   mot1dir=0; // if remove commment, the motors will stop when signal from sensor lost
 //   mot2dir=0;
    closesignal=0; //no clouds
  }


 //Vibrate button
  if(!digitalRead(SWVIBRO)){vibrotimer=millis()+10000;}
  if(vibrotimer<millis())digitalWrite(motVIBRO,0);else digitalWrite(motVIBRO,1) ;

  if(mot1dir){if(mot1timer)mot1timer--;else mot1dir=0;}
  if(mot2dir){if(mot2timer)mot2timer--;else mot2dir=0;}

  
   // Soft start about 0.5s
  if(!mot1dir){digitalWrite(motor1a,0);digitalWrite(motor1b,0);mot1speed=0;}else{if(mot1speed>SMOOTH)mot1speed=255;else mot1speed++;}
  if(!mot2dir){digitalWrite(motor2a,0);digitalWrite(motor2b,0);mot2speed=0;}else{if(mot2speed>SMOOTH)mot2speed=255;else mot2speed++;}



  //Signal to motor pins
  if(mot1dir==1){digitalWrite(motor1a,0       );analogWrite (motor1b,mot1speed);} //PWM to motor pins
  if(mot1dir==2){analogWrite(motor1a,mot1speed);digitalWrite(motor1b,0);}
  if(mot2dir==1){digitalWrite(motor2a,0       );analogWrite (motor2b,mot2speed);}
  if(mot2dir==2){analogWrite(motor2a,mot2speed);digitalWrite(motor2b,0);}

  //Buttons processing. Condition "cnt>6"  contact debouncing, the STOP button does not need it
  if(!digitalRead(SW1up)   && cnt>6 && !sw1up  ){sw1up=1;  cnt=0;if(mot1dir){mot1dir=0;stop1reason=1;vcc1close=0;}else if(!digitalRead(lim1open)  ){mot1dir=OPEN; mot1timer=MAX_MOT1_OPEN;}}// button 1
  if(!digitalRead(SW1down) && cnt>6 && !sw1down){sw1down=1;cnt=0;if(mot1dir){mot1dir=0;stop1reason=1;vcc1close=0;}else if(!digitalRead(lim1closed)){mot1dir=CLOSE;mot1timer=MAX_MOT1_CLOSE;}}// button 2
  if(!digitalRead(SW2up)   && cnt>6 && !sw2up  ){sw2up=1;  cnt=0;if(mot2dir){mot2dir=0;stop2reason=1;vcc2close=0;}else if(!digitalRead(lim2open)  ){mot2dir=OPEN; mot2timer=MAX_MOT2_OPEN;}}// button 3
  if(!digitalRead(SW2down) && cnt>6 && !sw2down){sw2down=1;cnt=0;if(mot2dir){mot2dir=0;stop2reason=1;vcc2close=0;}else if(!digitalRead(lim2closed)){mot2dir=CLOSE;mot2timer=MAX_MOT2_CLOSE;}}// button 4
  if(!digitalRead(SWSTOP)){mot1dir=0;mot2dir=0;stop1reason=1;stop2reason=1;vcc1close=0;vcc2close=0;}//button STOP
  if(digitalRead(SW1up)  ){sw1up=0;}
  if(digitalRead(SW1down)){sw1down=0;}
  if(digitalRead(SW2up)  ){sw2up=0;}
  if(digitalRead(SW2down)){sw2down=0;}
}
