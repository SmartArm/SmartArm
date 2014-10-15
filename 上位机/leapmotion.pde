import java.util.Formatter;
void leapOnInit(){
     println("Leap Motion Init");
}
void leapOnConnect(){
     println("Leap Motion Connect");
}
void leapOnFrame(){
  
    //println("Leap Motion onFrame at ");
}
void leapOnDisconnect(){
     println("Leap Motion Disconnect");
}
void leapOnExit(){
     println("Leap Motion Exit");
}

void getLeapData()
{
  Hand hand;
  ArrayList<Hand> hands=leap.getHands();
  if(hands.size()>0)
  {
  hand=hands.get(0);//first hand
  //hand.draw();
  
  PVector hand_position    = hand.getPalmPosition();
  float   hand_roll        = hand.getRoll();
  float   hand_pitch       = hand.getPitch();
  float   hand_yaw         = hand.getYaw();
  int x,y,z,roll,pitch,yaw;
  x=(int) hand_position.x;
  y=(int) hand_position.y;
  z=(int) hand_position.z;
  pitch=(int) hand_roll; roll=(int) hand_pitch; yaw =(int) hand_yaw ;
   //String mes = String.format("%d %d %d %d %d %d ",x,y,z,roll,pitch,yaw); 
   //println(mes);
   ArrayList<Finger> fingers=hand.getFingers();
   int numOfFingers=fingers.size();
   println("the number of fingers is"+fingers.size());
   
  
  /* for(Finger finger : hand.getFingers()){

            // Basics
            finger.draw();
          }*/
   
   String mes = String.format("%d %d %d %d %d %d %d",x,y,z,roll,pitch,yaw,numOfFingers); 
   println(mes);
   controlClient.write(mes);
  }
}
