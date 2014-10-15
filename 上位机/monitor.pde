import de.voidplus.leapmotion.*;
import development.*;
LeapMotion leap;
import processing.net.*;

Server myServer;
Client myClient;
Client controlClient;
final int row=160,col=120;
color colors[][]=new color[row][col];
byte dataRec[]=new byte[row*col*2+1];//+1 for the stop sign
byte stop='s';

void setup()
{
  size(800,600,P3D);
  leap = new LeapMotion(this).withGestures("circle");
 // myServer = new Server(this, 5204); //set up a server to received the camera data
  myClient = new Client(this, "192.168.1.118", 5204);//receive camera data
  controlClient=new Client(this, "192.168.1.118", 5205);//sent data to control the arm
  /* for(int j=0;j<row;j++)
 {
   for(int i=0;i<col;i++)
   {
    //colors[j][i]=color(0,0,0);
    print(hex(colors[j][i])+" ");
   }//initialize the color table
 }*/
 for(int x=0;x<dataRec.length-1;x++){
   dataRec[x]=(byte) 0x62;
   dataRec[x+1]=(byte) 0xdf;
   x++;
 }//for test
 
    for(int j=0;j<row;j++)
 {
   for(int i=0;i<col;i++)
   {
    colors[j][i]=color(0,0,0);
   
   }//initialize the color table
 }
  displayImage();
}
void draw()
{
 
 
 getLeapData();
 transformData(dataRec); 
 displayImage();
 
 
}
