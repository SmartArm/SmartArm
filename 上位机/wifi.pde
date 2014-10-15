
void serverEvent(Server someServer, Client someClient) {
  println("We have a new client: " + someClient.ip());
   int dataIn=someClient.read();
  print(dataIn);
}
/******************************************************************************/
//functionName:transformData
//description:when myClient received data,this function was call,this function 
//read bytes and stored into dataRec until 's' was found,

/******************************************************************************/
void clientEvent(Client someClient) {
 if(myClient.available() > 0)
 {
   // Read until we get a linefeed
    int byteCount = myClient.readBytesUntil(stop, dataRec); 
    
    // Convert the byte array to a String
    String myString = new String(dataRec);
    // Display the string
    println(myString); 
    if(byteCount==row*col*2+1)
    {
      //do something
    }

 }

}
