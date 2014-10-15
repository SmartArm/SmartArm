/******************************************************************************/
//functionName:transformData
//input:a byte type array stored the RGB565 data received from client
//output:none
//result:uncode the RGB565 format data and stored into the global array colors[][]
/******************************************************************************/
void transformData(byte[] d)
{
  int pixel=0,x=0,y=0;
  for(int i=0;i<d.length-1;i++)
  {
    x=pixel%col;
    float t=pixel/col;
    y=floor(t);
    
  
    int temp=d[i],R,G,B;
    temp=temp>>3;
    temp=temp&(0x1f);
    R=(int) temp<<3;//R
  
    temp=d[i];
    temp=temp&(0x07);
    temp=temp*8;
    G=(int) temp;
    i++;
    temp=d[i];
    temp=temp>>5;
    temp=temp&(0x07);
    G+=temp<<2;//G
  
    temp=d[i];
    temp=temp&(0x1f);
    B=(int) temp<<3;//B
  
    colors[y][x]=color(R,G,B);
    pixel++;
  }
}
void displayImage()
{
   PImage img = createImage(col, row, RGB);
  img.loadPixels();

  for(int j=0;j<row;j++)
  {
    for(int i=0;i<col;i++)
    {
      img.pixels[j*col+i]=colors[j][i];
    }
  }
  img.updatePixels();
  image(img, 0, 0);

  
}
