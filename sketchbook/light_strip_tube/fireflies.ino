void fireflies() {
  const int fireflycount = 3;
  int flyPos[fireflycount];
  
  for( int i=0; i < fireflycount; i++) {
    flyPos[i]=random(160);
  }
  
  boolean done=false;
  int counter = 0;
  while(!done) {
    // Start by turning all pixels off:
    for(int i=0; i<strip.numPixels(); i++) 
      strip.setPixelColor(i, 0);  

    for( int i=0; i < fireflycount; i++) {
      strip.setPixelColor(flyPos[i], strip.Color(127,127,127));  
      
      if( random(4) == 0 ) {
        int d = random(7)+1;
        // Up
        if( (d == 1 || d == 2 || d == 3) && flyPos[i] < (160-7) )
          flyPos[i] += 7;
        if( (d == 7 || d == 6 || d == 5) && flyPos[i] >=7 )
          flyPos[i] -= 7;
        if( (d == 3 || d == 4 || d == 5) && flyPos[i] < 159 )
          flyPos[i] += 1;
        if( (d == 1 || d == 8 || d == 7) && flyPos[i] > 0 )
          flyPos[i] -= 1;
      }
      
    }
    strip.show(); 
    delay(250);
    counter++;
    if( counter > 40 )
      done=true;
  }    
}
