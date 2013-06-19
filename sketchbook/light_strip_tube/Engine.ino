void engine() {
  
  int curSpot = 0;
  int curSpot2=40;
  int lit=0;
  boolean done=false;
  while(!done) {
    // Start by turning all pixels off:
    for(int i=0; i<strip.numPixels(); i++) 
      strip.setPixelColor(i, 0);
  
    int spot = curSpot;
    strip.setPixelColor(spot, strip.Color(0,0,127));
    strip.setPixelColor(spot+1, strip.Color(0,0,127));
    strip.setPixelColor(159-spot, strip.Color(0,0,127));
    strip.setPixelColor(158-spot, strip.Color(0,0,127));
  
    spot = curSpot2;
    strip.setPixelColor(spot, strip.Color(0,0,127));
    strip.setPixelColor(spot+1, strip.Color(0,0,127));
    strip.setPixelColor(159-spot, strip.Color(0,0,127));
    strip.setPixelColor(158-spot, strip.Color(0,0,127));
  
    for( int i=nLEDs/2-lit/2; i < nLEDs/2+lit/2; i++ ) {
      strip.setPixelColor(i, strip.Color(127,127,127));
    }
  
    curSpot+=3;
    if (curSpot > nLEDs/2-lit/2) {
      curSpot = 0;
      lit+=2;
    }
    curSpot2+=3;
    if (curSpot2 > nLEDs/2-lit/2) {
      curSpot2 = 0;
      lit+=2;
    }
    
    if( lit > 18 ) {
      lit=0;
      curSpot=0;
      curSpot2=40;
      for(int j=0; j < 3; j++ ){
        for(int i=0; i<160; i++) 
          strip.setPixelColor(i, strip.Color(127,127,127));
        strip.show();              // Refresh LED states
        for(int i=0; i<160; i++) 
          strip.setPixelColor(i, strip.Color(0,0,0));
        strip.show();              // Refresh LED states
        delay(5);
      }
      delay(3000);
      done=true;
    }
    strip.show();              // Refresh LED states
  }
}
