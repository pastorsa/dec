void rainbowChase() {
  // Current spot along the strip to start the rainbow
  int curSpot = 0;
  boolean done=false;
  
  while (!done) {
    
    // Start by turning all pixels off:
    for(int i=0; i<strip.numPixels(); i++) 
      strip.setPixelColor(i, 0);
  
    // Temp holding spot for where we start the rainbox
    int spot=curSpot;
    
    // Start setting LEDs and moving "up" the strip
    strip.setPixelColor(spot++, strip.Color(20,0,0)); // Set new pixel 'on'
    strip.setPixelColor(spot++, strip.Color(127,0,0)); // Set new pixel 'on'
    strip.setPixelColor(spot++, strip.Color(0,127,0)); // Set new pixel 'on'
    strip.setPixelColor(spot++, strip.Color(0,0,127)); // Set new pixel 'on'
    strip.setPixelColor(spot++, strip.Color(60,0,127)); // Set new pixel 'on'
    strip.setPixelColor(spot++, strip.Color(15,0,40)); // Set new pixel 'on'
    
    strip.show();              // Refresh LED states
  
    curSpot++;
    if (curSpot > nLEDs)
      done=true;
    delay(100);
  }
}
