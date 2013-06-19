void rain() {
  // Create an array of 20 raindrops
  const int count = 20;
  int pos[count];
  
  // Set each rain drop at the starting gate.
  // Signify by a position of -1
  for( int i=0; i < count; i++) {
    pos[i]=-1;
  }
  
  // Main loop. Keep looping until we've done
  // enough "frames."
  boolean done=false;
  int counter = 0;
  while(!done) {
  
    // Start by turning all LEDs off:
    for(int i=0; i<strip.numPixels(); i++) 
      strip.setPixelColor(i, 0);  

    // Loop for each rain drop
    for( int i=0; i < count; i++) {
      // If the drop is out of the starting gate,
      // turn on the LED for it.
      if( pos[i] >= 0 ) {
        strip.setPixelColor(pos[i], strip.Color(0,0,127));  
        
        // Move the drop down one row
        pos[i] -= 7;
   
        // If we've fallen off the strip, but us back at the starting gate.
        if( pos[i] < 0 )
          pos[i]=-1;    
      }
      
      // If this drop is at the starting gate, randomly
      // see if we should start it falling.
      if ( pos[i] == -1 && random(40) == 0 && counter < 380 ) {
        // Pick one of the 6 starting spots to begin falling
        pos[i] = 159-random(6);
      }

      
    }
    strip.show(); 
    delay(25);
    counter++;
    
    // See if we should stop.
    if( counter > 400 )
      done=true;
  }    
}
