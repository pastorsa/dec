void sparks() {
  const int count = 4;
  int pos[count];
  
  for( int i=0; i < count; i++) {
    pos[i]=-1;
  }
  
  boolean done=false;
  int counter = 0;
  while(!done) {
    // Start by turning all pixels off:
    for(int i=0; i<strip.numPixels(); i++) 
      strip.setPixelColor(i, 0);  

    for( int i=0; i < count; i++) {
      if( pos[i] >= 0 ) {
        strip.setPixelColor(pos[i], strip.Color(127,127,0)); 

        // Move up one row        
        pos[i] += 7;
        
        // Randomly flip the spark in a direction
        if( random(4) == 0 ) {
          int d = random(7)+1;
          // Up
          if( (d == 1 || d == 2 || d == 3) && pos[i] < (160-7) )
            pos[i] += 7;
          // Down
          if( (d == 7 || d == 6 || d == 5) && pos[i] >=7 )
            pos[i] -= 7;
          // Left
          if( (d == 3 || d == 4 || d == 5) && pos[i] < 159 )
            pos[i] += 1;
          // Right
          if( (d == 1 || d == 8 || d == 7) && pos[i] > 0 )
            pos[i] -= 1;
        }
        // End of the strip, go back to start
        if( pos[i] > 159 )
          pos[i]=-1;    
      }
      
      // Possibly start one of the sparks
      if ( pos[i] == -1 && random(40) == 0 ) {
        pos[i] = random(7);
      }

      
    }
    strip.show(); 
    delay(25);
    counter++;
    if( counter > 400 )
      done=true;
  }    
}
