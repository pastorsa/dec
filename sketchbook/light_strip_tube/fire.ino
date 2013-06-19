void fire() {
  int states[160];
  for( int i=0; i < 160; i++ )
    states[i]=0;
    
  boolean done=false;
  int topend=17;
  
  int counter=0;
  while(!done) {
    
    for( int i=0; i < 160; i++ ) {
      int change = random(topend)-5;
      states[i] += change;
    }
 
     for( int i=7; i < 160; i++ ) {
      if (states[i] >states[i-7])
        states[i] = states[i-7];
    }
    for( int i=1; i < 160; i++ ) {
      if( states[i] - states[i-1] > 20 )
        states[i] = states[i-1] - 20;
    }
    for( int i=1; i < 160; i++ ) {
      if( states[i] < 0 )
        states[i] = 0;
      if( states[i] > 120 )
        states[i]=120;
    }
    for( int i=0; i < 160; i++ ) {
      if( states[i] <= 40 )
        strip.setPixelColor(i, strip.Color(states[i],0,0));
      else if ( states[i] <= 80 )
        strip.setPixelColor(i, strip.Color(40,states[i]-40,0));
      else
        strip.setPixelColor(i, strip.Color(0,0,states[i]-80));
    }
    strip.show();

    delay(50);
    counter++;
    if( counter > 200 )
      topend=10;
    if( counter > 400 )
      done=true;
  }
}
