void rainbow() {
  // Start by turning all pixels off:
  for(int i=0; i<strip.numPixels(); i++) 
    strip.setPixelColor(i, 0);

  for(int i=0; i<strip.numPixels(); i+=7) 
    strip.setPixelColor(i, strip.Color(127,0,0));

  for(int i=1; i<strip.numPixels(); i+=7) 
    strip.setPixelColor(i, strip.Color(127,127,0));

  for(int i=2; i<strip.numPixels(); i+=7) 
    strip.setPixelColor(i, strip.Color(0,0,127));

  for(int i=3; i<strip.numPixels(); i+=7) 
    strip.setPixelColor(i, strip.Color(50,0,127));

  for(int i=4; i<strip.numPixels(); i+=7) 
    strip.setPixelColor(i, strip.Color(127,127,127));
    
  for(int i=5; i<strip.numPixels(); i+=7) 
    strip.setPixelColor(i, strip.Color(0,127,127));

  for(int i=6; i<strip.numPixels(); i+=7) 
    strip.setPixelColor(i, strip.Color(0,127,2));

  strip.show();              // Refresh LED states

  curSpot++;
  if (curSpot > nLEDs)
    curSpot = 0;

  delay(3000);
}
