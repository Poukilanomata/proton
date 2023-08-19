//Interpret the signal
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  
  if (ch < 100) return defaultValue;
  
  return map(ch, 1000, 2000, minLimit, maxLimit);
}


// Gets an on/off signal
bool getBoolChannel(int channelInput, int minLimit, int maxLimit, int defaultValue, int switchLevel) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  int value;

  if (ch < 100) return defaultValue;
  
  value = map(ch, 1000, 2000, minLimit, maxLimit);
  return value >= switchLevel;
}

