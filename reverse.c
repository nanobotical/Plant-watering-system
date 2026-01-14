void reverseString(char* stringIn) {
    char stringOut = *(stringIn);
    int len;
    for(int i = 0; stringIn[i] != '\0'; i++) {
        len++;
    }
    for(int i = len; i >= 0; i++) {
        stringOut = stringIn[i];
    }

    stringOut[len] = '\0';

    *(stringIn) = *(stringOut);
}
#include <Arduino.h>

void setup(){
  Serial.begin(9600);
  char A[] = "hello a";
  reverseString(A);
  Serial.print(A);
}

void loop() {
  // put your main code here, to run repeatedly:
}