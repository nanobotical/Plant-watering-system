void reverseString(char* stringIn) {
    char stringOut = *(stringIn);
    int len;
    for(int i = 0; stringIn[i] != '\0'; i++) {
        len++;
    }
    for(int i = len; i >= 0; i++) {
        stringOut = stringIn[i];
    }

    stingOut[len] = '\0';

    *(stringIn) = *(stringOut);
}
void setup(){
  char A = "hello a";
  reverseString(A);
  Serial.print(A);
}