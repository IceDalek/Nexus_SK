byte ena = 9;
byte in1 = 7;
byte in2 = 8;
byte ena1 = 6;
byte in3 = 4;
byte in4 = 5;
void setup() {
    pinMode( ena, OUTPUT );
    pinMode( in1, OUTPUT );
    pinMode( in2, OUTPUT );
    pinMode( ena1, OUTPUT );
    pinMode( in3, OUTPUT );
    pinMode( in4, OUTPUT );
}
void loop() {
  //backward
    // выставляем 100% мощность на моторе А - 255 из 255
    analogWrite( ena, 100 );
    analogWrite( ena1, 100 );
    // выставляем режим мотора - вращение по часовой
    digitalWrite( in1, HIGH );
    digitalWrite( in2, LOW );
    digitalWrite( in3, HIGH );
    digitalWrite( in4, LOW );
    /*
    delay(3000); // пауза 3сек

    // выставляем мощность на мотора А - 150 из 255
    analogWrite( ena1, 100 );
    analogWrite( ena, 100 );
    // режим мотора - вращение против часовой
    digitalWrite( in1, LOW );
    digitalWrite( in2, HIGH );
        digitalWrite( in3, LOW );
    digitalWrite( in4, HIGH );
    delay(3000); // пауза 3сек
    */
}
