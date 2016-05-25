void setup() {
    Serial.begin(9600);
}



String str;

void loop() {
    if (Serial.available())
    {
        String str = Serial.readString();
    }

    
}
