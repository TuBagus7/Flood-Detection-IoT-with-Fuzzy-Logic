#include <Arduino.h>

#define BLYNK_TEMPLATE_ID "TMPL66vGq-995"
#define BLYNK_TEMPLATE_NAME "Deteksi Banjir"
#define BLYNK_AUTH_TOKEN "Un2tXilwKuH3jKhaseuMQdMC65gl0IeU"

#include <WiFi.h>
#include <Wire.h>
#include <Blynk/BlynkApi.h>
#include <Preferences.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>


char auth[] = BLYNK_AUTH_TOKEN;
const char ssid[] = "Redmi Note 10 Pro";
const char password[] = "1sampai8";



const int raindropPin = 35; // GPIO pin connected to the raindrop sensor FC-37
const int trigPin = 33; // Defines the trigPin
const int echoPin = 32; // Defines the echoPin
long duration, distance; // Defines variables for the duration of the pulse and the distance to the object
int tby = 34;
#define VPIN_informasi V0
#define VPIN_jarak V1
#define VPIN_hujan V2
#define VPIN_tbdy V3
#define VPIN_potensi V4

// Var fuzzy
    float x1,x2,x3;
    float a_pred1,a_pred2,a_pred3,a_pred4,a_pred5,a_pred6,a_pred7,a_pred8,a_pred9,a_pred10,a_pred11,a_pred12,a_pred13,a_pred14,a_pred15,a_pred16,a_pred17,a_pred18,a_pred19,a_pred20,a_pred21,a_pred22,a_pred23,a_pred24,a_pred25,a_pred26,a_pred27;
    float z1,z2,z3,z4,z5,z6,z7,z8,z9,z10,z11,z12,z13,z14,z15,z16,z17,z18,z19,z20,z21,z22,z23,z24,z25,z26,z27;
    float zTerbobot,jaDekat,jaSedang,jaJauh,chRendah,chSedang,chTinggi,tbRendah,tbSedang,tbTinggi;

BlynkTimer timer;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void checkBlynkStatus()
{ // called every 2 seconds by SimpleTimer

  bool isconnected = Blynk.connected();
  if (isconnected == false)
  {
    Serial.println("Blynk Not Connected");
  }
  if (isconnected == true)
  {
    Serial.println(" Blynk IoT Connected ");
    // Blynk.virtualWrite(VPIN_Judul, "Monitoring Air Quality and Smoke Emissions at Urban Bus Stops");
  }

}
void connect() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

//================ Fuzzyfication Jarak air ===========================
float ja_Jauh(float x1)
{
    float a,b,miu;
    a = 250, b = 500;
    if(x1<=a)
    {
        miu=0;
    }
    else if(x1>a && x1<b)
    {
        miu=(x1-a)/(b-a);
    }
    else
    {
        miu=1;
    }
    return miu;
}

float ja_Dekat(float x1)
{
    float a,b,miu;
    a = 0, b = 250;
    if(x1<=a)
    {
        miu=1;
    }
    else if(x1>a && x1<b)
    {
        miu=(b-x1)/(b-a);
    }
    else
    {
        miu=0;
    }
    return miu;
}

float ja_Sedang(float x1)
{
    float a,b,c,miu;
    a = 125, b = 375, c = 250;
    if (x1 <= a || x1 >= c)
    {
        miu = 0;
    }
    else if (x1 > a && x1 < b)
    {
        miu = (x1 - a) / (b - a);
    }
    else if (x1 > b && x1 < c)
    {
        miu = (c - x1) / (c - b);
    }
    else if (x1 == b)
    {
        miu = 1;
    }
    return miu;
}
//================ Fuzzyfication Curah Hujan ===========================
float ch_Tinggi(float x2)
{
    float a,b,miu;
    a = 28.75, b = 57.5;
    if(x2<=a)
    {
        miu=0;
    }
    else if(x2>a && x2<b)
    {
        miu=(x2-a)/(b-a);
    }
    else
    {
        miu=1;
    }
    return miu;
}

float ch_Rendah(float x2)
{
    float a,b,miu;
    a = 0, b = 28.75;
    if(x2<=a)
    {
        miu=1;
    }
    else if(x2>a && x2<b)
    {
        miu=(b-x2)/(b-a);
    }
    else
    {
        miu=0;
    }
    return miu;
}

float ch_Sedang(float x2){
    float a,b,c,miu;
    a = 14.375, b = 43.125;
    if (x2 <= a || x2 >= c)
    {
        miu = 0;
    }
    else if (x2 > a && x2 < b)
    {
        miu = (x2 - a) / (b - a);
    }
    else if (x2 > b && x2 < c)
    {
        miu = (c - x2) / (c - b);
    }
    else if (x2 == b)
    {
        miu = 1;
    }
    return miu;
}

//================ Fuzzyfication Turbidity ===========================
float tb_Tinggi(float x3){
   float a,b,miu;
    a = 300, b = 600;
    if(x3<=a)
    {
        miu=0;
    }
    else if(x3>a && x3<b)
    {
        miu=(x3-a)/(b-a);
    }
    else
    {
        miu=1;
    }
    return miu;
}

float tb_Rendah(float x3)
{
    float a,b,miu;
    a = 0, b = 300;
    if(x3<=a)
    {
        miu=1;
    }
    else if(x3>a && x3<b)
    {
        miu=(b-x3)/(b-a);
    }
    else
    {
        miu=0;
    }
    return miu;
}

float tb_Sedang(float x3)
{
    float a,b,c,miu;
    a = 150, b = 450, c = 300;
    if (x3 <= a || x3 >= c)
    {
        miu = 0;
    }
    else if (x3 > a && x3 < b)
    {
        miu = (x3 - a) / (b - a);
    }
    else if (x3 > b && x3 < c)
    {
        miu = (c - x3) / (c - b);
    }
    else if (x3 == b)
    {
        miu = 1;
    }
    return miu;
}

//================ Fuzzyfication Persenstase banjir ===========================
float pb(float m1, float m2, float m3)
{
    float a1,a2,a_pred;
    if (m1>m2){
        a1 = m2;
    }else{
        a1 = m1;
    }

    if (m1>m3){
        a2 = m3;
    }else{
        a2 = m1;
    }

    if (a1>a2){
        a_pred = a2;
    }else{
        a_pred = a1;
    }

    return a_pred;
}


//================ Fuzzyfication prediksi ===========================
float rendah(float a_pred)
{
    float a=0,b=30,z;
    z = b-(a_pred*(b-a));

    return z;
}
float sedang(float a_pred)
{
    float a=31,b=70,z;
    z = b-(a_pred*(b-a));

    return z;
}
float tinggi(float a_pred)
{
    float a=71,b=90,z;
    z = b-(a_pred*(b-a));

    return z;
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(raindropPin, INPUT);
  timer.setInterval(2000L, checkBlynkStatus);
  WiFi.begin(ssid, password);
  connect();
  Blynk.begin(auth, ssid, password, "blynk.cloud", 80);
  lcd.begin();
  lcd.backlight();
}

void loop() 
{
        Blynk.run();
        delay(10);
        // ================= Raindrop Sensor =================
        int hj = analogRead(raindropPin);
        int rain = map(hj, 0, 4095, 150, 0);
        // Serial.print("Rain : ");
        // Serial.println(rain);

        //================== Usonic Sensor ========================
        digitalWrite(trigPin, LOW); 
        delayMicroseconds(2); 
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distance = (duration * 0.0343) / 2;

        //================== Turbidity Sensor ========================
        int val_tby = analogRead(tby);
        int tbdy = map(val_tby, 0, 3952, 150, 0);  
        // int tbdy = val_tby;
        // Serial.print("Turbidity : ");
        // Serial.println(tbdy);

        //============================== MAIN ===========================
        Blynk.virtualWrite(VPIN_jarak, distance);
        Serial.print("Nilai Jarak Air : ");
        Serial.println (distance);
        // lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("JA: ");
        lcd.setCursor(3,0);
        lcd.print(distance);
        // delay(1000);

        Blynk.virtualWrite(VPIN_hujan, rain);
        Serial.print("Nilai Curah Hujan : ");
        Serial.println (rain);
        // lcd.clear();
        lcd.setCursor(5,0);
        lcd.print("CH: ");
        lcd.setCursor(8,0);
        lcd.print(rain);
        // delay(1000);

        Blynk.virtualWrite(VPIN_tbdy, tbdy);
        Serial.print("Nilai Kekeruhan Air :");
        Serial.println(tbdy);
        // lcd.clear();
        lcd.setCursor(11,0);
        lcd.print("KA: ");
        lcd.setCursor(14,0);
        lcd.print(tbdy);
        // delay(1000);

        Serial.println("============================================");
        //======================= ========== =========================
        jaJauh = ja_Jauh(distance);
        jaDekat = ja_Dekat(distance);
        jaSedang = ja_Sedang(distance);

        chTinggi = ch_Tinggi(rain);
        chRendah = ch_Rendah(rain);
        chSedang = ch_Sedang(rain);

        tbTinggi = tb_Tinggi(tbdy);
        tbRendah = tb_Rendah(tbdy);
        tbSedang = tb_Sedang(tbdy);

        a_pred1 = pb(jaDekat,chRendah,tbRendah);
        a_pred2 = pb(jaDekat,chRendah,tbSedang);
        a_pred3 = pb(jaDekat,chRendah,tbTinggi);
        a_pred4 = pb(jaDekat,chSedang,tbRendah);
        a_pred5 = pb(jaDekat,chSedang,tbSedang);
        a_pred6 = pb(jaDekat,chSedang,tbTinggi);
        a_pred7 = pb(jaDekat,chTinggi,tbRendah);
        a_pred8 = pb(jaDekat,chTinggi,tbSedang);
        a_pred9 = pb(jaDekat,chTinggi,tbTinggi);
        a_pred10 = pb(jaSedang,chRendah,tbRendah);
        a_pred11 = pb(jaSedang,chRendah,tbSedang);
        a_pred12 = pb(jaSedang,chRendah,tbTinggi);
        a_pred13 = pb(jaSedang,chSedang,chRendah);
        a_pred14 = pb(jaSedang,chSedang,tbSedang);
        a_pred15 = pb(jaSedang,chSedang,tbTinggi);
        a_pred16 = pb(jaSedang,chTinggi,tbRendah);
        a_pred17 = pb(jaSedang,chTinggi,tbSedang);
        a_pred18 = pb(jaSedang,chTinggi,tbTinggi);
        a_pred19 = pb(jaJauh,chRendah,tbRendah);
        a_pred20 = pb(jaJauh,chRendah,tbSedang);
        a_pred21 = pb(jaJauh,chRendah,tbTinggi);
        a_pred22 = pb(jaJauh,chSedang,tbRendah);
        a_pred23 = pb(jaJauh,chSedang,tbSedang);
        a_pred24 = pb(jaJauh,chSedang,tbTinggi);
        a_pred25 = pb(jaJauh,chTinggi,tbRendah);
        a_pred26 = pb(jaJauh,chTinggi,tbSedang);
        a_pred27 = pb(jaJauh,chTinggi,tbTinggi);


        z1 = rendah(a_pred1);
        z2 = sedang(a_pred2);
        z3 = sedang(a_pred3);
        z4 = sedang(a_pred4);
        z5 = sedang(a_pred5);
        z6 = sedang(a_pred6);
        z7 = sedang(a_pred7);
        z8 = sedang(a_pred8);
        z9 = tinggi(a_pred9);
        z10 = sedang(a_pred10);
        z11 = sedang(a_pred11);
        z12 = sedang(a_pred12);
        z13 = sedang(a_pred13);
        z14 = sedang(a_pred14);
        z15 = tinggi(a_pred15);
        z16 = sedang(a_pred16);
        z17 = tinggi(a_pred17);
        z18 = tinggi(a_pred18);
        z19 = sedang(a_pred19);
        z20 = sedang(a_pred20);
        z21 = tinggi(a_pred21);
        z22 = sedang(a_pred22);
        z23 = tinggi(a_pred23);
        z24 = tinggi(a_pred24);
        z25 = tinggi(a_pred25);
        z26 = tinggi(a_pred26);
        z27 = tinggi(a_pred27);

        zTerbobot = ((a_pred1*z1)+(a_pred2*z2)+(a_pred3*z3)+(a_pred4*z4)+(a_pred5*z5)+(a_pred6*z6)+(a_pred7*z7)+(a_pred8*z8)+(a_pred9*z9)+(a_pred10*z10)+(a_pred11*z11)+(a_pred12*z12)+(a_pred13*z13)+(a_pred14*z14)+(a_pred15*z15)+(a_pred16*z16)+(a_pred17*z17)+(a_pred18*z18)+(a_pred19*z19)+(a_pred20*z20)+(a_pred21*z21)+(a_pred22*z22)+(a_pred23*z23)+(a_pred24*z24)+(a_pred25*z27)+(a_pred26*z26)+(a_pred27*z27))
        /(a_pred1+a_pred2+a_pred3+a_pred4+a_pred5+a_pred6+a_pred7+a_pred8+a_pred9+a_pred10+a_pred11+a_pred12+a_pred13+a_pred14+a_pred15+a_pred16+a_pred17+a_pred18+a_pred19+a_pred20+a_pred21+a_pred22+a_pred23+a_pred24+a_pred25+a_pred26+a_pred27);

        //  Serial.println("======================================================");
        // Serial.println("1.Tahap Fuzzycation");
        // Serial.print("a. Nilai Miu Nilai Jarak Air Dekat = ");
        // Serial.println(ja_Dekat(x1));
        // Serial.print("b. Nilai Miu Nilai Jarak Air Sedang = ");
        // Serial.println(ja_Sedang(x1));
        // Serial.print("c. Nilai Miu Nilai Jarak Air Jauh = ");
        // Serial.println(ja_Jauh(x1));
        // Serial.print("a. Nilai Miu Nilai Curah Hujan Rendah = ");
        // Serial.println(ch_Rendah(x2));
        // Serial.print("b. Nilai Miu Nilai Curah Hujan Sedang = ");
        // Serial.println(ch_Sedang(x2));
        // Serial.print("c. Nilai Miu Nilai Curah Hujan Tinggi = ");
        // Serial.println(ch_Tinggi(x2));
        // Serial.print("a. Nilai Miu Nilai Kekeruhan Air Rendah = ");
        // Serial.println(tb_Rendah(x3));
        // Serial.print("b. Nilai Miu Nilai Kekeruhan Air Sedang = ");
        // Serial.println(tb_Sedang(x3));
        // Serial.print("c. Nilai Miu Nilai Kekeruhan Air Tinggi = ");
        // Serial.println(tb_Tinggi(x3));
        // Serial.println("======================================================");

        // Serial.println("2) Tahap Inferensi");
        // Serial.println("[R1] IF Nilai Jat AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air RENDAH THEN Rendah");
        // Serial.print("a-predikat1 = ");
        // Serial.println(a_pred1);
        // Serial.print("Z1 = ");
        // Serial.println(z1);

        // Serial.println("[R2] IF Nilai Jarak Air Dekat AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air SEDANG THEN Sedang");
        // Serial.print("a-predikat2 = ");
        // Serial.println(a_pred2);
        // Serial.print("Z2 = ");
        // Serial.println(z2); 

        // Serial.println("[R3] IF Nilai Jarak Air Dekat AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air TINGGI THEN SEDANG");
        // Serial.print("a-predikat3 = ");
        // Serial.println(a_pred3);
        // Serial.print("Z3 = ");
        // Serial.println(z3);     

        // Serial.println("[R4] IF Nilai Jarak Air Dekat AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air RENDAH THEN SEDANG");       
        // Serial.print("a-predikat4 = ");
        // Serial.println(a_pred4);
        // Serial.print("Z4 = ");
        // Serial.println(z4); 

        // Serial.println("[R5] IF Nilai Jarak Air Dekat AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air SEDANG THEN SEDANG");
        // Serial.print("a-predikat5 = ");
        // Serial.println(a_pred5);
        // Serial.print("Z5 = ");
        // Serial.println(z5);

        // Serial.println("[R6] IF Nilai Jarak Air Dekat AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air TINGGI THEN SEDANG");
        // Serial.print("a-predikat6 = ");
        // Serial.println(a_pred6);
        // Serial.print("Z6 = ");
        // Serial.println(z6);

        // Serial.println("[R7] IF Nilai Jarak Air Dekat AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air RENDAH THEN SEDANG");
        // Serial.print("a-predikat7 = ");
        // Serial.println(a_pred7);
        // Serial.print("Z7 = ");
        // Serial.println(z7);

        // Serial.println("[R8] IF Nilai Jarak Air Dekat AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air SEDANG THEN SEDANG");
        // Serial.print("a-predikat8 = ");
        // Serial.println(a_pred8);
        // Serial.print("Z8 = ");
        // Serial.println(z8);

        // Serial.println("[R9] IF Nilai Jarak Air Dekat AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air TINGGI THEN TINGGI");
        // Serial.print("a-predikat9 = ");
        // Serial.println(a_pred9);
        // Serial.print("Z9 = ");
        // Serial.println(z9);

        // Serial.println("[R10] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air RENDAH THEN SEDANG");
        // Serial.print("a-predikat10 = ");
        // Serial.println(a_pred10);
        // Serial.print("Z10 = ");
        // Serial.println(z10);    

        // Serial.println("[R11] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air SEDANG THEN Sedang");
        // Serial.print("a-predikat11 = ");
        // Serial.println(a_pred11);
        // Serial.print("Z11 = ");
        // Serial.println(z11);    

        // Serial.println("[R12] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air TINGGI THEN Sedang");
        // Serial.print("a-predikat12 = ");
        // Serial.println(a_pred12);
        // Serial.print("Z12 = ");
        // Serial.println(z12);

        // Serial.println("[R13] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air RENDAH THEN Sedang");
        // Serial.print("a-predikat13 = ");
        // Serial.println(a_pred13);
        // Serial.print("Z13 = ");
        // Serial.println(z13);

        // Serial.println("[R14] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air SEDANG THEN Sedang");
        // Serial.print("a-predikat14 = ");
        // Serial.println(a_pred14);
        // Serial.print("Z14 = ");
        // Serial.println(z14);

        // Serial.println("[R15] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air TINGGI THEN Tinggi");
        // Serial.print("a-predikat15 = ");
        // Serial.println(a_pred15);
        // Serial.print("Z15 = ");
        // Serial.println(z15);

        // Serial.println("[R16] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air RENDAH THEN Sedang");
        // Serial.print("a-predikat16 = ");
        // Serial.println(a_pred16);
        // Serial.print("Z16 = ");
        // Serial.println(z16);

        // Serial.println("[R17] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air SEDANG THEN Tinggi");
        // Serial.print("a-predikat17 = ");
        // Serial.println(a_pred17);
        // Serial.print("Z17 = ");
        // Serial.println(z17);

        // Serial.println("[R18] IF Nilai Jarak Air Sedang AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air TINGGI THEN Tinggi");
        // Serial.print("a-predikat18 = ");
        // Serial.println(a_pred18);
        // Serial.print("Z18 = ");
        // Serial.println(z18);

        // Serial.println("[R19] IF Nilai Jarak Air Jauh AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air RENDAH THEN Sedang");
        // Serial.print("a-predikat19 = ");
        // Serial.println(a_pred19);
        // Serial.print("Z19 = ");
        // Serial.println(z19);

        // Serial.println("[R20] IF Nilai Jarak Air Jauh AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air SEDANG THEN Sedang");
        // Serial.print("a-predikat20 = ");
        // Serial.println(a_pred20);
        // Serial.print("Z20 = ");
        // Serial.println(z20);

        // Serial.println("[R21] IF Nilai Jarak Air JAUH AND Nilai Curah Hujan RENDAH AND Nilai Kekeruhan Air TINGGI THEN Tinggi");
        // Serial.print("a-predikat21 = ");
        // Serial.println(a_pred21);
        // Serial.print("Z21 = ");
        // Serial.println(z21);

        // Serial.println("[R22] IF Nilai Jarak Air JAUH AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air RENDAH THEN Sedang");
        // Serial.print("a-predikat22 = ");
        // Serial.println(a_pred22);
        // Serial.print("Z22 = ");
        // Serial.println(z22);

        // Serial.println("[R23] IF Nilai Jarak Air JAUH AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air SEDANG THEN Tinggi");
        // Serial.print("a-predikat23 = ");
        // Serial.println(a_pred23);
        // Serial.print("Z23 = ");
        // Serial.println(z23);

        // Serial.println("[R24] IF Nilai Jarak Air JAUH AND Nilai Curah Hujan SEDANG AND Nilai Kekeruhan Air TINGGI THEN Tinggi");
        // Serial.print("a-predikat24 = ");
        // Serial.println(a_pred24);
        // Serial.print("Z24 = ");
        // Serial.println(z24);

        // Serial.println("[R25] IF Nilai Jarak Air JAUH AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air RENDAH THEN Tinggi");
        // Serial.print("a-predikat25 = ");
        // Serial.println(a_pred25);
        // Serial.print("Z25 = ");
        // Serial.println(z25);

        // Serial.println("[R26] IF Nilai Jarak Air JAUH AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air SEDANG THEN Tinggi");
        // Serial.print("a-predikat26 = ");
        // Serial.println(a_pred26);
        // Serial.print("Z26 = ");
        // Serial.println(z26);

        // Serial.println("[R27] IF Nilai Jarak Air JAUH AND Nilai Curah Hujan TINGGI AND Nilai Kekeruhan Air TINGGI THEN Tinggi");
        // Serial.print("a-predikat27 = ");
        // Serial.println(a_pred27);
        // Serial.print("Z27 = ");
        // Serial.println(z27);

        Serial.println("======================================================");
        Serial.println ("3) Tahap Defuzzycation");
        Serial.print("Kemungkinan  = ");
        Serial.print(zTerbobot );
        Serial.print( " Potensial Banjir : ");
        // lcd.clear();
        lcd.setCursor(6,1);
        lcd.print(zTerbobot);
        lcd.setCursor(10, 1);
        lcd.print("%");
        Blynk.virtualWrite(VPIN_potensi, zTerbobot);
        // delay(1000);
        // lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("P: ");

        if (zTerbobot >= 0 && zTerbobot <=30){
            Serial.println( "Rendah" );
            Blynk.virtualWrite(VPIN_informasi, "Rendah");
            lcd.setCursor(2, 1);
            lcd.print("Rdh");
            // delay(1000);
        }else if (zTerbobot >= 31 && zTerbobot <=70){
            Serial.println("Sedang" );
            Blynk.virtualWrite(VPIN_informasi, "Sedang");
            lcd.setCursor(2, 1);
            lcd.print("Sdg");
            // delay(1000);
        }else if (zTerbobot >=71 && zTerbobot <=90){
            Serial.println("Tinggi");
            Blynk.virtualWrite(VPIN_informasi, "Tinggi");
            lcd.setCursor(2, 1);
            lcd.print("Tgi");
            // delay(1000);
        }else {
            Serial.println("Tidak Terbobot");
        }
        
}











