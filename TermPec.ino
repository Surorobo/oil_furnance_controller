#include <Wire.h>
#include <LCD_I2C.h>
#include <EEPROM.h>

/* correction of bridges & amplifiers */
#define CORRECTION_T_FWD   0 //v desatinách stupňa = 0°C
#define CORRECTION_T_RET -18 //v desatinách stupňa = -1.8°C

/* borders setting */
#define MIN_FAN_SPEED   4   //[%] minimálne percento PWM ventilátora (aby sa nezastavil)
#define DEF_T_MIN       400 //[°C] v desatinách stupňa
#define DEF_T_MAX       770 //[°C] v desatinách stupňa
#define DEF_HYST        20  //[°C] v desatinách stupňa = 2.0°C hyst=(i+1)/10 [0-127]=>[0.1-12.8] 
#define DEF_FWDRET_DIFF 10  //[°C] v desatinách stupňa = 1°C

/* HW setting */
#define UART_SPEED 9600
#define DISP_I2C_ADDRESS  0x20
#define DISP_LINE_LENGTH    20
#define DISP_LINE_CENTER  (DISP_LINE_LENGTH - 7)/2

/* menu setting */
#define DISP_TEMP       0
#define DISP_TMIN       1
#define DISP_TMAX       2
#define DISP_HYST       3
#define DISP_FLAMESENS  4
#define DISP_TEST_CERP  5
#define DISP_TEST_VENT  6

#define DEBOUNCE_DELAY  300    //[ms]
#define SETVAL_DELAY    100    //[ms]

#define KEYUP     1
#define KEYDOWN   2
#define KEYLEFT   3
#define KEYRIGHT  4
#define KEYSTART  5
#define KEYSTOP   6

#define CHECK_FIRE_DELAY 7200000;  //kontrola horenia a teplôt sa vykoná až po dvoch hodinách od zapnutia

const unsigned long dispLightDuration = 60000;  //[ms] interval po ktorom zhasne displej a nastaví sa východzie menu
const unsigned long measureDelay = 2000;        //[ms] interval načítania prepisovania hodnôt na displeji v kľudovej polohe

unsigned long CheckFireDelay = CHECK_FIRE_DELAY;            

const int KeyRightPin = 4;  //pin RIGHT key
const int KeyUpPin    = 5;  //pin UP key
const int KeyDownPin  = 6;  //pin DOWN key
const int KeyLeftPin  = 7;  //pin LEFT key
const int KeyStartPin = 3;  //pin start fan key
const int KeyStopPin  = 2;  //pin stop fan key

const int relayPumpPin      = 11;
const int relayFanPin       = 12;
const int flameDetectorPin  = 8;
const int PwmFanPin         = 9;
const int Fwd_PT1000_Pin    = A0;
const int Ret_PT1000_Pin    = A1;
const int FanSpeedTrimmer   = A2;
const int Led_PT1000_Read   = 13;

const char degreeC [3] = {0x01, 'C', 0x00}; //° je na displeji v setup() uložený na pozícii 0x01h
const int noSensorIndicator = 345;  //hodnota odveci, ktorá sa berie, že teplota nebola načítaná (odpojený senzor)
unsigned long lastPress = 0;        //čas posledného stlačenia tlačítka
unsigned long lastMeasure = 0;      //čas posledného merania a následného vyhodnocovania
bool memWriteRequest = false;       //bola zmena údajov, uloženie po vyskočení z menu    

bool relayPump = false; //stav relé čerpadla
bool relayFan = true;   //stav relé ventilátora
bool enableFan = true;  //ručné zapnutie alebo vypnutie
bool flameCheck = 1;    //snímanie stavu plameňa
bool flame = 1;         //stav plameňa
int setSpeedFan = 50;   //0-100% -nastavená rýchlosť ventilátora (potenciometrom a štart tlačidlom alebo cez UART)
int speedFan = 100;     //0-100% -rýchlosť ventilátora, pri 0% vypne relayFan
int tFwd;               //teplota výstupu z pece v desatinách °C
int tRet;               //teplota spiatočky v desatinách °C
int tPreviousMax = 0;   //Uložená maximálna teplota od spustenia, alebo raz za 15 min. v desatinách °C (nábeh teploty/klesanie)

//v EEPROM je stat 1 byte, druhý byte je aktuálny stav relé, tMax 4 byte, tMin 4 byte
//  dokopy 10 byte
unsigned int memAddr = 0;     //adresa začiatku bloku pamäte s hodnotami
unsigned int intSize;         //zistí sa cez sizeof(int)
unsigned int memShift;        //vypočíta sa 2 + (2 x intSize)
unsigned int eepromLength;    //zistí sa cez EEPROM.length()
unsigned int memFanByte = 1;  //adresa byte v pamäti, kde sa uloží enableFan stav (ručné zapnutie a vypnutie ventilátora)

//status: bit 0: 1 - vymazané alebo ešte nepoužité, 0 - práve aktívne
int tHyst = DEF_HYST;  //pri výpočte sa delí 10-mi, takže krok 0.1
int tMin = DEF_T_MIN; //minimálna teplota zapína čerpadlo
int tMax = DEF_T_MAX; //maximálna teplota -vypína ventilátor
int tempVal;          //používaná pri editovaní hodnôt
int menuH = 0;          //horizontálna pozícia menu
int menuV = 0;          //vertikálna pozícia menu
bool editmode = false;  //prepínač editácie hodnoty up/down šípkami
bool bckl = true;       //zapnuté podsvietenie displeja

LCD_I2C lcd(DISP_I2C_ADDRESS, DISP_LINE_LENGTH, 2);

//znak °C -menšie C
//byte degC[] = {B11000, B11000, B00110, B01001, B01000, B01000, B01001, B00110};
//znak °C -užšie C
//byte degC[] = {B11010, B11101, B00100, B00100, B00100, B00100, B00101, B00010};
//znak samotný °
byte degree[] = {B00110, B01001, B01001, B00110, B00000, B00000, B00000, B0000};
//UP
byte char_up[] = { B00100, B01110, B11111, B10101, B00100, B00000, B00000, B00000 };
//RIGHT
byte char_right[] = { B00000, B01100, B00110, B11111, B00110, B01100, B00000, B00000 };
//DOWN
byte char_down[] = { B00000, B00000, B00000, B00100, B10101, B11111, B01110, B00100 };
//LEFT
byte char_left[] = { B00000, B00110, B01100, B11111, B01100, B00110, B00000, B00000 };
//LEFT SET BRACKET
byte char_lb[] = { B01000, B10000, B10000, B10000, B10000, B10000, B10000, B01000 };
//RIGHT SET BRACKET
byte char_rb[] = { B00010, B00001, B00001, B00001, B00001, B00001, B00001, B00010 };

//======================================================================================================================
//======================================================================================================================
void setup()
{
  Serial.begin(UART_SPEED);

  pinMode(KeyUpPin, INPUT_PULLUP);
  pinMode(KeyDownPin, INPUT_PULLUP);
  pinMode(KeyLeftPin, INPUT_PULLUP);
  pinMode(KeyRightPin, INPUT_PULLUP);
  pinMode(KeyStartPin, INPUT_PULLUP);
  pinMode(KeyStopPin, INPUT_PULLUP);
  
  pinMode(relayPumpPin, OUTPUT);
  pinMode(relayFanPin, OUTPUT);
  pinMode(Led_PT1000_Read, OUTPUT);
  pinMode(PwmFanPin, OUTPUT);
  pinMode(flameDetectorPin, INPUT);

  //**************** načítanie a kontrola hodnôt z pamäte **************************
  //bit 0 - 6:  hysterézia -delí sa 10timi - krok 0.1 stupňa, 0 - 127 (0-12.7 °C)
  //bit 7:      1 vymazané, nepoužité
  //            0 aktívne
  //druhý byte je posledný stav ventilátora, nastavené teploty sú ďalšie 3x4 byty
  intSize = sizeof(int);
  memShift = 3 + (2 * intSize); 
  eepromLength = EEPROM.length();
  byte readByte = EEPROM.read(memAddr);
  while(readByte != 0xAA)
  {
    memAddr += memShift;
    if((memAddr + memShift) >= eepromLength)
    {
      memAddr = eepromLength;
      break;
    }
    readByte = EEPROM.read(memAddr);
  }
  
  //memset start 1 byte, enableFan + setSpeedFan 1 byte, flameCheck + hyst 1 byte, tMax 2 byte, tMin 2 byte
  // speedFan    enableFan
  // 1100 100x   xxxx xxx1
  //  tHyst     flameCheck //bit 1 - 7:  hysterézia -delí sa 10timi - krok 0.1 stupňa, 0 - 127 (0-12.7 °C)
  // 1111 111x   xxxx xxx1  
  if(memAddr != eepromLength) //načítanie hodnôt a prípadná korekcia
  {
    memFanByte = memAddr + 1;
    readByte  = EEPROM.read(memFanByte);
    enableFan = readByte & 0x01;
    setSpeedFan = int(readByte >> 1); //v percentách
    readByte  = EEPROM.read(memAddr + 2);
    flameCheck = readByte & 0x1;
    tHyst = int(readByte >> 1);       //už je uložená v 10-tinách stupňa
    EEPROM.get(memAddr + 3, tMin);
    EEPROM.get(memAddr + 3 + intSize, tMax);
  }
  tMin = (tMin < 100) ? 100 : tMin;     //10.0 °C
  tMin = (tMin > 500) ? 500 : tMin;     //50.0 °C
  tMax = (tMax < 510) ? 510 : tMax;     //51.0 °C
  tMax = (tMax > 950) ? 950 : tMax;     //95.0 °C
  tHyst = (tHyst > 125) ? 125 : tHyst;  //12.5 °C
  tHyst = (tHyst < 2) ? 2 : tHyst;      //0.2 °C
  if(tMax <= (tMin + tHyst))
  {
    tMin = DEF_T_MIN;
    tMax = DEF_T_MAX;
  }
  //**************** koniec načítania a kontroly hodnôt z pamäte **************************

  TCCR1A = _BV(COM1A1);// | _BV(COM1B1);  //zapnutie PWM výstupov OC1A a OC1B na digitálnych pinoch 9 a 10
  TCCR1B = _BV(WGM13) | _BV(CS10);        //nastavenie phase and frequency correct PWM and deličky 1 na časovači 1
  ICR1 = 399;                             //nastavenie frekvencie časovača 1 na 20kHz: 16MHz / (2 * (399 + 1)) = 20kHz
  pwmSet(setSpeedFan);                    //nastavenie rýchlosti ventilátora na max
  
  lcd.begin(); // If you are using more I2C devices using the Wire library use lcd.begin(false)
               // this stop the library(LCD_I2C) from calling Wire.begin()
  lcd.backlight();
  lcd.clear();
  lcd.createChar(1, degree);    //vytvorenie ° pre 2-znakový °C
  lcd.createChar(2, char_up);   //vytvorenie šípky hore
  lcd.createChar(3, char_right);//vytvorenie šípky vpravo
  lcd.createChar(4, char_down); //vytvorenie šípky dole
  lcd.createChar(5, char_left); //vytvorenie šípky vlavo
  lcd.createChar(6, char_lb);   //vytvorenie ľavej zátvorky nastavenia
  lcd.createChar(7, char_rb);   //vytvorenie pravej zátvorky nastavenia
  //lcd.createChar(8, degC);    //vytvorenie jednoznakového °C "/010"
    //max 8 znakov od 0 do 7, lebo znak 8 je mapovaný do 0, 9 do 1, 10 do 2 ... 
  dispSet();
  digitalWrite(relayPumpPin, relayPump);  //relayPump inicializovaný na false = OFF
  digitalWrite(relayFanPin, relayFan);    //relayFan inicializovaný na true = ON
  delay(3000);                            //rozbeh ventilátora naplno
  if(!enableFan)
  {                   //nastavenie uloženej hodnoty
    relayFan = false;
    digitalWrite(relayFanPin, relayFan);    //do relayFan sa nastaví uložená hodnota
    speedFan = 0;
    pwmSet(speedFan);
  }
}

//======================================================================================================================
//======================================================================================================================
void loop()
{
  int key = getKeyNumber();
  int i;
  static byte serialQuest = 0;
  
  unsigned long tm = millis(); //aktualny čas od štartu [ms]
  if(tm < lastMeasure)
  {
    lastMeasure = 0;
    lastPress = 0;
    CheckFireDelay = CHECK_FIRE_DELAY;
  }
  
  //mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
  //po ubehnutí času measureDelay alebo stlačením UP alebo pri pohybe menu úplne hore znova načíta a ak je menuV v 0, vypíše hodnoty teplôt
  if((lastMeasure == 0) || ((tm -lastMeasure) >= measureDelay) || ((key == KEYUP) && (menuV == DISP_TEMP)))
  {
    lastMeasure = tm;
    //po stlačeni tlačítka štart sa spustí ventilátor, až po dosiahnutie nastavenej maximálnej teploty, vtedy sa vypne. Zakaždým sa uloží okamžitý stav uložený v memFanByte EEPROM
    //Ďalši štart až po stlačení tlačidla štart. Tlačidlo stop vypne ventilátor (tiež na trvalo s uložením stavu do pamäte).
    //keď teplota klesne pod 30°C 
    //Čerpadlo sa spustí pri minimálnej teplote a vypne sa po poklese pod minimálnu teplotu s odpočítanou hysteréziou.

    //zakaždým DISPDELAY cyklom zapnutie napätia na senzoroch (aj lediek), zmeranie teplôt snímačmi PT1000, vypnutie napätia na senzoroch
    //a podľa nameraných hodnôt prepnutie relé čerpadla a rýchlosti ventilátora
    
    //meranie a korekcia hodnôt - kompenzácia mostíkov a zosilňovačov
    digitalWrite(Led_PT1000_Read, 1);                     //zapnutie signalizačnej led
    tFwd = readPT1000(Fwd_PT1000_Pin) + CORRECTION_T_FWD; //pridáva sa kôli korekcii teploty výstupu z pece
    tRet = readPT1000(Ret_PT1000_Pin) + CORRECTION_T_RET; //pridáva sa kôli korekcii spiatočky
    digitalWrite(Led_PT1000_Read, 0);                     //vypnutie signalizačnej led

    if(flameCheck)
      flame = !digitalRead(flameDetectorPin);  //flameDetectorPin je invertovaný, 0 = ak horí

    /* ---- VENTILÁTOR ---- */
    if(enableFan)
    {
      //kontrola maximálnej teploty
      if(relayFan)  //ak je zapnuty, pripadne vypnutie ventilatora
      {
        if(tFwd < tMax)
          updateFanSpeed();
        else
        {
          //rýchlosť nastaví na 0 a vypne ventilátor (bez iskrenia relé)
          speedFan = 0;
          pwmSet(speedFan);
          relayFan = false; 
          digitalWrite(relayFanPin, relayFan);
        }
        
        /* ---- VYPNUTIE VENTILÁTORA NA KONCI ---- */
        if(tm > CheckFireDelay)
        {
          if(tPreviousMax < tFwd) //uloženie tej najvyššej hodnoty
            tPreviousMax = tFwd;
          if((tPreviousMax > tFwd) && (tFwd < (tMin - tHyst))) //ak je zapnutý ventilátor, predtým bola vyššia teplota a výstupná teplota klesla pod min teplotu.
          {
            /* ---- AK JE TEST PLAMEŇA ---- */
            if(flameCheck)
            {
              if(!flame)
              {
                delay(2000);  //načítanie stavu plameňa ešte raz po 2 sekundách
                flame = !digitalRead(flameDetectorPin);  //flameDetectorPin je invertovaný, 0 = ak horí
                if(!flame)
                {
                  setAndWriteFan(false);   //zakáže ventilátor, rýchlosť nastaví na 0 a stav uloží do pamäti
                  tPreviousMax = 0;
                }
              }
            } /* ---- AK JE TEST PLAMEŇA END ---- */
            else
            {
              setAndWriteFan(false);   //zakáže ventilátor, rýchlosť nastaví na 0 a stav uloží do pamäti aj enableFan
              tPreviousMax = 0;
            }
          }
        }
        /* ---- VYPNUTIE VENTILÁTORA NA KONCI END---- */
      }
      else
      {
        if(tFwd < (tMax - tHyst))
        {
          relayFan = true;  //zapne ventilátor
          digitalWrite(relayFanPin, relayFan);
          delay(100);       //kôli zákmitu po zapnutí relé
          updateFanSpeed(); //nastavenie rýchlosti ventilátora, podľa aktuálnej teploty
        }
      }
    }
    /* ---- VENTILÁTOR END ---- */

    /* ---- ČERPADLO ---- */
    if(relayPump)
    {
      //ak je min teplota - hyst. - vypnutie čerpadla + kontrola spiatočky (musí byť chladnejšia ako výstupná z pece)
      if((tFwd < (tMin - tHyst)) || (tFwd <= tRet))
      {
        relayPump = false;
        digitalWrite(relayPumpPin, relayPump);
      }
    }
    else
    {
      //ak je teplota vyssia, ako nastavena minimalna vystupna teplota a vyssia ako spiatocka o diff, cerpadlo sa zapne
      if((tFwd > tMin) && (tFwd > (tRet + DEF_FWDRET_DIFF)))
      {
        relayPump = true;
        digitalWrite(relayPumpPin, relayPump);
        
      }
    }
    /* ---- ČERPADLO END ---- */
   
    if(menuV == DISP_TEMP)
      dispTemp(tFwd, tRet);
  }
  //mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm

  //******************* diaľkové ovládanie cez UART ***********************
  // get vars:        ?<0A>,
  // get setting:     #<0A>
  // write settting:  !:<0A>
  // vars:            tf=472 tr=355 pr=0 fr=1 fs=100
  // setting:         fe=1 mn=400 mx=770 hy=20 ss=70
  while(Serial.available() > 0) 
  {
    i = Serial.read();
    //test na "?<0A>" -getVars, "#<0A>" -getSetting  alebo na "!:" -setSetting
    if(serialQuest)
    {
      if((i == 0x0A) || (i == 0x0D)) //0A=<LF> alebo 0D=<CR>
      {
        if((serialQuest == 0x02) || (serialQuest == 0x04)) //bol predtým otáznik alebo mriežka
          serialQuest |= 0x01;  // = 0000 0011 alebo 0000 0101
        else
          serialQuest = 0x00;
        break;
      }
      else if(i == 0x3A) //3A=:
      {
        if(serialQuest == 0x20) //bol predtým výkričník
          serialQuest |= 0x10;  // = 0011 0000
        else
          serialQuest = 0x00;
        break;
      }
      else
        serialQuest = 0x00;
    }
    
    if(i == 0x3F) //3F=? otáznik
    {
      serialQuest = 0x02;  //0000 0010
      delay(3); //čakanie na dobeh 0A v sériovom porte
    }
    else if(i == 0x23) //3F=# mriezka
    {
      serialQuest = 0x04;  //0000 0100
      delay(3);
    }
    else if(i == 0x21) //21=! výkričník
    {
      serialQuest = 0x20;  //0010 0000
      delay(3);
    }
  }

  if(serialQuest == 0x03)  //0000 0011 = GET VARIABLES
  {
    serialQuest = 0;
    Serial.print("tf=");
    Serial.print(tFwd);
    Serial.print(" tr=");
    Serial.print(tRet);
    Serial.print(" pr=");
    Serial.print(relayPump);
    Serial.print(" fr=");
    Serial.print(relayFan);
    Serial.print(" fs=");
    Serial.println(speedFan);
    
  }
  else if(serialQuest == 0x05)  //0000 0101 = GET SETTING
  {
    serialQuest = 0;
    Serial.print("fe=");
    Serial.print(enableFan);
    Serial.print(" mn=");
    Serial.print(tMin);
    Serial.print(" mx=");
    Serial.print(tMax);
    Serial.print(" hy=");
    Serial.print(tHyst);
    Serial.print(" ss=");
    Serial.println(setSpeedFan);
  }
  else if(serialQuest == 0x30)  //0011 0000 = SET SETTING
  {
    serialQuest = 0;
    char s[64]; //bežne je max 36 ale rezerva
    char *st, *t, *p;
    int j;
    int tmpV;
    
    //fe=1 mn=39.0 mx=70.0 hy=2.0 di=4.0
    i= Serial.readBytesUntil(0x0A, s, 63);
    s[i] = '\0';
    i = 0;
    for(j = 1, st = s; j < 6 ; j++, st = NULL)
    {
      t = strtok(st, " ");
      if(t == NULL)
        break;
      if(p = strchr(t, '='))
      {
        if(p == NULL)
          break;
        *(p++) = '\0';
        if(!strcmp(t, "fe"))
        {
          bool b = atoi(p);
          if(b != enableFan)
          {
            setAndWriteFan(b);
            if(b)
              CheckFireDelay = tm + CHECK_FIRE_DELAY;
          }
        }
        else if(!strcmp(t, "mn"))
        {
          tmpV = atoi(p);
          if(tmpV != tMin)
          {
            if((tmpV >= 100) && (tmpV <= 500))  //10.0 až 50.0 °C
            {
              tMin = tmpV;
              i++;
            }
          }
        }
        else if(!strcmp(t, "mx"))
        {
          tmpV = atoi(p);
          if(tmpV != tMax)
          {
            if((tmpV >= 510) && (tmpV <= 950)) //51.0 až 95.0 °C
            {
              if(tmpV < (tMin + tHyst))
                tmpV = tMin + tHyst;
              if(tMax != tmpV)
              {
                tMax = tmpV;
                i++;
              }
            }
          }
        }
        else if(!strcmp(t, "hy"))
        {
          tmpV = atoi(p);
          if(tmpV != tHyst)
          {
            if((tmpV <= 126) && (tmpV >= 2)) //0.2 až 12.6 °C
            {
              tHyst = tmpV;
              i++;
            }
          }
        }
        else if(!strcmp(t, "ss"))
        {
          int b = atoi(p);
          if(b != setSpeedFan)
          {
            if((b >= 0) && (b < 101))
            {
              setSpeedFan = b;
              i++;
            }
          }
        }
      }
    }
    if(i) //ak i -> bola aspoň jedna zmena, uloženie hodnôt do pamäte.
    {
      memWriteRequest = true;
    }
  }

  //******************* keyboard ***********************
  if(key)
  {
    lastPress = tm;
    //netreba, keď displej svieti stále
    // if(!bckl)
    // {
    //   lcd.backlight();
    //   bckl = true;
    // }
    if(editmode && ((key == KEYUP) || (key == KEYDOWN)))
      delay(SETVAL_DELAY);
    else
      delay(DEBOUNCE_DELAY);

    if(menuV > DISP_TEMP)
    {
      if(key == KEYLEFT)
        menuH--;
      else if(key == KEYRIGHT)
        menuH++;
      if((menuH < 0) || (menuH > 3))
        menuH = 0;
    }
    
    if(menuH == 0)
    {
      if(editmode)
      {
        editmode = false;
      }
      else
      {
        if(key == KEYUP)
        {
          if(menuV > DISP_TEMP)
            menuV--;
        }
        else if(key == KEYDOWN)
        {
          if(menuV < DISP_TEST_VENT)
            menuV++;
  //        else
  //          menuV = 0;
        }
      }
    }
  }
  else
  {
    if((tm - lastPress) >= dispLightDuration)
    {
      //keď je zakomentované, displej svieti stále
      // if(bckl) 
      // {
      //   lcd.noBacklight();
      //   bckl = false;
      // }
      menuV = DISP_TEMP;
      menuH = 0;
    }

    //ulozenie hodnôt do pamäte, ak je potreba a menu je v základnej pozícii
    if(memWriteRequest)
    {
      if(menuV == DISP_TEMP)
      {
        memWriteRequest = false;
        memWriteSetting();
      }
    }
  }
  
  if(key)
  {
    if(key == KEYSTART)
    {
      setFanByTrimmer();
      if(enableFan && relayFan)
      {
        updateFanSpeed();
        //12345678901234567890
        //Rychlost ventilatora
        //        100%     
        lcd.clear();
        lcd.print("Rychlost ventilatora");
        lcd.setCursor(8, 1);
        lcd.print(setSpeedFan);
      }
      else
      {
        //12345678901234567890
        //Zapnutie ventilatora
        //        100%
        CheckFireDelay = tm + CHECK_FIRE_DELAY;
        setAndWriteFan(true);
        lcd.clear();
        lcd.print("Zapnutie ventilatora");
        lcd.setCursor(8, 1);
        lcd.print(setSpeedFan);       
      }
      delay(2000);
      key = 0;
    }
    else if(key == KEYSTOP)
    {
      if(!enableFan && !relayFan)
      {
        lcd.clear();
        lcd.print("     Ventilator");
        lcd.setCursor(0, 1);
        lcd.print("   je uz vypnuty");
      }
      else
      {
        setAndWriteFan(false);
        lcd.clear();
        lcd.print("      Vypnutie");
        lcd.setCursor(0, 1);
        lcd.print("    ventilatora");
      }
      delay(2000);
      key = 0;
    }
    else if(menuV == DISP_TMIN) //+++++++++++++++++++++++++++++++++++++++++TMIN
    {
      if(menuH == 1)
      {
        if(key == KEYRIGHT)
          tempVal = tMin;
        editmode = true;
        if(key == KEYUP)
          tempVal += 0.1;
        if(key == KEYDOWN)
          tempVal -= 0.1;
        tempVal = (tempVal < 10) ? 10 : tempVal;
        tempVal = (tempVal > 60) ? 60 : tempVal;
        if(tempVal > (tMax - tHyst))
          tempVal = tMax - tHyst;
        dispMenuSet();
      }
      else if(menuH == 2)
        dispMenuQuest();
      else if(menuH == 3)
      {
        tMin = tempVal;
        dispSavedAndSched();
        editmode = false;
        menuH = 0;
      }
      if(menuH == 0)
        dispMenuVal("Minimalna teplota", tMin);
    }
    else if(menuV == DISP_TMAX) //+++++++++++++++++++++++++++++++++++++++++TMAX
    {
      if(menuH == 1)
      {
        if(key == KEYRIGHT)
          tempVal = tMax;
        editmode = true;
        if(key == KEYUP)
          tempVal += 0.1;
        if(key == KEYDOWN)
          tempVal -= 0.1;
        tempVal = (tempVal < 50) ? 50 : tempVal;
        tempVal = (tempVal > 95) ? 95 : tempVal;
        if(tempVal < (tMin + tHyst))
          tempVal = tMin + tHyst;
        dispMenuSet();
      }
      else if(menuH == 2)
        dispMenuQuest();
      else if(menuH == 3)
      {
        tMax = tempVal;
        dispSavedAndSched();
        editmode = false;
        menuH = 0;
      }
      if(menuH == 0)
        dispMenuVal("Maximalna teplota", tMax);
    }
    else if(menuV == DISP_HYST) //+++++++++++++++++++++++++++++++++++++++++HYST
    {
      if(menuH == 1)
      {
        if(key == KEYRIGHT)
          tempVal = tHyst;
        editmode = true;
        if(key == KEYUP)
          tempVal += 0.1;
        if(key == KEYDOWN)
          tempVal -= 0.1;
        tempVal = (tempVal > 12.6) ? 12.6 : tempVal;
        tempVal = (tempVal < 0.2) ? 0.2 : tempVal;
        dispMenuSet();
      }
      else if(menuH == 2)
        dispMenuQuest();
      else if(menuH == 3)
      {
        tHyst = tempVal;
        dispSavedAndSched();
        editmode = false;
        menuH = 0;
      }
      if(menuH == 0)
        dispMenuVal("Hysterezia cerpadla", tHyst);
    }
    else if(menuV == DISP_FLAMESENS)  //+++++++++++++++++++++++++++++++++++++++++SNIMAC_PLAMENA
    {
     if(menuH == 1)
      {
        if(key == KEYRIGHT)
          tempVal = flameCheck;
        if((key == KEYUP) || (key == KEYDOWN))
          tempVal = !tempVal;
        lcd.setCursor(0, 1);
        lcd.print("\05    \02\06");
        if(tempVal)
          lcd.print("ZAP.");
        else
          lcd.print("VYP.");
        lcd.print("\07\04 ");
        lcd.print("   \03");
      }
      else if(menuH == 2)
      {
        //12345678901234567890
        //ZAPNUTY  < Ulozit? >
        lcd.setCursor(0, 1);
        if(tempVal)
          lcd.print("ZAPNUTY");
        else
          lcd.print("VYPNUTY");
        lcd.print("  \05 Ulozit? \03");
      }
      else if(menuH == 3)
      {
        flameCheck = tempVal;
        dispSavedAndSched();
        editmode = false;
        menuH = 0;
      }
      if(menuH == 0)
        dispMenuTestval("Detektor plamena", flameCheck);
    }


    else if(menuV == DISP_TEST_CERP)  //+++++++++++++++++++++++++++++++++++++++++TEST_CERP
    {
      if(menuH == 0)
        dispMenuTestval("Test zap. cerpadla", relayPump);
      else
      {
        if(menuH == 2)
        {
          menuH = 1;
          if(key == KEYRIGHT)
          {
            if(relayPump)
              relayPump = false;
            else
              relayPump = true;
            digitalWrite(relayPumpPin, relayPump);
          }
        }
        if(menuH == 1)
        {
          //lcd.setCursor(0, 0);
          //lcd.print("Test zap. cerpadla");
          lcd.setCursor(0, 1);
          if(relayPump)
            lcd.print(" ZAPNUTE -> vypnut  ");
          else
            lcd.print(" VYPNUTE -> zapnut  ");
        }
      }
    }
    else if(menuV == DISP_TEST_VENT)  //+++++++++++++++++++++++++++++++++++++++++TEST_VENT
    {
      if(menuH == 0)
        dispMenuTestval("Test ventilatora", relayFan);
      else
      {
        if(menuH == 2)
        {
          menuH = 1;
          if(key == KEYRIGHT)
          {
            if(relayFan)
            {
              relayFan = false;
              pwmSet(0);
              digitalWrite(relayFanPin, relayFan);
            }
            else
            {
              relayFan = true;
              digitalWrite(relayFanPin, relayFan);
              delay(100);
              updateFanSpeed();
            }
            
          }
        }
        if(menuH == 1)
        {
          //lcd.setCursor(0, 0);
          //lcd.print("Test ventilatora");
          lcd.setCursor(0, 1);
          if(relayFan)
            lcd.print(" ZAPNUTY -> vypnut  ");
          else
            lcd.print(" VYPNUTY -> zapnut  ");
        }
      }
    }
  }
}

//======================================================================================================================
//======================================================================================================================
int readPT1000(int ainPin)
{
  unsigned int tempsample = 0;
  int retVal;

  //priemerná hodnota z piatich meraní
  for(uint8_t i = 0; i < 5; i++)
    tempsample += analogRead(ainPin);

  if(tempsample < 5000)   //1000*5  (1024 ani 1020 nedosiahne nikdy - vysoká teplota)
  {
    //float voltFwd = tempsample / 5 * 5.0 / 1024.0;
    //po vykrateni /5*5 voltFwd = tempsample / 1024.0;
    //*tempFvar = ((((tempsample / 1024.0) - 0.455) * 100) / 1.88);
    //po zjednodušení: *tempFvar = (tempsample - 465.9) * 0.05194;
    //long sa vypočíta takto"
    long Lvar = ((tempsample * 100L) - 46590) * 5194 / 10000;
    retVal = (int)(Lvar / 100L);
    return retVal;
  }
  else
    return noSensorIndicator; //odpojený snímač PT1000

}

//======================================================================================================================
//======================================================================================================================
void pwmSet(int percento)
{
  //speedFan je v % 0-100%
  //0 = min, 400 = max
  OCR1A = percento * 4;
}

//======================================================================================================================
//======================================================================================================================
void dispMenuVal(const char* title, int tVal)
{
  lcd.clear();
  lcd.print(title);
  lcd.setCursor(0, 1);
  lcd.print("\2\4");
  lcd.setCursor(6, 1);      //"12345678901234567890"
  lcd.print(tVal / 10);  //"     58.5 °C   >    "
  lcd.print(".");
  lcd.print(abs(tVal % 10));
  lcd.print(degreeC);
  lcd.print("   \003");
}

//======================================================================================================================
//======================================================================================================================
void dispMenuTestval(const char* title, bool relayVal)
{
  lcd.clear();
  lcd.print(title);
  lcd.setCursor(0, 1);
  if(menuV == DISP_TEST_VENT)
    lcd.print("\2");
  else
    lcd.print("\2\4");
  lcd.setCursor(7, 1);
  if(relayVal)
    lcd.print("ZAPNUTY");
  else
    lcd.print("VYPNUTY");
}

//======================================================================================================================
//======================================================================================================================
void dispSet(void)
{
  //"12345678901234567890"
  //"Tmin:25.0 Tmax:85.4 "
  //"Hyst: 5.0 Fan: 100% "
  lcd.clear();
  lcd.print("tMin:");
  lcd.print(tMin / 10);
  lcd.print(".");
  lcd.print(abs(tMin % 10));
  lcd.print(" tMax:");
  lcd.print(tMax / 10);
  lcd.print(".");
  lcd.print(abs(tMax % 10));
  lcd.setCursor(0, 1);
  lcd.print("Hyst: ");
  lcd.print(tHyst / 10);
  lcd.print(".");
  lcd.print(abs(tHyst % 10));
  lcd.print(" Fan: ");
  lcd.print(setSpeedFan);
  lcd.print("%");
}

//======================================================================================================================
//======================================================================================================================
void dispTemp(int tF, int tR)
{
  //  12345678901234567890
  //  ^-184.1°C ˇ-155.3 °C 
  //  pec=100% oh=1 obeh=1 
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("\2 ");
  if(tF == noSensorIndicator)
    lcd.print(" ---- ");
  else
  {
    lcd.print(tF / 10);
    lcd.print(".");
    lcd.print(abs(tF % 10));
  }
  lcd.print(degreeC);
  lcd.print("   ");
  lcd.setCursor(10, 0);
  lcd.print("\4 ");
  if(tR == noSensorIndicator)
    lcd.print(" ---- ");
  else
  {
    lcd.print(tR / 10);
    lcd.print(".");
    lcd.print(abs(tR % 10));
  }
  lcd.print(degreeC);
  lcd.print("   ");
  lcd.setCursor(0, 1);
  lcd.print("pec:");
  lcd.print(speedFan);
  lcd.print("%   ");
  lcd.setCursor(9, 1);
  if(flameCheck)
  {
    lcd.print("oh:");
    lcd.print(flame);
  }
  else
    lcd.print("    ");
  lcd.print(" obeh:");
  lcd.print(relayPump);
}

//======================================================================================================================
//======================================================================================================================
//12345678901234567890
//<    ^(48.7)ˇ ˚C   >
void dispMenuSet(void)
{
  lcd.setCursor(0, 1);
  lcd.print("\05    \02\06");
  lcd.print(tempVal / 10);
  lcd.print(".");
  lcd.print(abs(tempVal % 10));
  lcd.print("\07\04 ");
  lcd.print(degreeC);
  lcd.print("   \03");
}

//======================================================================================================================
//======================================================================================================================
//12345678901234567890
//25.3˚C < Ulozit? >
void dispMenuQuest(void)
{
  lcd.setCursor(0, 1);
  lcd.print(tempVal / 10);
  lcd.print(".");
  lcd.print(abs(tempVal % 10));
  lcd.print(degreeC);
  lcd.print(" \05  Ulozit?  \03 ");
}
//======================================================================================================================
//======================================================================================================================
//12345678901234567890
//      ULOZENE
void dispSavedAndSched(void)
{
  memWriteRequest = true;
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(DISP_LINE_CENTER, 1);
  lcd.print("ULOZENE");
  delay(800);
}

//======================================================================================================================
//======================================================================================================================
int getKeyNumber(void)
{
  //read if PIN_KEY_x is pulled to ground (0)
  if(!digitalRead(KeyUpPin))
    return KEYUP;
  else if(!digitalRead(KeyDownPin))
    return KEYDOWN;
  else if(!digitalRead(KeyLeftPin))
    return KEYLEFT;
  else if(!digitalRead(KeyRightPin))
    return KEYRIGHT;
  else if(!digitalRead(KeyStartPin))
    return KEYSTART;
  else if(!digitalRead(KeyStopPin))
    return KEYSTOP;
  else
    return 0;
}

//======================================================================================================================
//======================================================================================================================
void memWriteSetting(void)
{
  if(memAddr >= eepromLength)
    memAddr = 0;
  else
  {
    EEPROM.write(memAddr, 0xFF);
    memAddr += memShift;
    if((memAddr + memShift) > eepromLength)
      memAddr = 0;
  }
  EEPROM.write(memAddr, 0xAA); //start byte
  memFanByte = memAddr + 1;    //nastavenie premennej memFanByte kôli ukladaniu stavu ventilátora !
  byte wrByte = (((byte)enableFan & 0x01) | ((byte)setSpeedFan << 1));
  EEPROM.write(memFanByte, wrByte);
  wrByte = (((byte)flameCheck & 0x01) | ((byte)tHyst << 1));
  EEPROM.write(memAddr + 2, wrByte);
  EEPROM.put(memAddr + 3, tMin);
  EEPROM.put(memAddr + 3 + intSize, tMax);
}

//======================================================================================================================
//======================================================================================================================
void setFanByTrimmer(void)
{
  //nastavenie rýchlosti trimrom
  //0-1023 -> 0-100  *100/1024 aby sa voslo do 2byte int vykratim 4mi => *25/256  
  //no kedze vycita najviac 1020 -> zlomok 100/1020 sa vykrati 20timi => *5/51 
  setSpeedFan = analogRead(FanSpeedTrimmer) * 5 / 51;
  speedFan = setSpeedFan;
}

//======================================================================================================================
//======================================================================================================================
void setAndWriteFan(bool b)
{
  relayFan = b;
  
  if(b)
  {
    digitalWrite(relayFanPin, 1); //zapne ventilátor a spustí pwm
    delay(100);
    updateFanSpeed();
  }
  else
  {
    speedFan = 0;
    pwmSet(speedFan);
    digitalWrite(relayFanPin, 0); //vypne pwm a potom relé ventilátora
  }
    
  if(enableFan != b) //ak je zmena nastavenia, uloží ju
  {
    enableFan = b;
    byte wrByte = (((byte)enableFan & 0x01) | ((byte)setSpeedFan << 1));
    EEPROM.write(memFanByte, wrByte);
  }
}

//======================================================================================================================
//======================================================================================================================
void updateFanSpeed(void)
{
  if((tMax - tFwd) < 30)              // < 3˚C -porovnávam desatiny stupňa !!!
    speedFan = setSpeedFan / 2;       //  20% z nastavenej hodnoty na potenciometri
  else if((tMax - tFwd) < 50)         // < 5˚C
    speedFan = setSpeedFan * 4 / 10;  //  40%
  else if((tMax - tFwd) < 70)         // < 7˚C
    speedFan = setSpeedFan * 6 / 10;  //  60%
  else if((tMax - tFwd) < 100)        // < 10˚C
    speedFan = setSpeedFan * 8 / 10;  //  80%
  else 
    speedFan = setSpeedFan;
  if(speedFan < MIN_FAN_SPEED)
      speedFan = MIN_FAN_SPEED;
  pwmSet(speedFan);                   //nastavenie rýchlosti ventilátora
}

//======================================================================================================================
//======================================================================================================================
