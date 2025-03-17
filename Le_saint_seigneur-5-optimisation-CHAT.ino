#include <EEPROM.h>
#include "RTClib.h"
#include <Wire.h>
#include "rgb_lcd.h"
#include <SD.h>
#include <DHT.h>
#include <ChainableLED.h>
#include <SoftwareSerial.h>
#include <SPI.h>

// Définition du nombre de LEDs dans la chaîne
#define NUM_LEDS  5
ChainableLED leds(8, 9, NUM_LEDS);

// Adresses EEPROM
#define ADR_LOG_INTERVAL   10
#define ADR_FILE_MAX_SIZE  11
#define ADR_LUMIN          12
#define ADR_LUMIN_LOW      13
#define ADR_LUMIN_HIGH     14
#define ADR_TEMP_AIR       15
#define ADR_MIN_TEMP_AIR   16
#define ADR_MAX_TEMP_AIR   17
#define ADR_HYGR           18
#define ADR_HYGR_MINT      19
#define ADR_HYGR_MAXT      20

// Valeurs par défaut
#define DEFAULT_LOG_INTERVAL  1
#define DEFAULT_FILE_MAX_SIZE 4096
#define DEFAULT_LUMIN         1
#define DEFAULT_LUMIN_LOW     255
#define DEFAULT_LUMIN_HIGH    768
#define DEFAULT_TEMP_AIR      1
#define DEFAULT_MIN_TEMP_AIR  -10
#define DEFAULT_MAX_TEMP_AIR  60
#define DEFAULT_HYGR          1
#define DEFAULT_HYGR_MINT     0
#define DEFAULT_HYGR_MAXT     50

// Broche de la carte SD
#define SD_CS_PIN 10

// Capteur DHT
#define DHTTYPE DHT11
#define DHTPIN 4
DHT dht(DHTPIN, DHTTYPE);

// Capteur de luminosité
#define LIGHT_SENSOR_PIN A0

RTC_DS1307 rtc;
rgb_lcd lcd;
SoftwareSerial SoftSerial(6, 7);
File monFichier;
char nomDuFichier[13];

// Variables d'affichage
long last_Update = 0;
int displayState = 0;

// Boutons
const int RB = 3;
const int GB = 2;

// Taille maximale de la carte SD (29 Go)
const unsigned long long SD_MAX_SIZE = 29ULL * 1024 * 1024 * 1024;

// Gestion des boutons et de l'inactivité
volatile unsigned long lastPushRB = 0;
volatile unsigned long lastPushGB = 0;
unsigned long last_Activity = 0;
unsigned long last_Mesure = 0;
const unsigned long inactivity_time = 5000;
volatile bool RD_Pushed = false;
volatile bool GRN_Pushed = false;

// Modes de fonctionnement
enum Mod {Standard, Config, Maintenance, Eco};
Mod actual_Mod = Config;

// Structure de configuration déclarée globalement (pas de malloc)
struct ConfigParameters {
    uint8_t log_Interval;
    uint16_t file_Max_Size;
    uint8_t lumin;
    uint8_t lumin_Low;
    uint8_t lumin_High;
    uint8_t temp_Air;
    int8_t min_Temp_Air;
    int8_t max_Temp_Air;
    uint8_t hygr;
    uint8_t hygr_Mint;
    uint8_t hygr_Maxt;
};
ConfigParameters config;

// Indicateur d'initialisation de la carte SD
bool sdInitialized = false;

// Fonction utilitaire pour définir la couleur de toutes les LEDs
inline void setAllLEDs(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds.setColorRGB(i, r, g, b);
  }
}

// Fonction d'erreur LED unifiée (définie via des macros pour garder les usages existants)
void errorLED(uint16_t delayRed, uint16_t delayGreen) {
  setAllLEDs(255, 0, 0);
  delay(delayRed);
  setAllLEDs(0, 255, 0);
  delay(delayGreen);
}
#define erreurColorCaptor() errorLED(500, 500)
#define erreurColorFalseData() errorLED(500, 1000)
#define erreurColorRTC() errorLED(500, 500)
#define erreurColorGPS() { setAllLEDs(255, 0, 0); delay(500); setAllLEDs(50, 50, 0); delay(500); }
#define erreurColorSDacces() { setAllLEDs(255, 0, 0); delay(500); setAllLEDs(255, 255, 255); delay(1000); }

// Affichage en mode maintenance
void DisplayMaintenance() {
    if (millis() - last_Update >= 3000) {
        last_Update = millis();
        lcd.clear();
        switch (displayState) {
            case 0:
                lcd.setCursor(0, 0);
                lcd.print(F("hum:"));
                lcd.setCursor(6, 0);
                lcd.print(collectHumidity());
                break;
            case 1:
                lcd.setCursor(0, 0);
                lcd.print(F("temp:"));
                lcd.setCursor(6, 0);
                lcd.print(collectTemperature());
                break;
            case 2:
                lcd.setCursor(0, 0);
                lcd.print(F("lum:"));
                lcd.setCursor(6, 0);
                lcd.print(collectLuminosity());
                break;
        }
        displayState = (displayState + 1) % 3;
    }
}

// Création du nom de fichier en fonction de la date et d'une révision
void createNameFile(int revision) {
    DateTime now = rtc.now();
    sprintf(nomDuFichier, "%02d%02d%02d%02d.LOG", now.year() % 100, now.month(), now.day(), revision);
}

// Vérification de la taille du fichier et gestion de la révision
void verifySizeFile() {
    if (SD.exists(nomDuFichier)) {
        monFichier = SD.open(nomDuFichier, FILE_WRITE);
        if (monFichier.size() > config.file_Max_Size) {
            monFichier.close();
            int revision = 1;
            do {
                createNameFile(revision);
                revision++;
            } while (SD.exists(nomDuFichier));
        }
        monFichier.close();
    }
}

// Écriture des données sur la carte SD
void writeSD(float temperature, float humidity, float sensorValue, const char* GPS) {
    if (!sdInitialized) {
        if (!SD.begin(SD_CS_PIN)) {
            Serial.println(F("Erreur SD!"));
            setAllLEDs(255, 0, 0);
            delay(500);
            setAllLEDs(255, 255, 255);
            delay(1000);
            return;
        }
        sdInitialized = true;
    }
    verifySizeFile();
    Serial.print(F("Ouverture du fichier: "));
    Serial.println(nomDuFichier);
    monFichier = SD.open(nomDuFichier, FILE_WRITE);
    if (monFichier) {
        monFichier.print(F("Temperature: "));
        if (isnan(temperature) || temperature <= config.min_Temp_Air || temperature >= config.max_Temp_Air) { 
            monFichier.print(F("N/A"));
            erreurColorFalseData();
        } else {
            monFichier.print(temperature);
        }
        monFichier.print(F(" °C, Humidite: "));
        if (isnan(humidity)) {
            monFichier.print(F("N/A"));
            erreurColorFalseData();
        } else {
            monFichier.print(humidity);
        }
        monFichier.print(F(" , Luminosite: "));
        if (isnan(sensorValue)) {
            monFichier.print(F("N/A"));
            erreurColorFalseData();
        } else {
            if (sensorValue <= config.lumin_Low)
                monFichier.print(F("LOW"));
            else if (sensorValue <= config.lumin_High)
                monFichier.print(F("MID"));
            else
                monFichier.print(F("HIGH"));
        }
        monFichier.print(F(" lx, GPS: "));
        monFichier.print(GPS);
        Serial.println(F("Donnees ecrites sur la carte SD!"));
        monFichier.close();
    } else {
        Serial.println(F("Erreur ouverture fichier SD."));
        erreurColorSDacces();
    }
}

// Suppression des fichiers .log dans un dossier (fonction récursive)
void supprimerFichiersLog(File dossier) {
  while (true) {
    File entry = dossier.openNextFile();
    if (!entry) break;
    if (entry.isDirectory())
      supprimerFichiersLog(entry);
    else {
      String nomFichier = entry.name();
      if (nomFichier.endsWith(".log")) {
        SD.remove(nomFichier);
        Serial.print(F("Fichier supprime : "));
        Serial.println(nomFichier);
      }
    }
    entry.close();
  }
}

void supprimerLogsCarteSD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("Initialisation SD echouee!"));
    return;
  }
  File root = SD.open("/");
  if (root) {
    supprimerFichiersLog(root);
    root.close();
    Serial.println(F("Fichiers .log supprimes."));
  } else {
    Serial.println(F("Erreur ouverture dossier racine."));
  }
}

// Vérification si la carte SD est pleine
bool isSDCardFull() {
    File root = SD.open("/");
    if (!root) {
        Serial.println(F("Erreur: ouverture SD!"));
        return false;
    }
    unsigned long long totalUsed = 0;
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        if (!entry.isDirectory())
            totalUsed += entry.size();
        entry.close();
    }
    root.close();
    return totalUsed >= SD_MAX_SIZE;
}

// Initialisation des capteurs
void initSensors() {
    dht.begin();
    SoftSerial.begin(9600);
}

// Lecture du GPS dans un buffer (remplace l'utilisation de String)
void readGPS(char* buffer, size_t len) {
    strncpy(buffer, "nan", len);
    delay(1000);
    if (SoftSerial.available()) {
        String gpsData;
        while (gpsData.indexOf("$GPGGA") == -1) {
            gpsData = SoftSerial.readStringUntil('\n');
            if (gpsData.length() <= 0) {
                strncpy(buffer, "nan", len);
                return;
            }
        }
        gpsData.toCharArray(buffer, len);
    }
}

// Fonctions de collecte de données des capteurs
float collectLuminosity() {
    pinMode(LIGHT_SENSOR_PIN, INPUT_PULLUP);
    int sensorValue = analogRead(LIGHT_SENSOR_PIN);
    return (sensorValue != 1023) ? sensorValue : NAN;
}
float collectTemperature() {
    float temperature = dht.readTemperature();
    return isnan(temperature) ? NAN : temperature;
}
float collectHumidity() {
    float humidity = dht.readHumidity();
    return isnan(humidity) ? NAN : humidity;
}

// Collecte et enregistrement des données en fonction du mode
void CollectData(Mod mod) {
    unsigned long interval;
    if (mod == Eco) {
        interval = (unsigned long)config.log_Interval * 2UL * 60UL * 1000UL;
        delay(4000);
    } else {
        interval = (unsigned long)config.log_Interval * 60UL * 1000UL;
        delay(1000);
    }
    if (last_Mesure == 0 || millis() - last_Mesure >= interval) {
        last_Mesure = millis();
        float temperature = collectTemperature();
        float humidity = collectHumidity();
        float luminosity = collectLuminosity();
        char gpsBuffer[50];
        readGPS(gpsBuffer, sizeof(gpsBuffer));
        writeSD(temperature, humidity, luminosity, gpsBuffer);
    }
}

// Vérification des capteurs (une seule lecture)
bool veriftemp() {
    float temp = dht.readTemperature();
    if (!isnan(temp)) {
        Serial.println(F("Capteur temperature OK"));
        return true;
    } else {
        erreurColorCaptor();
        return false;
    }
}
bool verifhum() {
    float hum = dht.readHumidity();
    if (!isnan(hum)) {
        Serial.println(F("Capteur humidite OK"));
        return true;
    } else {
        erreurColorCaptor();
        return false;
    }
}
bool veriflum() {
    pinMode(LIGHT_SENSOR_PIN, INPUT_PULLUP);
    int val = analogRead(LIGHT_SENSOR_PIN);
    if (val != 1023) {
        Serial.println(F("Capteur luminosite OK"));
        return true;
    } else {
        erreurColorCaptor();
        return false;
    }
}

// Changement de mode
void changeMode(Mod newMod) {
    actual_Mod = newMod;
    lcd.begin(16, 2);
    switch (actual_Mod) {
        case Standard:
            lcd.print(F("Mode Standard"));
            setAllLEDs(0, 20, 0);
            break;
        case Config:
            Serial.println(F("Mode Configuration"));
            lcd.print(F("Mode Config"));
            setAllLEDs(50, 50, 0);
            last_Activity = millis();
            break;
        case Maintenance:
            lcd.print(F("Mode Maintenance"));
            setAllLEDs(20, 10, 0);
            break;
        case Eco:
            Serial.println(F("Mode Economique"));
            lcd.print(F("Mode Economique"));
            setAllLEDs(0, 0, 20);
            break;
    }
}

// Fonctions d'interruption pour les boutons
void interruptRB() {
    if (digitalRead(RB) == LOW) {
        lastPushRB = millis();
        RD_Pushed = true;
    } else {
        RD_Pushed = false;
    }
}
void interruptGB() {
    if (digitalRead(GB) == LOW) {
        lastPushGB = millis();
        GRN_Pushed = true;
    } else {
        GRN_Pushed = false;
    }
}
// Détection d'appui long
bool longPushButton(volatile unsigned long &lastPush, volatile bool &pushButton) {
    if (pushButton && millis() - lastPush >= 1000) {
        while (digitalRead(RB) == LOW || digitalRead(GB) == LOW) {
            delay(10);
        }
        pushButton = false;
        return true;
    }
    return false;
}

// Configuration via Serial en mode Config
void configParam() {
    Serial.println(F("Saisir commande (ex: LOG_INTERVALL=...):"));
    while (!Serial.available()) {}
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("LOG_INTERVALL=")) {
        config.log_Interval = command.substring(14).toInt();
        EEPROM.put(ADR_LOG_INTERVAL, config.log_Interval);
        Serial.println(F("Log interval actualise"));
    } else if (command.startsWith("FILE_MAX_SIZE=")) {
        config.file_Max_Size = command.substring(14).toInt();
        EEPROM.put(ADR_FILE_MAX_SIZE, config.file_Max_Size);
        Serial.println(F("Taille fichier actualisee"));
    } else if (command.startsWith("LUMIN=")) {
        config.lumin = command.substring(6).toInt();
        EEPROM.put(ADR_LUMIN, config.lumin);
        Serial.println(F("Capteur luminosite actualise"));
    } else if (command.startsWith("LUMIN_LOW=")) {
        config.lumin_Low = command.substring(10).toInt();
        EEPROM.put(ADR_LUMIN_LOW, config.lumin_Low);
        Serial.println(F("Luminosite faible actualisee"));
    } else if (command.startsWith("LUMIN_HIGH=")) {
        config.lumin_High = command.substring(11).toInt();
        EEPROM.put(ADR_LUMIN_HIGH, config.lumin_High);
        Serial.println(F("Luminosite haute actualisee"));
    } else if (command.startsWith("TEMP_AIR=")) {
        config.temp_Air = command.substring(9).toInt();
        EEPROM.put(ADR_TEMP_AIR, config.temp_Air);
        Serial.println(F("Capteur temperature actualise"));
    } else if (command.startsWith("MIN_TEMP_AIR=")) {
        config.min_Temp_Air = command.substring(13).toInt();
        EEPROM.put(ADR_MIN_TEMP_AIR, config.min_Temp_Air);
        Serial.println(F("Temperature minimale actualisee"));
    } else if (command.startsWith("MAX_TEMP_AIR=")) {
        config.max_Temp_Air = command.substring(13).toInt();
        EEPROM.put(ADR_MAX_TEMP_AIR, config.max_Temp_Air);
        Serial.println(F("Temperature maximale actualisee"));
    } else if (command.startsWith("HYGR=")) {
        config.hygr = command.substring(5).toInt();
        EEPROM.put(ADR_HYGR, config.hygr);
        Serial.println(F("Capteur humidite actualise"));
    } else if (command.startsWith("HYGR_MINT=")) {
        config.hygr_Mint = command.substring(10).toInt();
        EEPROM.put(ADR_HYGR_MINT, config.hygr_Mint);
        Serial.println(F("Humidite minimale actualisee"));
    } else if (command.startsWith("HYGR_MAXT=")) {
        config.hygr_Maxt = command.substring(10).toInt();
        EEPROM.put(ADR_HYGR_MAXT, config.hygr_Maxt);
        Serial.println(F("Humidite maximale actualisee"));
    } else if (command == "RESET") {
        config.log_Interval = DEFAULT_LOG_INTERVAL;
        config.file_Max_Size = DEFAULT_FILE_MAX_SIZE;
        config.lumin = DEFAULT_LUMIN;
        config.lumin_Low = DEFAULT_LUMIN_LOW;
        config.lumin_High = DEFAULT_LUMIN_HIGH;
        config.temp_Air = DEFAULT_TEMP_AIR;
        config.min_Temp_Air = DEFAULT_MIN_TEMP_AIR;
        config.max_Temp_Air = DEFAULT_MAX_TEMP_AIR;
        config.hygr = DEFAULT_HYGR;
        config.hygr_Mint = DEFAULT_HYGR_MINT;
        config.hygr_Maxt = DEFAULT_HYGR_MAXT;
        EEPROM.put(ADR_LOG_INTERVAL, config.log_Interval);
        EEPROM.put(ADR_FILE_MAX_SIZE, config.file_Max_Size);
        EEPROM.put(ADR_LUMIN, config.lumin);
        EEPROM.put(ADR_LUMIN_LOW, config.lumin_Low);
        EEPROM.put(ADR_LUMIN_HIGH, config.lumin_High);
        EEPROM.put(ADR_TEMP_AIR, config.temp_Air);
        EEPROM.put(ADR_MIN_TEMP_AIR, config.min_Temp_Air);
        EEPROM.put(ADR_MAX_TEMP_AIR, config.max_Temp_Air);
        EEPROM.put(ADR_HYGR, config.hygr);
        EEPROM.put(ADR_HYGR_MINT, config.hygr_Mint);
        EEPROM.put(ADR_HYGR_MAXT, config.hygr_Maxt);
        Serial.println(F("Station reinitialisee par defaut"));
    } else if (command == "VERSION") {
        Serial.println(F("Version: 1.0.0, Lot: 159455138"));
    } else {
        Serial.println(F("Commande non reconnue."));
    }
}

// Suppression récursive de tous les fichiers/dossiers sur la SD
void supprimerTousLesFichiers(File dossier, const char* chemin) {
  while (true) {
    File entry = dossier.openNextFile();
    if (!entry) {
      SD.rmdir(chemin);
      Serial.print(F("Dossier supprime : "));
      Serial.println(chemin);
      break;
    }
    if (entry.isDirectory()) {
      char sousChemin[strlen(chemin) + strlen(entry.name()) + 2];
      sprintf(sousChemin, "%s/%s", chemin, entry.name());
      supprimerTousLesFichiers(entry, sousChemin);
    } else {
      SD.remove(entry.name());
      Serial.println(entry.name());
    }
    entry.close();
  }
}
void supprimerTousLesFichiersCarteSD() {
  Serial.println(F("Initialisation SD..."));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("Echec de l'initialisation SD!"));
    return;
  }
  File root = SD.open("/");
  if (!root) {
    Serial.println(F("Erreur ouverture dossier racine."));
    return;
  }
  supprimerTousLesFichiers(root, "/");
  root.close();
  Serial.println(F("Tous fichiers/dossiers supprimes."));
}

void setup() {
    Serial.begin(9600);
    dht.begin();

    // Initialisation de la config par défaut puis lecture EEPROM
    config.log_Interval  = DEFAULT_LOG_INTERVAL;
    config.file_Max_Size = DEFAULT_FILE_MAX_SIZE;
    config.lumin         = DEFAULT_LUMIN;
    config.lumin_Low     = DEFAULT_LUMIN_LOW;
    config.lumin_High    = DEFAULT_LUMIN_HIGH;
    config.temp_Air      = DEFAULT_TEMP_AIR;
    config.min_Temp_Air  = DEFAULT_MIN_TEMP_AIR;
    config.max_Temp_Air  = DEFAULT_MAX_TEMP_AIR;
    config.hygr          = DEFAULT_HYGR;
    config.hygr_Mint     = DEFAULT_HYGR_MINT;
    config.hygr_Maxt     = DEFAULT_HYGR_MAXT;
    EEPROM.get(ADR_LOG_INTERVAL, config.log_Interval);
    EEPROM.get(ADR_FILE_MAX_SIZE, config.file_Max_Size);
    EEPROM.get(ADR_LUMIN, config.lumin);
    EEPROM.get(ADR_LUMIN_LOW, config.lumin_Low);
    EEPROM.get(ADR_LUMIN_HIGH, config.lumin_High);
    EEPROM.get(ADR_TEMP_AIR, config.temp_Air);
    EEPROM.get(ADR_MIN_TEMP_AIR, config.min_Temp_Air);
    EEPROM.get(ADR_MAX_TEMP_AIR, config.max_Temp_Air);
    EEPROM.get(ADR_HYGR, config.hygr);
    EEPROM.get(ADR_HYGR_MINT, config.hygr_Mint);
    EEPROM.get(ADR_HYGR_MAXT, config.hygr_Maxt);

    initSensors();
    if (!rtc.begin()) {
        Serial.println(F("RTC non trouve!"));
        erreurColorRTC();
        while (1);
    }
    if (!rtc.isrunning()) {
        Serial.println(F("RTC arrete, reglage de l'heure."));
        erreurColorRTC();
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    supprimerTousLesFichiersCarteSD();
    createNameFile(0);
    delay(5000);

    pinMode(RB, INPUT_PULLUP);
    pinMode(GB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RB), interruptRB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(GB), interruptGB, CHANGE);
    if (digitalRead(RB) == LOW)
        changeMode(Config);
    else
        changeMode(Standard);
}

void loop() {
    switch (actual_Mod) {
        case Standard:
            setAllLEDs(0, 20, 0);
            CollectData(Standard);
            if (longPushButton(lastPushRB, RD_Pushed))
                changeMode(Maintenance);
            else if (longPushButton(lastPushGB, GRN_Pushed))
                changeMode(Eco);
            break;
        case Config:
            setAllLEDs(50, 50, 0);
            configParam();
            if (millis() - last_Activity >= inactivity_time)
                changeMode(Standard);
            break;
        case Maintenance:
            setAllLEDs(20, 10, 0);
            DisplayMaintenance();
            if (longPushButton(lastPushGB, GRN_Pushed))
                changeMode(Standard);
            else if (longPushButton(lastPushRB, RD_Pushed))
                changeMode(Eco);
            break;
        case Eco:
            setAllLEDs(0, 0, 20);
            CollectData(Eco);
            if (longPushButton(lastPushGB, GRN_Pushed))
                changeMode(Maintenance);
            else if (longPushButton(lastPushRB, RD_Pushed))
                changeMode(Standard);
            break;
    }
    if (RD_Pushed || GRN_Pushed)
        last_Activity = millis();
}
