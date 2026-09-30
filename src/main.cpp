#include <Arduino.h>
#include "mFader.h"
#include "Button.h"

#include "soc/rtc_cntl_reg.h"
#include "esp_system.h"
#include "esp32-hal-tinyusb.h"


bool tunerMode = true;
bool mainMode = true;

TaskHandle_t serialTask;
TaskHandle_t tunerTask;
TaskHandle_t sensorTask;
TaskHandle_t randomGotoTask;


const char* FW_VERSION = "0.1.3";


mFader mainFader(13, 14, 17 /** 4 on breadboard, 3 on main */, 0, 256, -0.01, false);
mFader subFader1(-1, -1, 1, 0, 256, -0.01);
mFader subFader2(-1, -1, 2, 0, 256, -0.01);
ButtonMap buttonMap;

constexpr size_t BUTTON_1 = 0;
constexpr size_t BUTTON_2 = 1;
constexpr size_t BUTTON_3 = 2;
constexpr size_t BUTTON_4 = 3;
constexpr size_t BUTTON_BIG = 4;
constexpr size_t BUTTON_ROTARY = 5;

bool doRandomFaderMovement = false;
double randomFaderMovementPeriod = 1000;

bool sendPID = false;



String getDeviceId(){
    uint64_t chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
    String chipIdStr = String((uint16_t)(chipid >> 32), HEX);
    chipIdStr += String((uint32_t)chipid, HEX);
    return chipIdStr;
}

void enterBootMode()
{
    Serial.println("Entering ROM USB download mode...");
    Serial.flush();
    delay(50);

    usb_persist_restart(RESTART_BOOTLOADER);
    // Does not return: it prepares USB, sets the ROM download flag, and restarts.
}
void serialCommandTask(void* pvParameters){
    for(;;){
        vTaskDelay(50);

        if (Serial.available() > 0){
            
            String command = Serial.readStringUntil('\n');
            //Serial.println("checking command: " + command);
            // Extract the command and the number
            int spaceIndex = command.indexOf(' '); // Look for the first space
            if (spaceIndex != -1) {
                String cmd = command.substring(0, spaceIndex); // Extract command
                String numberStr = command.substring(spaceIndex + 1); // Extract number
                
                // Convert string to integer
                float number = numberStr.toFloat();

                // Call another function and pass the number
                if(cmd == "goto"){
                                        mainFader.GoTo(number);
                    

                }
                else if (cmd == "kp"){
                    mainFader.pid->SetTunings(double(number), mainFader.pid->GetKi(), mainFader.pid->GetKd());
                    Serial.println("kp set to: " + String((mainFader.pid->GetKp())));
                }
                else if(cmd == "ki"){
                    mainFader.pid->SetTunings(mainFader.pid->GetKp(), double(number), mainFader.pid->GetKd());
                    Serial.println("ki set to: " + String((mainFader.pid->GetKi())));
                }
                else if(cmd == "kd"){
                    mainFader.pid->SetTunings(mainFader.pid->GetKp(), mainFader.pid->GetKi(), double(number));
                    Serial.println("kd set to: " + String((mainFader.pid->GetKd())));
                }
                else if(cmd == "motorcut"){
                    mainFader.motorCutOff = double(number);
                    Serial.println("motor cut off set to: " + String((mainFader.motorCutOff)));
                }
                else if(cmd == "searchtime"){
                    mainFader.searchTime = double(number);
                    Serial.println("search time set to: " + String((mainFader.searchTime)));
                }
                else if(cmd == "searchdelay"){
                    mainFader.searchDelay = double(number);
                    Serial.println("search delay set to: " + String(number));
                }
                else if(cmd == "dampzone"){
                    mainFader.dampZone = double(number);
                    Serial.println("damp zone set to: " + String((mainFader.dampZone)));
                }
                else if(cmd == "dampreduction"){
                    mainFader.dampReduction = double(number);
                    Serial.println("damp reduction set to: " + String((mainFader.dampReduction)));
                }
                else if(cmd == "deadzone"){
                    mainFader.deadZone = double(number);
                    Serial.println("dead zone set to: " + String((mainFader.deadZone)));
                }
                else if(cmd == "margin"){
                    mainFader.margin = double(number);
                    Serial.println("margin set to: " + String((mainFader.margin)));
                }
                else if(cmd == "alpha"){
                    mainFader.alpha =  double(number);
                    mainFader.faderFilter.setFrequency(double(number));
                    Serial.println("filter set to: " + String(number));
                    Serial.println("filter current output is " + String(mainFader.faderFilter.output()));
                }
                else if(cmd == "pwmf"){
                    mainFader.PWM_FREQ = double(number);
                    ledcSetup(mainFader.PWM_CH_UP, mainFader.PWM_FREQ, mainFader.PWM_RES);
                    ledcSetup(mainFader.PWM_CH_DN, mainFader.PWM_FREQ, mainFader.PWM_RES);

                    ledcAttachPin(mainFader.upControlPin, mainFader.PWM_CH_UP);
                    ledcAttachPin(mainFader.downControlPin, mainFader.PWM_CH_DN);
                }
                else if(cmd == "rgt"){
                    randomFaderMovementPeriod = double(number);
                    Serial.println("random fader movement period set to: " + String(randomFaderMovementPeriod));
                }
            }
            else if (command == "pos"){
                Serial.println("current possition is: " + String(mainFader.currentPos) + " and raw value is: " + String(mainFader.getCurrentPosRaw()));
            }
            else if(command == "getpid"){
                Serial.println("current pid values are: kp: " + 
                    String() + mainFader.pid->GetKp() + "|| ki: " + 
                    String(mainFader.pid->GetKi()) + "|| kd: " + 
                    String(mainFader.pid->GetKd()));

            }
            else if(command == "tuner"){
                Serial.println("flipping tuner mode");
                tunerMode = !tunerMode;
                if (tunerMode == true) mainMode = false;
            }
            else if(command == "tuneron"){
                tunerMode = true;
                mainMode = false;
            }
            else if(command == "main"){
                Serial.println("flipping main mode");
                mainMode = !mainMode;
                if (mainMode == true) tunerMode = true;
            }
            else if(command == "smooth"){
                
                mainFader.smoothMode = !mainFader.smoothMode;

                if(mainFader.smoothMode) Serial.println("smoothing toggled on");
                else Serial.println("smooth toggled off");

            }
            else if(command == "setmax"){
                mainFader.setMax();
                subFader1.setMax();
                subFader2.setMax();
                Serial.println("max set");
            }
            else if(command == "setmin"){
                mainFader.setMin();
                subFader1.setMin();
                subFader2.setMin();
                Serial.println("min set");
            }
            else if(command == "midcalibrate"){
                mainFader.calibrateCurveMidpoint();
                Serial.println("midpoint set");
            }
            else if(command == "WSMM_IDENTITY"){
                String identity = "WSMM:" + String(FW_VERSION) + ":" + getDeviceId();
                Serial.println(identity);
            }
            else if(command == "WSMM_PING"){
                Serial.println("PONG");
            }
            else if(command == "bootloader"){
                enterBootMode();
            }
            else if(command == "testmotor"){
                mainFader.testMotor();
            }
            else if(command == "testmotorup"){
                digitalWrite(mainFader.upControlPin, HIGH);
                delay(200);
                digitalWrite(mainFader.upControlPin, LOW);
            }
            else if(command == "pinout"){
                Serial.println("pinout: mainfader up: " + String(mainFader.upControlPin) + " down: " + String(mainFader.downControlPin) + " pot: " + String(mainFader.potPin) + " touch: " + String(mainFader.touchPin));
                Serial.println("pinout: subfader1 up: " + String(subFader1.upControlPin) + " down: " + String(subFader1.downControlPin) + " pot: " + String(subFader1.potPin) + " touch: " + String(subFader1.touchPin));
                Serial.println("pinout: subfader2 up: " + String(subFader2.upControlPin) + " down: " + String(subFader2.downControlPin) + " pot: " + String(subFader2.potPin) + " touch: " + String(subFader2.touchPin));
            }
            else if(command == "getfaderconfig"){
                //outputs all config values on main fader, like pid, deadzone, alpha, etc..
                //formatted in code to be across multiple lines for readability
                Serial.println("mainfader config: kp: " + 
                    String(mainFader.pid->GetKp()) + " ki: " + 
                    String(mainFader.pid->GetKi()) + " kd: " + 
                    String(mainFader.pid->GetKd()) + " deadzone: " + 
                    String(mainFader.deadZone) + " alpha: " + 
                    String(mainFader.alpha) + " dampzone: " + 
                    String(mainFader.dampZone) + " dampreduction: " + 
                    String(mainFader.dampReduction) + " searchtime: " + 
                    String(mainFader.searchTime) + " searchdelay: " + 
                    String(mainFader.searchDelay) + " motorcutoff: " + 
                    String(mainFader.motorCutOff));

            }
            else if(command == "rgt"){
                doRandomFaderMovement = !doRandomFaderMovement;
                if(doRandomFaderMovement) Serial.println("random fader movement enabled");
                else Serial.println("random fader movement disabled");
            }
        }
    }
}

void tunerModeOutputTask(void* pvParameters){
    for(;;){
        vTaskDelay(40);
        if(!mainFader.isMoving) mainFader.outputValue = mainFader.smoothedPos;
        if(tunerMode && !mainMode)
        {

            String dataString = String(millis()) + "," +
                                String(mainFader.pidSetpoint) + "," +
                                String(mainFader.currentPos) + "," +
                                String(mainFader.smoothedPos) + "," +
                                String(mainFader.pidOutput) + "," +
                                String(subFader1.smoothedPos) + "," +
                                String(subFader2.smoothedPos) + "," +
                                String(buttonMap.isPressed(BUTTON_1)) + "," +
                                String(buttonMap.isPressed(BUTTON_2)) + "," +
                                String(buttonMap.isPressed(BUTTON_3)) + "," +
                                String(buttonMap.isPressed(BUTTON_4)) + "," +
                                String(buttonMap.isPressed(BUTTON_BIG)) + "," +
                                String(mainFader.isMoving);
            if(sendPID){
                dataString += "," +
                String(mainFader.pid->GetKp()) + "," +
                String(mainFader.pid->GetKi()) + "," +
                String(mainFader.pid->GetKd());
                sendPID = false;
            }


            // Send the data over Serial
            Serial.println(dataString);

        }
        else if (mainMode){
            
            double conversionValue = 1028 / 255;
            String dataString = "F:" + // command line
                                String(int(mainFader.outputValue * conversionValue)) + "|" +
                                String(int(subFader1.smoothedPos * conversionValue)) + "|" +
                                String(int(subFader2.smoothedPos * conversionValue));
                            
            Serial.println(dataString);
            
            //Serial.println(mainFader.outputValue * (1028 / 255));
        }
    }   
}

void readSensors(void* pvParameters){
    for(;;){
        vTaskDelay(10 / portTICK_PERIOD_MS);

        buttonMap.update();

        float mainMFaderPos = mainFader.updatePosition();
        mainFader.Update();
        subFader1.updatePosition();
        subFader2.updatePosition();
    }
}

void randomGoto(void* pvParameters){
    for(;;){
        vTaskDelay(randomFaderMovementPeriod / portTICK_PERIOD_MS);
        if(doRandomFaderMovement){
            float randomPos = random(0, 255);
            mainFader.GoTo(randomPos);
        }
    }
}

void buttonPressCallback(Button &button){
    Serial.println("B:" + String(button.index_));
}
void buttonHoldCallback(Button &button){
    Serial.println("B:" + String(button.index_ + 4));
}
void switchCallbackEnable(Button &button){
    Serial.println("B:" + String(button.index_) + ",1");
}
void switchCallbackRelease(Button &button){
    Serial.println("B:" + String(button.index_) + ",0");
}



void setup() {
    Serial.begin(115200);

    //core 1 reserved for display
    //core 0 for everything else, reading sensors at highest priority
    mainFader.startup("mainFader");
    subFader1.startup("subFader1");
    subFader2.startup("subFader2");


    //large switch
    

    //mechanical switches
    buttonMap.add(41).setHoldConfig(false, 0).setCallbacks(switchCallbackEnable, switchCallbackRelease, nullptr);

    buttonMap.add(4).setCallbacks(nullptr, buttonPressCallback, buttonHoldCallback).setHoldConfig(true, 1000);
    buttonMap.add(5).setCallbacks(nullptr, buttonPressCallback, buttonHoldCallback).setHoldConfig(true, 1000);
    buttonMap.add(6).setCallbacks(nullptr, buttonPressCallback, buttonHoldCallback).setHoldConfig(true, 1000); // disabled: GPIO36 is on external memory bus
    buttonMap.add(7).setCallbacks(nullptr, buttonPressCallback, buttonHoldCallback).setHoldConfig(true, 1000); // disabled: GPIO37 is on external memory bus

    


    //rotary encorder, to be removed and added to display only
    //buttonMap.add(40);

    buttonMap.begin();

    //prefs.begin("WSMM", false);

    Serial.println("setup complete, initiating tasks");


    xTaskCreatePinnedToCore(serialCommandTask, "Command checking task", 4096, NULL, 2, &serialTask, 1);
    xTaskCreatePinnedToCore(tunerModeOutputTask, "tuner output task", 2048, NULL, 1, &tunerTask, 1);
    xTaskCreatePinnedToCore(readSensors, "sensor checking task", 2048, NULL, 100, &sensorTask, 1);
    xTaskCreatePinnedToCore(randomGoto, "random fader movement task", 2048, NULL, 1, &randomGotoTask, 1);

    Serial.println("Tasks started, regular runtime proceeding");

}

void loop() {
}
