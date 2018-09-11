#include <Boards.h>
#include <Firmata.h>

#include "Crypto.h"

#define ERROR_LENGTH    0xFF

#define SUCCESS     0x00

#define AES_INIT    0x00
#define AES_SET_CBC 0x01
#define AES_SET_ECB 0x02
#define AES_ENCRYPT 0x03
#define AES_DECRYPT 0x04

#define MAX_AES_PROCESS 32

static byte aes_key[16];
static byte aes_iv[16];
static int mode;
static byte aes_result[MAX_AES_PROCESS];
void SysExAESCallback(byte command, byte byteCount, byte *arrayPointer){
    switch(command){
        case AES_INIT:
            if(byteCount != 32){
                Firmata.write(ERROR_LENGTH);
                return;
            }
            memcpy(aes_key,arrayPointer,16);
            memcpy(aes_iv,arrayPointer+16,16);
            AES.Initialize(aes_iv,aes_key);
            // response success
            Firmata.write(SUCCESS);
        break;

        case AES_SET_CBC:
        case AES_SET_ECB:
            if(byteCount != 1){
                // send error response
                Firmata.write(ERROR_LENGTH);
                return;                
            }
            mode = arrayPointer[0];
            // response success
            Firmata.write(SUCCESS);            
        break;

        case AES_ENCRYPT:
            if(byteCount > MAX_AES_PROCESS){
                // send error response
                Firmata.write(ERROR_LENGTH);
                return;                
            }
            AES.Encrypt(arrayPointer,byteCount,aes_result,mode);
            // panjang response kelipatan 16
            Firmata.sendSysex(command, ((byteCount+15)/16)*16, aes_result);            
        break;

        case AES_DECRYPT:
            if(byteCount > MAX_AES_PROCESS){
                // send error response
                Firmata.write(ERROR_LENGTH);
                return;                
            }        
            AES.Decrypt(arrayPointer,byteCount,aes_result,mode);
            // panjang response kelipatan 16
            Firmata.sendSysex(command, ((byteCount+15)/16)*16, aes_result);            
        break;
    }
}

void setup() {
    Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION,FIRMATA_MINOR_VERSION);
    Firmata.attach(START_SYSEX,SysExAESCallback);
    Firmata.begin();
}

void loop() {

    while(Firmata.available()){
        Firmata.processInput();
    }


}
