#ifndef CRYPTO_H
#define CRYPTO_H

#include "Arduino.h"

#define CRYPTO_CBC  0
#define CRYPTO_ECB  1
/*
 * All the IV and Key variables should has 16 bytes in its length
 * The padding use Zero padding
 * Output buffer harus kelipatan 16 byte 
*/
class CryptoAES
{
    public:
        CryptoAES();
        void Initialize(byte* const piv, byte* const pkey);
        void SetIV(byte* piv); // Assume 16 byte
        void SetKey(byte* pkey); // Assume 16 byte
        void Encrypt(byte* pplaintext,int len,byte* poutput,int mode);
        void Decrypt(byte* pciphertext, int len,byte* poutput,int mode);

    private:
        byte* m_piv;
        byte* m_pkey;
        void process(byte* pdatain,int len,byte* poutput,int mode,bool isencrypt);
        void encryption(const byte* plaintext, const byte* key, byte* ciphered);
        void decryption(const byte* ciphered, const byte* key,byte* plaintext);        
};

extern CryptoAES AES;
#endif
