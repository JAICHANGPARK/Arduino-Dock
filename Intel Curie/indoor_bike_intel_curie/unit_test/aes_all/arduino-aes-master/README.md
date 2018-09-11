# AES Cryptography for Arduino/Genuino 101 Board
=================================================
This project will be tested in Genuino 101 board

Installation
------------

- Download the files in this repository (using either clone or the download button)
- Copy the `Crypto.h` and `Crypto.cpp` into your `sketch` folder
- add `#include "Crypto.h"` in your sketch.


Usage
-----

At the moment only 128bit keys are supported

The library supports 3 kinds of operations.

1. single block encryption/decryption
-  multiple block encryption/decryption using CBC (single call)
-  multiple block encryption/decryption using CBC (multiple calls)

The single block enc/decryption are the following methods:

```c++
void AES::Encrypt;
void AES::Decrypt;
```

Usage example:
	
```c++
Serial.begin(57600);
uint8_t key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
char data[] = "0123456789012345"; //16 chars == 16 bytes
Serial.println(data);
```
