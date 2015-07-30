/*
The MIT License (MIT)

Copyright (c) 2015 Lijun (http://lijun.li/)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#ifndef LL_COMMON_H__
#define LL_COMMON_H__

#define LL_UUID_BASE {0xd8, 0xc0, 0xfa, 0x1b, 0xdf, 0x1a, 0x91, 0xd4, 0x53, 0xf7, 0x2b, 0xd5, 0x79, 0x76, 0x5c, 0x03}  
// 035c7679-d52b-f753-d491-1adf1bfac0d8  <-- hashed from lijunxyz

// First service UUID can have an offset of 0x7679
/* Services and characteristics in all device types 
*/
// OTA triggering service
#define OTATS_UUID_SERVICE 0x7679
#define OTATS_UUID_CHAR 0x7680

/* Non-generic  services and characteristics 
*/
#define RCV1S_UUID_SERVICE  0x7685
#define RCV1S_UUID_MODE_CHAR  0x7686
#define RCV1S_UUID_MOVING_PARAMS_CHAR  0x7687

/* Parameters 
*/
#define OTA_TRIG_CMD_LEN   4   // length of the receiving commmand
#define OTA_TRIG_CMD  0x00112233   // OTA triggering command

#endif  // LL_COMMON_H__


