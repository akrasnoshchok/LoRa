#ifndef _HEX_H_
#define _HEX_H_

static char hex [] = { '0', '1', '2', '3', '4', '5', '6', '7',
                        '8', '9' ,'A', 'B', 'C', 'D', 'E', 'F' };
 
//The function that performs the conversion. Accepts a buffer with "enough space" TM 
//and populates it with a string of hexadecimal digits.Returns the length in digits
int uintToHexStr(unsigned int num, char* buff)
{
    int len=0,k=0;
    do//for every 4 bits
    {
        //get the equivalent hex digit
        buff[len] = hex[num&0xF];
        len++;
        num>>=4;
    }while(num!=0);
    //since we get the digits in the wrong order reverse the digits in the buffer
    for(;k<len/2;k++)
    {//xor swapping
        buff[k]^=buff[len-k-1];
        buff[len-k-1]^=buff[k];
        buff[k]^=buff[len-k-1];
    }
    //null terminate the buffer and return the length in digits
    buff[len]='\0';
    return len;
}

unsigned int num_hex_digits(unsigned int n) {
    int ret = 0;
    while(n) {
        n >>= 4;
        ++ret;
    }
    return ret;
}

void make_hex_string(unsigned n, char *s)
{
    const char hex_lookup[] = "0123456789abcdef";
    int len = num_hex_digits(n);

    if(n == 0) {
        *s++ = '0';
        *s++ = '0';
        *s++ = '\0';
        return;
    }

    if (len & 1) {
        *s++ = '0';
    }
    s[len] = '\0';

    for (--len; len >= 0; n >>= 4, --len) {
        s[len] = hex_lookup[n & 0xf];
    }
}

#endif