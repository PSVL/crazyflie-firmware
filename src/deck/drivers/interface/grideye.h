
#ifndef _GRIDEYE_H_
#define _GRIDEYE_H_

void grideyeInit(I2C_Dev *i2cPort);
bool grideyeTest(void);
bool grideyeTestConnection();
void grideyeReadData();

#endif /* _GRIDEYE_H_ */
