/** @filename: esp1/hardware/lidar.cpp
 *  @description: Contract for the LiDAR sensor driver and
 *               feature extraction pipeline on ESP_1.
 * 
 *  @note: the raw sensor data must be simplified and refined
 *       before it can be used by the sophisticated mathematical
 *       models like the EKF SLAM or the Bayesian Occupancy Grid.
 * 
 *  @job: Run the raw data through Feature Extraction to create a
 *        simplified version of the scan.
 */

#include "esp1/hardware/lidar.h"
#include "Arduino.h"

rp_descriptor_t resp_descriptor[] = {{0xA5,0x5A,0x54,0x00,0x00,0x40,0x82},//Legacy Version
								{0xA5,0x5A,0x84,0x00,0x00,0x40,0x84},//Extended Version
								{0xA5,0x5A,0x54,0x00,0x00,0x40,0x85},//Dense Version
								{0xA5,0x5A,0x05,0x00,0x00,0x40,0x81}};//StartScan

rq_message_t req_message[] = {{0xA5,0x25}, //Stop
                                {0xA5,0x40}, //Reset
                                {0xA5,0x20}, //Scan
                                {0xA5,0x82}}; //Express scan

Lidar::Lidar(HardwareSerial *_mySerial)
{
	serial=_mySerial;
}

bool Lidar::start()
{
    // Reset the device
	serial->write(req_message[rq_reset],2);
    
    // Clear serial buffer
    while(serial->available()) //read as long the hardware buffer is not empty
    {
        serial->read();
    }

    // Start scan
    serial->write((uint8_t*)&req_message[rq_scan],2); //standard scan request

	rp_descriptor_t descr;

	if(!checkForTimeout(5000,7)) //wait for response
	{
		serial->readBytes((uint8_t*)&descr,7);
        return compareDescriptor(descr,resp_descriptor[startScan]);
	}
	return false;
}

uint16_t Lidar::readMeasurePoints()
{
	uint16_t count=0;
	count=awaitStandardScan();
	return count;
}

uint16_t Lidar::awaitStandardScan()
{
	uint8_t *pBuff=(uint8_t*)&DataBuffer; //Pointer to Buffer
	uint16_t count=0;
	rawScanDataPoint_t point;
	bool frameStart=false;

	uint32_t startTime=millis();
	while(millis()<(startTime+5000)) //timeout after 5 seconds
	{
		if(serial->available()>=5)
		{
			serial->readBytes((uint8_t*)&point,5);
			
			//search for frameStart
			if((point.quality&0x01)&&(!(point.quality&0x02))&&!frameStart)
			{
				if(point.angle_low&0x01) //check Bit
				{
					frameStart=true;
				}
			} else if(frameStart&&(point.quality&0x01)&&!(point.quality&0x02)&&count>1)
			{
				if(point.angle_low&0x01)
				{
					return count;
				}
			}
			else if(frameStart)
			{
				memmove(pBuff,(uint8_t*)&point,sizeof(point)); //copy memory from incoming buffer to DataBuffer
				count++; //new point
				if(count<sizeof(DataBuffer)/sizeof(rawScanDataPoint))
				{
					pBuff=pBuff+5; //move pointer to next measure point in storage
				}
			}
			else if(!frameStart)
				serial->readBytes((uint8_t*)&point,1);
			
		}
	}
	return count;
}

float Lidar::calcAngle(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t winkel=_highByte<<7;
	winkel|=_lowByte>>1;
	return winkel/64.0;
}

float Lidar::calcDistance(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t distance=(_highByte)<<8;
	distance|=_lowByte;
	return distance/4.0;
}

// DEBUG FUNCTIONS //
bool Lidar::checkForTimeout(uint32_t _time,size_t _size)
{
	float startTime=millis();
	while(!(serial->available()>=_size))
	{
		if(millis()>(startTime+_time)){
			Serial.println("Lidar Timeout");
			return true;
		}
	}
	return false;
}

bool Lidar::compareDescriptor(rp_descriptor_t _descr1,rp_descriptor_t _descr2)
{
	for(size_t i=0;i<sizeof(rp_descriptor_t);i++)
	{
		if(_descr1[i]!=_descr2[i])
		{
			return false;
		}
	}
	return true;
}
// END DEBUG FUNCTIONS //