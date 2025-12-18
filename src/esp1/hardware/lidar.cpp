/** @filename: esp1/hardware/lidar.cpp
 *  @description: Contract for the LiDAR sensor driver and
 *               feature extraction pipeline on ESP_1.
 * 
 *  @note: the raw sensor data must be simplified and refined
 *       before it can be used by the Bayesian Occupancy Grid.
 * 
 *  @job: Run the raw data through Feature Extraction to create a
 *        simplified version of the scan.
 * 
 *  FIXED: Corrected scan completion logic to prevent duplicate data
 */

#include "esp1/hardware/lidar.h"
#include "Arduino.h"

#define MIN(a,b) (((a)<(b))?(a):(b))

rp_descriptor_t resp_descriptor[] = {{0xA5,0x5A,0x54,0x00,0x00,0x40,0x82},  //Legacy Version
								{0xA5,0x5A,0x84,0x00,0x00,0x40,0x84},       //Extended Version
								{0xA5,0x5A,0x54,0x00,0x00,0x40,0x85},       //Dense Version
								{0xA5,0x5A,0x05,0x00,0x00,0x40,0x81}};      //StartScan

rq_message_t req_message[] = {{0xA5,0x25},    //Stop
                                {0xA5,0x40},  //Reset
                                {0xA5,0x20}}; //Scan

bool Lidar::start()
{
	serial.setRxBufferSize(LIDAR_SERIAL_BUFFER_SIZE);
    serial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
	serial.write(req_message[rq_reset], 2);
	delay(1000);
    while(serial.available()) serial.read();
    serial.write((uint8_t*) &req_message[rq_scan], 2);

	rp_descriptor_t descr;
	if(!checkForTimeout(5000, 7)) {
		serial.readBytes((uint8_t*) &descr, 7);
        return compareDescriptor(descr, resp_descriptor[startScan]);
	}
	return false;
}

uint16_t Lidar::readMeasurePoints()
{
	return awaitStandardScan();
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
		if(serial.available()>=5)
		{
			serial.readBytes((uint8_t*)&point,5);
			
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
			// else if(!frameStart)
			// 	serial->readBytes((uint8_t*)&point,1);
			
		}
	}
	return count;
}

float Lidar::calcAngle(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t angle = _highByte << 7;
	angle |= _lowByte >> 1;
	return angle / 64.0;
}

float Lidar::calcDistance(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t distance = (_highByte) << 8;
	distance |= _lowByte;
	return distance / 4.0;
}

void Lidar::build_scan(LiDARScan* scan, bool &scanComplete_, float& lastAngleESP_) {
    // Read raw packets
    uint16_t raw_count = this->readMeasurePoints();

    // Loop through raw data
    for (uint16_t i = 0; i < raw_count; i++) {
        if (scanComplete_) break;
        
        rawScanDataPoint_t dataPoint = this->DataBuffer[i];

        // Standard RPLIDAR Logic: Distance 0 usually means error/invalid
        float dist_mm = this->calcDistance(dataPoint.distance_low, dataPoint.distance_high);
        if (dist_mm == 0.0f) continue; 
        
        float angle_deg = this->calcAngle(dataPoint.angle_low, dataPoint.angle_high);
        uint8_t quality = dataPoint.quality >> 2;

        // Skip very low quality points if necessary (e.g., quality < 10)
        // But for now, we keep them to see "everything"
        if (quality == 0) continue; 

        if (scan->count < MAX_LIDAR_POINTS) {
            scan->angles[scan->count] = angle_deg;
            scan->distances[scan->count] = dist_mm;
            scan->qualities[scan->count] = quality;
            scan->count++;
        } else {
            scanComplete_ = true;
            break;
        }

        // Detect full rotation (360 -> 0 wrap)
        // Using strict hysteresis to avoid triggering on jitter around 0
        if (angle_deg < lastAngleESP_ - 20.0f && lastAngleESP_ > 340.0f) {
            scanComplete_ = true;
        }
        
        lastAngleESP_ = angle_deg;
    }
    
    scan->timestamp_ms = millis();
}

uint16_t Lidar::readScanLive(LiDARScan* scan, bool &scanComplete, float &lastAngle) 
{
    scan->count = 0;
    rawScanDataPoint_t point;
    bool frameStart = false;

    uint32_t startTime = millis();
	uint16_t downsampleCounter = 0;

    while (millis() - startTime < 5000) {
        if (serial.available() < DATA_SIZE) continue;

        // read 5 bytes in the serial buffer
        serial.readBytes((uint8_t*)&point, DATA_SIZE);

        // detection of frame START
        bool isStartFlag = (point.quality & 0x01) && !(point.quality & 0x02);

        if (!frameStart) {
            if (isStartFlag && (point.angle_low & 0x01)) {
                frameStart = true;
                scan->count = 0;
                lastAngle = 0;
				downsampleCounter = 0;
            }
            continue;
        }

        // detection of frame END
        if (isStartFlag && scan->count > 1 && (point.angle_low & 0x01)) {
            scanComplete = true;
            return scan->count;
        }

        // decode point data
        float dist_mm = calcDistance(point.distance_low, point.distance_high);
        float angle_deg = calcAngle(point.angle_low, point.angle_high);

        if (dist_mm <= 0) continue;

		// downsample by 5
		downsampleCounter++;
        if (downsampleCounter % 5 != 0) {
            lastAngle = angle_deg;
            continue;
        }

        // direct storage in buffer
        if (scan->count < MAX_LIDAR_POINTS) {
            scan->angles[scan->count] = angle_deg;
            scan->distances[scan->count] = dist_mm;
            scan->qualities[scan->count] = point.quality >> 2;
            scan->count++;
        }

        // CRITICAL FIX: detect full rotation with hysteresis
        const float ANGLE_WRAP_THRESHOLD = 10.0f;
        if (angle_deg < lastAngle - ANGLE_WRAP_THRESHOLD && lastAngle > 350.0f) {
            scanComplete = true;
            return scan->count;
        }

        lastAngle = angle_deg;
    }

    return scan->count;
}

// DEBUG FUNCTIONS //
bool Lidar::checkForTimeout(uint32_t _time, size_t _size)
{
	float startTime = millis();
	while(!(serial.available() >= _size))
	{
        vTaskDelay(1); // FIX: Yield here too
		if(millis() > (startTime + _time)){
			Serial.println("Lidar Timeout");
			return true;
		}
	}
	return false;
}

bool Lidar::compareDescriptor(rp_descriptor_t _descr1, rp_descriptor_t _descr2)
{
	for(size_t i = 0; i < sizeof(rp_descriptor_t); i++)
	{
		if(_descr1[i] != _descr2[i])
		{
			return false;
		}
	}
	return true;
}
// END DEBUG FUNCTIONS //