/*
 *  @author KKest
 *		@created 10.01.2022
 *
 * Library to control an rpLidar S2
 *
 */
#include "../esp1/hardware/rpLidar.h"
#include "Arduino.h"


rpLidar::rpLidar()
{
	// serial=_mySerial;
	status=false;
}

void rpLidar::begin(uint32_t baud, uint16_t bufferSize, HardwareSerial *_mySerial, int8_t rxPin, int8_t txPin) {
	serial = _mySerial;
	serial->setTxBufferSize(bufferSize);
    serial->begin(baud, SERIAL_8N1, rxPin, txPin);
}


stDeviceInfo_t rpLidar::getDeviceInfo()
{
	clearSerialBuffer();
	stDeviceInfo_t info;
	rp_descriptor_t descr;
	serial->write((uint8_t*)&req_message[rq_info],2); //send Device Info request
	if(!checkForTimeout(10,27))	//wait for Response
	{
		serial->readBytes((uint8_t*)&descr,7);
		serial->readBytes((uint8_t*)&info,20);
	}
	return info;
}

stDeviceStatus_t rpLidar::getDeviceHealth()
{
	clearSerialBuffer(); //remove old data in SerialBuffer
	rp_descriptor_t descr;
	stDeviceStatus_t deviceStatus;
	serial->write((uint8_t*)&req_message[rq_health],2); //send device health request
	if(!checkForTimeout(400,10)) //wait for response
	{
		serial->readBytes((uint8_t*)&descr,7);
		serial->readBytes((uint8_t*)&deviceStatus,3);
	}
	return deviceStatus;
}

uint16_t rpLidar::scanExpress()
{
	start(express);
	return readMeasurePoints();
}

uint16_t rpLidar::scanStandard()
{
	start(standard);
	return readMeasurePoints();
}

void rpLidar::resetDevice()
{
	serial->write((uint8_t*)&req_message[rq_reset],2); //send reset request
	delay(800); //wait for reboot
	clearSerialBuffer(); //remove old data in SerialBuffer
	status=false;
}

void rpLidar::stopDevice()
{
	serial->write((uint8_t*)&req_message[rq_stop],2);
}

bool rpLidar::start(uint8_t _mode)
{
	resetDevice();
	clearSerialBuffer();
	//GG switch(express)
	switch(_mode)
	{
		case standard:
			serial->write((uint8_t*)&req_message[rq_scan],2); //standard scan request
			break;
		case express:
			serial->write((uint8_t*)&req_Express[legacyVersion],9); //express scan request
			break;
		default:
			return false;
			break;
	}

	rp_descriptor_t descr;

	if(!checkForTimeout(100,7)) //wait for response
	{
		serial->readBytes((uint8_t*)&descr,7);
		//printf("descr: ");
		//for(int i=0;i<7;i++)
		//	printf("%x ", descr[i]);
		//printf("\r\n");
		switch(_mode)
		{
			case standard:
				scanMode=_mode;
				status=true;
				return compareDescriptor(descr,resp_descriptor[startScan]);
				break;

			case express:
				scanMode=_mode;
				status=true;
				//GG return compareDescriptor(descr,resp_descriptor[legacyVersion]);
				return compareDescriptor(descr,resp_descriptor[denseVersion]);
				break;
			default :
				Serial.print("Kein Mode : ");
				Serial.println(_mode);
				scanMode=stop;
				status=false;
				return false;
				break;
		}

	}
	return false;
}

uint16_t rpLidar::readMeasurePoints()
{
	uint16_t count=0;
	switch(scanMode)
	{
		case standard:
			count=awaitStandardScan();
			break;
		case express:
			count=awaitExpressScan();
			break;
	}
	return count;
}


uint16_t rpLidar::awaitStandardScan()
{
	uint8_t *pBuff=(uint8_t*)&DataBuffer; //Pointer to Buffer
	uint16_t count=0;
	stScanDataPoint_t point;
	bool frameStart=false;

	uint32_t startTime=millis();
	while(millis()<(startTime+5000)) //timeout after 5 seconds
	{
		if(serial->available()>=5)
		{
			serial->readBytes((uint8_t*)&point,5);
			// printf("point.angle_high=%d point.angle_low=%d point.distance_high=%d point.distance_low=%d point.quality=%d\r\n",
			// 	point.angle_high, point.angle_low, point.distance_high,point.distance_low,point.quality);
			
			//search for frameStart
			//	new Frame? S=1 !S=0 an checkbit information can be found in protocol datasheet
			if((point.quality&0x01)&&(!(point.quality&0x02))&&!frameStart) //  framestart? S=1 !S=0
			{
				//GG if(point.angle_high&0x01) //check Bit
				if(point.angle_low&0x01) //check Bit
				{
					frameStart=true;
					//printf("found framestart at count=%d\r\n",count);
				}
			} else if(frameStart&&(point.quality&0x01)&&!(point.quality&0x02)&&count>1) //2. framestart?
			{
				//GG if(point.angle_high&0x01)
				if(point.angle_low&0x01)
				{
					//printf("found frameend at count=%d\r\n",count);
					return count;
				}
			}
			else if(frameStart)
			{
				memmove(pBuff,(uint8_t*)&point,sizeof(point)); //copy memory from incoming buffer to DataBuffer
				count++; //new point
				if(count<sizeof(DataBuffer)/sizeof(scanDataPoint))//inside the array bondaries?
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

uint16_t rpLidar::awaitExpressScan()
{
	uint8_t Buffer[2];
	uint16_t count=0; //count of Packets with angle and 40 cabins
	uint8_t cabinCount=0; //count of cabin which haves to be written
	serial->flush();
	uint8_t crc=0;
	uint32_t internTimeCount=millis();
	while(count<79)
	{
		if(serial->available()>=1)
		{
			uint8_t sync1=serial->read(); //Sync byte 1 of Packet
			if(((sync1&0xF0)==0xA0))
			{
				while(serial->available()<=1)
				{
						if(millis()>internTimeCount+500)
						{
							return 0;
						}
					
				}
				internTimeCount=millis();
				uint8_t sync2=serial->read(); //Sync byte 2 of Packet
				if((sync2&0xF0)==0x50)
				{
					crc=(sync2<<4)|(sync1&0x0F);

					while(serial->available()<2)
					{
						if(millis()>internTimeCount+500)return 0;
					}
					serial->readBytes((uint8_t*)&Buffer,2);//read angle
					ExpressDataBuffer[count].angle=(Buffer[1]<<8)|Buffer[0]; //connect angle low and high byte
					while(cabinCount<40)
					{
						while(serial->available()<2)
						{
							if(millis()>internTimeCount+500)return 0;
						}
						if(serial->available()>=2)//cabin available?
						{
							serial->readBytes((uint8_t*)&Buffer,2);
							ExpressDataBuffer[count].cabin[cabinCount]=Buffer[1]<<8|Buffer[0];
							cabinCount++;
							internTimeCount=millis();
						}
					}
					cabinCount=0;
					if(!checkCRC(ExpressDataBuffer[count],crc))
					{
						return 0;
					}
					count++;
				}
			}
		}
	}
	ExpressDataToPointArray((stExpressDataPacket_t*) ExpressDataBuffer, count-1);
	return count;
}


void rpLidar::setAngleOfInterest(uint16_t _left,uint16_t _right)
{
	//setter
	interestAngleLeft=_left;
	interestAngleRight=_right;
}


bool rpLidar::isDataBetweenBorders(stScanDataPoint_t _point)
{
	float angle=calcAngle(_point.angle_low,_point.angle_high);
	if((angle>=interestAngleLeft)&&(angle<=interestAngleRight))
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataBetweenBorders(float _angle)
{
	if((_angle>interestAngleLeft)&&(_angle<interestAngleRight))
	{
		return true;
	}
	return false;
}


bool rpLidar::isDataValid(stScanDataPoint_t _point)
{
	if(calcDistance(_point.distance_low,_point.distance_high)>0)
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataValid(uint16_t _distance)
{
	if(_distance>0)
	{
		return true;
	}
	return false;
}

bool rpLidar::isRunning()
{
	return status;
}

uint8_t rpLidar::isScanMode()
{
	return scanMode;
}

float rpLidar::calcAngle(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t winkel=_highByte<<7;
	winkel|=_lowByte>>1;
	return winkel/64.0;
}

float rpLidar::calcCapsuledAngle(uint16_t _Wi,uint16_t _Wi2,uint8_t _k)
{
	float angle1=_Wi/64.00;
	float angle2=_Wi2/64.00;
	float result;
	if(angle1<=angle2)
	{
		result=angle1+((angle2-angle1)/40)*_k;
	}
	else
	{
		result=angle1+((360+angle2-angle1)/40)*_k;
	}
	if(result>360.0)
	{
		result=result-360.0;
	}
	return result;
}




float rpLidar::calcDistance(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t distance=(_highByte)<<8;
	distance|=_lowByte;
	return distance/4.0;
}



//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//
//											 Debug Functions
//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//

void rpLidar::DebugPrintMeasurePoints(int16_t _count)
{
	Serial.println(_count-1);
	if(_count<=0)return;

    for(uint16_t i=0;i<_count-1;i++)
    {
		if(isDataBetweenBorders(DataBuffer[i])&&isDataValid(DataBuffer[i]))
		{
			Serial.print(i);
			Serial.print("\t|\t");
			Serial.print((DataBuffer[i].quality)>>2,DEC);
			Serial.print("\t|\t");
			Serial.print(calcAngle(DataBuffer[i].angle_low,DataBuffer[i].angle_high));
			Serial.print("\t|\t");
			Serial.print(calcDistance(DataBuffer[i].distance_low,DataBuffer[i].distance_high));
			Serial.println();
		}
	}
}

void rpLidar::DebugPrintDeviceErrorStatus(stDeviceStatus_t _status)
{
	Serial.println("\n--Device Health--");
	Serial.print("Status:");
	Serial.println(_status.status);
	Serial.print("Error Low:");
	Serial.print(_status.errorCode_low);
	Serial.print("Error High:");
	Serial.println(_status.errorCode_high);
	Serial.println('\n');
}

void rpLidar::DebugPrintDeviceInfo(stDeviceInfo_t _info)
{
	Serial.println("\n--Device Info--");
	Serial.print("Model:");
	Serial.println(_info.model);
	Serial.print("Firmware:");
	Serial.print(_info.firmware_major);
	Serial.print(".");
	Serial.println(_info.firmware_minor);
	Serial.print("Hardware:");
	Serial.println(_info.hardware);
	Serial.print("Serial Number:");
	for(uint16_t i=0;i<16;i++)
	{
		Serial.print(_info.serialnumber[i],HEX);
	}
	Serial.println('\n');

}

void rpLidar::DebugPrintDescriptor(rp_descriptor_t _descriptor)
{
	Serial.print ("Descriptor : ");
	for(uint8_t i=0;i<7;i++)
	{
		Serial.print(_descriptor[i],HEX);
		Serial.print("|");
	}
	Serial.println();
}

void rpLidar::DebugPrintBufferAsHex()
{
	for(uint16_t i=0;i<sizeof(DataBuffer)/sizeof(scanDataPoint);i++)
	{
		Serial.print("0x");
		Serial.print(DataBuffer[i].quality,HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].angle_low,HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].angle_high,HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].distance_low,HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].distance_high,HEX);
		Serial.println(",");
	}
	Serial.println();
}


//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//
//											End Debug Functions
//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//




bool rpLidar::compareDescriptor(rp_descriptor_t _descr1,rp_descriptor_t _descr2)
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

void rpLidar::clearSerialBuffer()
{
	while(serial->available())//read as long the hardware buffer is not empty
	{
		serial->read();
	}
}

bool rpLidar::checkCRC(stExpressDataPacket_t _package,uint8_t _crc)
{
	uint8_t crc=0;
	crc=(uint8_t)_package.angle&0x00FF;
	crc^=(uint8_t)(_package.angle>>8);
	for(int i=0;i<40;i++)
	{
		crc^=(uint8_t)_package.cabin[i];
		crc^=(uint8_t)(_package.cabin[i]>>8);

	}
	if(_crc==crc)
	{
		return true;
	}
	return false;
}

bool rpLidar::checkForTimeout(uint32_t _time,size_t _size)
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

bool rpLidar::ExpressDataToPointArray(stExpressDataPacket_t* _packets, uint16_t _count)
{
	uint16_t index=0;
	/// do a copy of expressData packet buffer and replace the angle of each cabin with the real angle
	for(uint16_t i=0;i<_count-2;i++) //each expressData packet
	{
		for(uint16_t j=0;j<40;j++) //each cabin in expressData packet
		{
			double angle=calcAngle(_packets,j); //calculate the angle of current cabin
			if(isDataBetweenBorders(angle)) //data correct interesting and in fov?
			{
				Data[index].angle=angle;
				Data[index].distance=_packets->cabin[j];
				index++;
			}
			else if(isDataBetweenBorders(angle))
			{
				Data[index].angle=0;
				Data[index].distance=0;
				index++;
			}
		}
		_packets++;
	}

/// sorting start
///Aufsteigend sortieren, höchster winkel am Ende des Arrays
	uint16_t tmpDistance; //temporary storage for value
	double tmpAngle; //temporary storage for value
	bool changed;
    int counter = index-1;

	do //sorts the data from low to high angle , highest angle is at the last index of the array
	{
		changed = false;
		for (int i = 0; i < counter; i++)
		{
			if (Data[i].angle > Data[i + 1].angle)
			{
				tmpAngle = Data[i + 1].angle;
				tmpDistance = Data[i + 1].distance;
				Data[i + 1].angle = Data[i].angle;
				Data[i + 1].distance = Data[i].distance;
				Data[i].angle = tmpAngle;
				Data[i].distance=tmpDistance;
				changed = true;
			}
		}
	}while(changed);
///sorting END
	return true;
}

double  rpLidar::calcAngle(stExpressDataPacket_t* _packets,uint16_t _k)
{
	double  angle1=(_packets->angle&0x7FFF)/64.00;
	_packets++;
	double  angle2=(_packets->angle&0x7FFF)/64.00;
	double  result;
	if(angle1<=angle2)
	{
		result=angle1+((angle2-angle1)/40)*_k;
	}
	else
	{
		result=angle1+((360+angle2-angle1)/40)*_k;
	}
	if(result>360.0)
	{
		result=result-360.0;
	}
	return result;

}
/*
LIDAR POSTPROCESSING

// Filename: esp1/hardware/lidar.cpp
// Description: LiDAR driver + lightweight feature extraction (2D)
// Works with your headers: common/data_types.h, esp1/hardware/lidar.h

#include "esp1/hardware/lidar.h"
#include <cmath>
#include <algorithm>
#include <chrono>

// ---------------------------
// Tunable parameters
// ---------------------------
namespace {
constexpr float kMinRangeM           = 0.05f;   // ignore < 5 cm
constexpr float kMaxRangeM           = 12.0f;   // sensor-dependent
constexpr float kMinQuality01        = 0.15f;   // 0..1 minimum average quality
constexpr float kAngleIncrementRad   = static_cast<float>(M_PI/180.0); // ~1 deg default
constexpr int   kMedianWin           = 5;       // median filter window (odd)
constexpr float kClusterGapNearM     = 0.08f;   // max gap for clustering at near ranges
constexpr float kClusterGapFarM      = 0.20f;   // ...at far ranges
constexpr float kFarRangeThresholdM  = 4.0f;
constexpr int   kMinClusterPoints    = 5;       // min points to accept a landmark
constexpr float kMaxAngularSpanRad   = static_cast<float>(8.0*M_PI/180.0); // ~8 degrees
constexpr float kMaxClusterStdM      = 0.12f;   // reject “spread out” clusters
//15 Hz prendre ttes les datas sur chaque angle, il faut au moins filtrer le moitié

#define LIDAR_UART Serial2
#define LIDAR_BAUD 115200      // RPLIDAR C1 default
#define LIDAR_RX_PIN 5         // ESP32 pin receiving from LiDAR TX
#define LIDAR_TX_PIN 4         // ESP32 pin sending to LiDAR RX

inline uint32_t now_ms() {
    using namespace std::chrono;
    return static_cast<uint32_t>(duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count());
}
inline float wrap_angle(float a) {
    while (a <= -M_PI) a += 2.f*static_cast<float>(M_PI);
    while (a >   M_PI) a -= 2.f*static_cast<float>(M_PI);
    return a;
}
template<typename T>
T clamp(T v, T lo, T hi){ return std::max(lo, std::min(hi, v)); }
} // namespace

// ---------------------------
// Lidar class impl
// ---------------------------
Lidar::Lidar() {}

// Replace with your real hardware init (UART baud, GPIO motor enable, etc.)
bool Lidar::initialize() {
     LIDAR_UART.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

        return true;
}

// Read one full 360° sweep into raw_points.
// Replace the body with your sensor protocol parsing.
uint32_t Lidar::read_raw_data(std::vector<RawLiDARPoint>& raw_points) {
    raw_points.clear();
    raw_points.reserve(MAX_RAW_LIDAR_POINTS);

    // -----------------------------
    // TODO: Parse your LiDAR stream.
    // This stub simulates a simple arc of points for wiring tests.
    // -----------------------------
    const uint32_t t_ms = now_ms();
    for (int i = 0; i < static_cast<int>(MAX_RAW_LIDAR_POINTS); ++i) {
        float ang = i * kAngleIncrementRad;              // 0..~2π
        if (ang > 2.f*static_cast<float>(M_PI)) break;
        // Fake wall at y = 2.0 m; compute range along ray (for demo only)
        float ray_dx = std::cos(ang), ray_dy = std::sin(ang);
        float r = INFINITY;
        if (std::fabs(ray_dy) > 1e-4f) {
            float t = 2.0f / ray_dy; // y = 2, solve (0,0)+t*(dx,dy)
            if (t > 0.f) r = t;
        }
        RawLiDARPoint p;
        p.angle_rad  = ang;
        p.distance_m = std::isfinite(r) ? clamp(r + 0.01f*std::sin(6*ang), kMinRangeM, kMaxRangeM) : INFINITY;
        p.quality    = 200; // mid-high quality
        raw_points.push_back(p);
    }

    return t_ms;
}

// Median filter helper (in-place on ranges keeping angles/quality)
static void median_filter(std::vector<RawLiDARPoint>& pts, int win) {
    if (win < 3 || win % 2 == 0 || pts.empty()) return;
    const int half = win / 2;
    std::vector<float> buf(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) buf[i] = pts[i].distance_m;

    std::vector<float> window;
    window.reserve(win);
    for (size_t i = 0; i < pts.size(); ++i) {
        window.clear();
        for (int k = -half; k <= half; ++k) {
            long j = static_cast<long>(i) + k;
            if (j < 0) j = 0;
            if (j >= static_cast<long>(buf.size())) j = static_cast<long>(buf.size()) - 1;
            window.push_back(buf[static_cast<size_t>(j)]);
        }
        std::nth_element(window.begin(), window.begin()+half, window.end());
        float med = window[half];
        pts[i].distance_m = med;
    }
}

// Distance between two polar samples accounting for angle step.
static float euclidean_gap(float r1, float r2, float dtheta) {
    // Law of cosines: d^2 = r1^2 + r2^2 - 2 r1 r2 cos(dtheta)
    if (!std::isfinite(r1) || !std::isfinite(r2)) return INFINITY;
    return std::sqrt(std::max(0.f, r1*r1 + r2*r2 - 2.f*r1*r2*std::cos(dtheta)));
}

std::vector<LiDARLandmark> Lidar::extract_features(const std::vector<RawLiDARPoint>& raw_points) {
    std::vector<LiDARLandmark> features;
    if (raw_points.empty()) return features;

    // Copy and pre-filter
    std::vector<RawLiDARPoint> pts = raw_points;

    // 1) Range & quality filtering
    pts.erase(std::remove_if(pts.begin(), pts.end(), [](const RawLiDARPoint& p){
        if (!std::isfinite(p.distance_m)) return true;
        if (p.distance_m < kMinRangeM || p.distance_m > kMaxRangeM) return true;
        float q = p.quality / 255.f;
        return q < 0.05f; // drop extremely low-quality
    }), pts.end());
    if (pts.size() < static_cast<size_t>(kMinClusterPoints)) return features;

    // 2) Median filter on range to suppress spikes
    median_filter(pts, kMedianWin);

    // 3) Adjacent-point clustering in scan order
    //    Gap threshold increases with range (beams spread).
    auto gap_threshold = [](float r)->float {
        return (r < kFarRangeThresholdM) ? kClusterGapNearM : kClusterGapFarM;
    };

    std::vector<size_t> cluster_idx;
    cluster_idx.reserve(32);

    auto flush_cluster = [&](const std::vector<size_t>& idxs){
        if ((int)idxs.size() < kMinClusterPoints) return;

        // Compute centroid in Cartesian, angular span, quality
        float cx=0.f, cy=0.f, wsum=0.f, ang_min=+1e9f, ang_max=-1e9f;
        float qsum=0.f; int n=0;
        for (size_t k : idxs) {
            const auto& p = pts[k];
            float a = p.angle_rad;
            float r = p.distance_m;
            float x = r*std::cos(a), y = r*std::sin(a);
            float w = 1.0f; // equal weights; could weight by quality
            cx += w*x; cy += w*y; wsum += w;
            ang_min = std::min(ang_min, a);
            ang_max = std::max(ang_max, a);
            qsum += (p.quality/255.f);
            ++n;
        }
        if (wsum <= 0.f) return;
        cx /= wsum; cy /= wsum;
        float span = std::fabs(wrap_angle(ang_max - ang_min));
        if (span > kMaxAngularSpanRad) return; // not point-like, likely a wall

        // radial compactness
        float std_acc=0.f;
        for (size_t k : idxs) {
            const auto& p = pts[k];
            float a = p.angle_rad;
            float r = p.distance_m;
            float x = r*std::cos(a) - cx, y = r*std::sin(a) - cy;
            std_acc += (x*x + y*y);
        }
        float spread = std::sqrt(std_acc / n);
        if (spread > kMaxClusterStdM) return;

        // Convert centroid back to polar for your LiDARLandmark type
        float range = std::sqrt(cx*cx + cy*cy);
        float angle = std::atan2(cy, cx);
        LiDARLandmark lm;
        lm.range   = range;
        lm.angle   = wrap_angle(angle);
        lm.quality = clamp(qsum / std::max(1, n), 0.f, 1.f);
        if (lm.quality < kMinQuality01) return;

        features.push_back(lm);
    };

    cluster_idx.clear();
    for (size_t i = 0; i+1 < pts.size(); ++i) {
        cluster_idx.push_back(i);
        float r1 = pts[i].distance_m, r2 = pts[i+1].distance_m;
        float gap = euclidean_gap(r1, r2, kAngleIncrementRad);
        float thr = std::max(gap_threshold(std::min(r1,r2)), 0.05f);
        if (!(gap < thr)) {        // cluster boundary
            flush_cluster(cluster_idx);
            cluster_idx.clear();
        }
    }
    // Add last point and flush
    if (!pts.empty()) cluster_idx.push_back(pts.size()-1);
    flush_cluster(cluster_idx);

    return features;
}

LiDARScan Lidar::get_processed_scan() {
    std::vector<RawLiDARPoint> raw_points;
    raw_points.reserve(MAX_RAW_LIDAR_POINTS);

    const uint32_t t_ms = read_raw_data(raw_points);
    // NOTE: read_raw_data must fill a full sweep in increasing angle order.

    LiDARScan scan;
    scan.timestamp_ms = t_ms;
    scan.landmarks    = extract_features(raw_points);
    return scan;
}
 */