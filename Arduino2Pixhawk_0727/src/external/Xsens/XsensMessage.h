#ifndef XsensMessage
#define XsensMessage

#include <map>

enum XsDataIdentifier
{
	XDI_None					= 0x0000,	//!< Empty datatype
	XDI_TypeMask				= 0xFE00,	//!< Mask for checking the group which a dataidentifier belongs to, Eg. XDI_TimestampGroup or XDI_OrientationGroup
	XDI_FullTypeMask			= 0xFFF0,	//!< Mask to get the type of data, without the data format
	XDI_FullMask				= 0xFFFF,	//!< Complete mask to get entire data identifier
	XDI_FormatMask				= 0x01FF,	//!< Mask for getting the data id without checking the group
	XDI_DataFormatMask			= 0x000F,	//!< Mask for extracting just the data format /sa XDI_SubFormat

	XDI_SubFormatMask			= 0x0003,	//!< Determines, float, fp12.20, fp16.32, double output... (where applicable)
	XDI_SubFormatFloat			= 0x0000,	//!< Floating point format
	XDI_SubFormatFp1220			= 0x0001,	//!< Fixed point 12.20
	XDI_SubFormatFp1632			= 0x0002,	//!< Fixed point 16.32
	XDI_SubFormatDouble			= 0x0003,	//!< Double format

	XDI_SubFormatLeft			= 0x0000,	//!< Left side data (ie XDI_GloveData for the left hand)
	XDI_SubFormatRight			= 0x0001,	//!< Right side data (ie XDI_GloveData for the right hand)

	XDI_TemperatureGroup		= 0x0800,	//!< Group for temperature outputs
	XDI_Temperature				= 0x0810,	//!< Temperature

	XDI_TimestampGroup			= 0x1000,	//!< Group for time stamp related outputs
	XDI_UtcTime					= 0x1010,	//!< Utc time from the GNSS receiver
	XDI_PacketCounter			= 0x1020,	//!< Packet counter, increments every packet
	XDI_Itow					= 0x1030,	//!< Itow. Time Of Week from the GNSS receiver
	XDI_GnssAge					= 0x1040,	//!< Gnss age from the GNSS receiver
	XDI_PressureAge				= 0x1050,	//!< Age of a pressure sample, in packet counts
	XDI_SampleTimeFine			= 0x1060,	//!< Sample Time Fine
	XDI_SampleTimeCoarse		= 0x1070,	//!< Sample Time Coarse
	XDI_FrameRange				= 0x1080,	//!< Reserved \internal add for MTw (if needed)
	XDI_PacketCounter8			= 0x1090,	//!< 8 bit packet counter, wraps at 256
	XDI_SampleTime64			= 0x10A0,	//!< 64 bit sample time

	XDI_OrientationGroup		= 0x2000,	//!< Group for orientation related outputs
	XDI_CoordSysMask			= 0x000C,	//!< Mask for the coordinate system part of the orientation data identifier
	XDI_CoordSysEnu				= 0x0000,	//!< East North Up orientation output
	XDI_CoordSysNed				= 0x0004,	//!< North East Down orientation output
	XDI_CoordSysNwu				= 0x0008,	//!< North West Up orientation output
	XDI_Quaternion				= 0x2010,	//!< Orientation in quaternion format
	XDI_QuaternionNED			= 0x2014,	//!< Orientation in quaternion format, NED coordinate system
	XDI_RotationMatrix			= 0x2020,	//!< Orientation in rotation matrix format
	XDI_RotationMatrixNED		= 0x2024,	//!< Orientation in rotation matrix format, NED coordinate system
	XDI_EulerAngles				= 0x2030,	//!< Orientation in euler angles format
	XDI_EulerAnglesNED			= 0x2034,	//!< Orientation in euler angles format, NED coordinate system

	XDI_PressureGroup			= 0x3000,	//!< Group for pressure related outputs
	XDI_BaroPressure			= 0x3010,	//!< Pressure output recorded from the barometer
	XDI_BaroPressureNED			= 0x3014,	//!< Pressure output recorded from the barometer, NED coordinate system

	XDI_AccelerationGroup		= 0x4000,	//!< Group for acceleration related outputs
	XDI_DeltaV					= 0x4010,	//!< DeltaV SDI data output
	XDI_Acceleration			= 0x4020,	//!< Acceleration output in m/s2
	XDI_AccelerationNED			= 0x4024,	//!< Acceleration output in m/s2, NED coordinate system
	XDI_FreeAcceleration		= 0x4030,	//!< Free acceleration output in m/s2
	XDI_AccelerationHR			= 0x4040,	//!< AccelerationHR output

	XDI_IndicationGroup			= 0x4800,	//!< 0100.1000 -> bit reverse = 0001.0010 -> type 18
	XDI_TriggerIn1				= 0x4810,	//!< Trigger in 1 indication
	XDI_TriggerIn2				= 0x4820,	//!< Trigger in 2 indication
	XDI_TriggerIn3				= 0x4830,	//!< Trigger in 3 indication

	XDI_PositionGroup			= 0x5000,	//!< Group for position related outputs
	XDI_AltitudeMsl				= 0x5010,	//!< Altitude at Mean Sea Level
	XDI_AltitudeEllipsoid		= 0x5020,	//!< Altitude at ellipsoid
	XDI_PositionEcef			= 0x5030,	//!< Position in earth-centered, earth-fixed format
	XDI_LatLon					= 0x5040,	//!< Position in latitude, longitude
	XDI_LatLon8B				= 0x5043,	//!< Position in latitude, longitude

	XDI_GnssGroup				= 0x7000,	//!< Group for Gnss related outputs
	XDI_GnssPvtData				= 0x7010,	//!< Gnss position, velocity and time data
	XDI_GnssSatInfo				= 0x7020,	//!< Gnss satellite information
	XDI_GnssPvtPulse			= 0x7030,	//!< Time of the PVT timepulse (4Hz upsampled from the 1PPS)

	XDI_AngularVelocityGroup	= 0x8000,	//!< Group for angular velocity related outputs
	XDI_RateOfTurn				= 0x8020,	//!< Rate of turn data in rad/sec
	XDI_RateOfTurnNED			= 0x8024,	//!< Rate of turn data in rad/sec, NED coordinate system
	XDI_DeltaQ					= 0x8030,	//!< DeltaQ SDI data
	XDI_RateOfTurnHR			= 0x8040,	//!< Rate of turn HR data

	XDI_RawSensorGroup			= 0xA000,	//!< Group for raw sensor data related outputs
	XDI_RawUnsigned				= 0x0000,	//!< Tracker produces unsigned raw values, usually fixed behavior
	XDI_RawSigned				= 0x0001,	//!< Tracker produces signed raw values, usually fixed behavior
	XDI_RawAccGyrMagTemp		= 0xA010,	//!< Raw acceleration, gyroscope, magnetometer and temperature data
	XDI_RawGyroTemp				= 0xA020,	//!< Raw gyroscope and temperature data
	XDI_RawAcc					= 0xA030,	//!< Raw acceleration data
	XDI_RawGyr					= 0xA040,	//!< Raw gyroscope data
	XDI_RawMag					= 0xA050,	//!< Raw magnetometer data
	XDI_RawDeltaQ				= 0xA060,	//!< Raw deltaQ SDI data
	XDI_RawDeltaV				= 0xA070,	//!< Raw deltaV SDI data
	XDI_RawBlob					= 0xA080,	//!< Raw blob data

	XDI_AnalogInGroup			= 0xB000,	//!< Group for analog in related outputs
	XDI_AnalogIn1				= 0xB010,	//!< Data containing adc data from analog in 1 line (if present)
	XDI_AnalogIn2				= 0xB020,	//!< Data containing adc data from analog in 2 line (if present)

	XDI_MagneticGroup			= 0xC000,	//!< Group for magnetometer related outputs
	XDI_MagneticField			= 0xC020,	//!< Magnetic field data in a.u.
	XDI_MagneticFieldNED		= 0xC024,	//!< Magnetic field data in a.u., NED coordinate system
	XDI_MagneticFieldCorrected	= 0xC030,	//!< Corrected Magnetic field data in a.u. (ICC result)

	XDI_SnapshotGroup			= 0xC800,	//!< Group for snapshot related outputs
	XDI_RetransmissionMask		= 0x0001,	//!< Mask for the retransmission bit in the snapshot data
	XDI_RetransmissionFlag		= 0x0001,	//!< Bit indicating if the snapshot if from a retransmission
	XDI_AwindaSnapshot			= 0xC810,	//!< Awinda type snapshot
	XDI_FullSnapshot			= 0xC820,	//!< Full snapshot
	XDI_GloveSnapshotLeft		= 0xC830,	//!< Glove Snapshot for Left Hand
	XDI_GloveSnapshotRight		= 0xC840,	//!< Glove Snapshot for Right Hand

	// The numbers of the GloveData items should match the GloveSnapshot items, but in the CA range
	XDI_GloveDataGroup			= 0xCA00,	//!< Group for usable glove data
	XDI_GloveDataLeft			= 0xCA30,	//!< Glove Data for Left Hand
	XDI_GloveDataRight			= 0xCA40,	//!< Glove Data for Right Hand

	XDI_VelocityGroup			= 0xD000,	//!< Group for velocity related outputs
	XDI_VelocityXYZ				= 0xD010,	//!< Velocity in XYZ coordinate frame
	XDI_VelocityXYZNED			= 0xD014,	//!< Velocity in XYZ coordinate frame, NED coordinate system

	XDI_StatusGroup				= 0xE000,	//!< Group for status related outputs
	XDI_StatusByte				= 0xE010,	//!< Status byte
	XDI_StatusWord				= 0xE020,	//!< Status word
	XDI_Rssi					= 0xE040,	//!< Rssi information
	XDI_DeviceId				= 0xE080,	//!< DeviceId output
	XDI_LocationId				= 0xE090,	//!< LocationId output
};


enum XDI_MID {
	REQDID = 0x00, 
	DEVICEID = 0x01, 
	INIT_MID = 0x02,
	INIT_MIDACK = 0x03,

	AKEUP = 0x3E, 
	GOTOCONFIG = 0x30,
	GOTOCONFIGACK = 0x31, 
	GOTOMEASUREMENT = 0x10,
	GOTOMEASUREMENTACK = 0x11, 
	RESTOREFACTORY_MID = 0x0E,
	REQPRODUCTCODE = 0x1C,
    PRODUCTCODE = 0x1D, 
	REQFWREV = 0x12, 
	FIRMWAREREV = 0x13, 
	RESET_MID = 0x40,
	RESET_MIDACK = 0x41,
	ERROR = 0x42, 
	WARNING = 0x43, 
	OUTPUTCONFIGURATION = 0xC0, 
	OUTPUTCONFIGURATIONACK = 0xC1,
	MTDATA2 = 0x36,
	SETGNSSMID = 0xAC, 
	SETGNSSMIDACK = 0xAD,
	SETALIGNMENTROTATION_MID = 0xEC,
	SETALIGNMENTROTATION_MIDACK = 0xED,
	STRINGOUTPUTTYPE_MID = 0x8E,
	STRINGOUTPUTTYPE_MIDACK = 0x8F,
	SETPORTCONFIG_MID = 0x8C,
	SETPORTCONFIG_MIDACK = 0x8D,
	SETNOROTATION = 0x22,
	NONE
};


enum MODE{
	CONFIGMODE, 
	MEASUREMENTMODE
};


enum OutputPackets {
    DEFAULT_OP = 0,
    DEFAULT2_OP,
    IMU_OP,
    MAG_OP,
    BAR_OP,
    GNSS_RAW_OP,
    EULERs_OP,
	QUT_OP, 
    EKF_OP,
	GYRO_OP,
	UTC_OP
};

enum DataStatus {
	NO_DATA = 0,
	DATA_OK = 1,
	DATA_WARNING = 2,
};


enum XsensMode{
	MODE_IMU, 
	MODE_AHRS,
	MODE_AHRS_QUT,
	MODE_INS,
	MODE_INS_PAV_QUT,
	MODE_INS_UTC,
	MODE_GNSS_RAW
};

struct UTC_TIME{
	my_data_u4 ns;
	my_data_u2 year;
	my_data_u1 month;
	my_data_u1 day;
	my_data_u1 hour;
	my_data_u1 minute;
	my_data_u1 second;
};

#endif

