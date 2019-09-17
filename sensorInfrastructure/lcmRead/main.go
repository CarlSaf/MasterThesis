package main

import (
	"bytes"
	"encoding/binary"
	"encoding/csv"
	"encoding/hex"
	"fmt"
	"log"
	"net"
	"os"
	"strings"
	"time"

	// import misc Einride Repos
)

const (
	LcmAddress     = "239.255.76.67:7667"
	ImuChannelName = "IMUlog"
)

type HeaderUDP struct {
	lcmMagic   []byte
	sequenceNo uint32
}
type MessageUDP struct {
	channel []byte
	data    []byte
}
type LcmPacketUDP struct {
	header  HeaderUDP
	message MessageUDP
}

// Makes a lcm packet out of a byte slice message
func (lcmPackage *LcmPacketUDP) makePackage(fMessage []byte) error {
	var err error
	// Retrieves the magic header and checks if the message is of udp type
	lcmPackage.header.lcmMagic = fMessage[:4]
	if hex.EncodeToString(lcmPackage.header.lcmMagic) != "4c433032" {
		log.Fatal("Header: not lcmPackage")
		return err
	}

	// Retrieves the sequence number
	lcmPackage.header.sequenceNo = binary.BigEndian.Uint32(fMessage[4:8])

	// Retrieves the channel name
	msg := bytes.NewBuffer(fMessage[8:])
	lcmPackage.message.channel, err = msg.ReadBytes(0x00)
	if err != nil {
		log.Fatal("Message: failed to read channel name", err)
		return err
	}

	// Remaining data is the udp message
	lcmPackage.message.data = msg.Bytes()

	return nil
}

func main() {
	log.Println("Initializing logging...")
	loggingConfig := zap.NewDevelopmentConfig()
	loggingConfig.EncoderConfig.EncodeLevel = zapcore.CapitalColorLevelEncoder
	logger, err := loggingConfig.Build()
	if err != nil {
		log.Panic("failed to initialize logging")
	}

	// Connecting to specified LCM network
	logger.Info("Connecting to LCM...")
	udpAdr, err := net.ResolveUDPAddr("udp", LcmAddress)
	if err != nil {
		logger.Panic("Failed to resolve lcm address", zap.Error(err))
	}

	// Establishing udp type connection within the LCM
	f, err := net.ListenUDP("udp", udpAdr)
	if err != nil {
		logger.Panic("Failed to establish udp connection", zap.Error(err))
	}

	// Creates a CSV-file for logging
	csvFile, err := os.Create("result_" + time.Now().String() + ".csv")
	if err != nil {
		log.Fatal("File creation failed:", err)
	}

	// Writes header for CSV-file
	err = writeCVSheader(csvFile)
	if err != nil {
		log.Fatal("File header failed:", err)
	}

	log.Println("LCM- & udp connections established successfully")
	log.Println("Waiting for message...")

	// Starts counter for time logging
	start := time.Now()

	for {
		// Creating a byte slice of maximum length for a udp package
		fMessage := make([]byte, 65507)

		// Copies incoming udp message onto fMessage
		messLen, err := f.Read(fMessage)
		log.Println("LCM-message retrieved")
		if err != nil {
			log.Fatal("Failed to read udp message", zap.Error(err))
		}

		// Creating a lcm packet out of udp message (byte slice)
		lcmPackage := LcmPacketUDP{}
		err = lcmPackage.makePackage(fMessage[:messLen])
		if err != nil {
			log.Fatal("makePackage failed:", err)
		}

		// Retrieves and formats the messages channel name
		channelName := strings.Trim(string(lcmPackage.message.channel), "\x00")
		// Saves the elapsed time for logging
		elapsed := fmt.Sprint(time.Now().Sub(start).Seconds())

		var s string
		isWritableMessage := false

		// CSV publications for imu packet data
		if channelName == ImuChannelName {
			//Unmarshal IMU package
			s, err = lcmPackage.imuUnmarshal()
			if err != nil {
				log.Fatal("IMU unmarshal failed:", err)
			}
			// Declares the message as writable
			isWritableMessage = true
		}

		// CSV publications for can packet data
		if channelName[:3] == "can" {
			// Creates a can frame
			var frame can.Frame
			// Unmarshal CAN package
			err := frame.UnmarshalJSON(lcmPackage.message.data)
			if err != nil {
				log.Fatal("unmarshal of CAN/LCM-data failed")
			}

			// Different message cases
			if channelName == "canSteeringReport" {
				canD := dataspeedcan.Steering_Report{}
				err = canD.UnmarshalCANFrame(frame)
				if err != nil {
					log.Fatal("CAN frame unmarshal failed", err)
				}

				s = s + fmt.Sprint(canD.ANGLE.Physical())
				s = s + " " + fmt.Sprint(canD.SPEED.Physical())
				s = s + " " + fmt.Sprint(canD.CMD.Physical())
				isWritableMessage = true
			}

			if channelName == "canWheelSpeedReport" {
				canD := dataspeedcan.WheelSpeed_Report{}
				err = canD.UnmarshalCANFrame(frame)
				if err != nil {
					log.Fatal("CAN frame unmarshal failed", err)
				}

				s = spaces(1 + 1) // csv format offset
				s = s + fmt.Sprint(canD.FL.Physical())
				s = s + " " + fmt.Sprint(canD.FR.Physical())
				s = s + " " + fmt.Sprint(canD.RL.Physical())
				s = s + " " + fmt.Sprint(canD.RR.Physical())
				isWritableMessage = true
			}

			if channelName == "canGyroReport" {
				canD := dataspeedcan.Gyro_Report{}
				err = canD.UnmarshalCANFrame(frame)
				if err != nil {
					log.Fatal("CAN frame unmarshal failed", err)
				}

				s = spaces(1 + 2 + 3) // csv format offset
				s = s + fmt.Sprint(canD.ROLL.Physical())
				s = s + " " + fmt.Sprint(canD.YAW.Physical())
				isWritableMessage = true
			}

			if channelName == "canAccelReport" {
				canD := dataspeedcan.Accel_Report{}
				err = canD.UnmarshalCANFrame(frame)
				if err != nil {
					log.Fatal("CAN frame unmarshal failed", err)
				}

				s = spaces(1 + 1 + 1 + 3 + 2) // csv format offset
				s = s + fmt.Sprint(canD.LAT.Physical())
				s = s + " " + fmt.Sprint(canD.LONG.Physical())
				s = s + " " + fmt.Sprint(canD.VERT.Physical())
				isWritableMessage = true
			}
		}

		// Writes the message in csv file if declared writable
		if isWritableMessage == true {
			err = writeCSV(csvFile, s, elapsed, channelName)
			if err != nil {
				log.Fatal("CVS write failed:", err)
			}
			log.Println("LCM-message successfully logged")
		}
	}
}

// Unmarshal imu packages from byte slice
func (lcmPackage *LcmPacketUDP) imuUnmarshal() (s string, err error) {
	var imuData xsens.MeasurementData
	msg := xsens.Message(lcmPackage.message.data)
	if msg.Identifier() == xsens.MessageIdentifierMTData2 {
		if err := imuData.UnmarshalMTData2(msg); err != nil {
			log.Fatal("Could not unmarshal imu data", err)
		}
	} else {
		log.Fatal("could not identify as IMU data", err)
		return "", err
	}

	// CSV formatting in accordance with csv file header
	s = spaces(6 + 5) // csv offset
	s = s + fmt.Sprintf("%f", imuData.EulerAngles)
	s = s + " " + fmt.Sprintf("%f", imuData.Acceleration)
	s = s + " " + fmt.Sprintf("%f", imuData.DeltaV)
	s = s + " " + fmt.Sprintf("%f", imuData.RateOfTurn)
	s = s + " " + fmt.Sprintf("%f", imuData.DeltaQ)
	s = s + " " + fmt.Sprintf("%f", imuData.LatLon)
	s = s + " " + fmt.Sprintf("%f", imuData.AltitudeEllipsoid)
	s = s + " " + fmt.Sprintf("%f", imuData.VelocityXYZ)
	s = s + " " + fmt.Sprintf("%d", imuData.GNSSPVTData.Lon)
	s = s + " " + fmt.Sprintf("%d", imuData.GNSSPVTData.Lat)
	s = s + " " + fmt.Sprintf("%d", imuData.GNSSPVTData.Height)
	s = s + " " + fmt.Sprintf("%d", imuData.GNSSPVTData.HeadVeh)
	s = strings.Replace(s, "}", "", -1)
	s = strings.Replace(s, "{", "", -1)

	return s, nil
}

// Writes string to csv file in accordance with csv header
func writeCSV(CSVfile *os.File, rowWrite, elapsed, channel string) error {
	writer := csv.NewWriter(CSVfile)
	defer writer.Flush()
	if err := writer.Write([]string{elapsed + " " + channel + " " + rowWrite}); err != nil {
		return err
	}
	return nil
}

// Writes CSV file header
func writeCVSheader(csvFile *os.File) error {
	x := []string{
		"LogTime",
		"lcmChannel",

		// CAN
		"SteeringReport-Angle", "SteeringReport-VEL", "SteeringReport-CMD",
		"WheelSpeedReport-FL", "WheelSpeedReport-FR", "WheelSpeedReport-RL", "WheelSpeedReport-RR",
		"IMU-Roll", "IMU-YAW", "IMU-Acc-lat", "IMU-Acc-long", "IMU-Acc-vert",

		// IMU
		"EulerAngleX", "EulerAngleY", "EulerAngleZ",
		"AccelerationX", "AccelerationY", "AccelerationZ",
		"DeltaVX", "DeltaVY", "DeltaVZ",
		"RateOfTurnX", "RateOfTurnY", "RateOfTurnZ",
		"DeltaQa", "DeltaQb", "DeltaQc", "DeltaQd",
		"Lat", "Lon", "AltitudeEllipsoid",
		"VelocityX", "VelocityY", "VelocityZ",

		"GNSSlon", "GNSSlat", "GNSSHeightAboveEllipsoid", "GNSSHeadVeh"} //,
	//"SpeedNorth", "SpeedEast", "SpeedDown"}

	fileData := [][]string{x}
	writer := csv.NewWriter(csvFile)
	defer writer.Flush()
	for _, value := range fileData {
		if err := writer.Write(value); err != nil {
			log.Fatal("CSV Creation Failed")
			return err
		}
	}
	return nil
}

// Function for csv formatting
func spaces(nr int) (s string) {
	s = " "
	for i := 0; i < nr; i++ {
		s = s + " "
	}
	return s
}
