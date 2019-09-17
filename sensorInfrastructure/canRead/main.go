package main

import (
	"log"
	
	// import misc Einride Repos
)

const (
	canAddres  = "vcan0"
	LcmAddress = "239.255.76.67:7667"
)

func main() {
	log.Println("Initializing logging...")
	loggingConfig := zap.NewDevelopmentConfig()
	loggingConfig.EncoderConfig.EncodeLevel = zapcore.CapitalColorLevelEncoder
	logger, err := loggingConfig.Build()
	if err != nil {
		log.Fatal("Logging failed", err)
	}

	// connecting to CAN-bus
	conn, err := socketcan.Dial("can", canAddres)
	if err != nil {
		log.Fatal("CAN dial failed", err)
	}

	// Sets up a lcm network
	logger.Info("Initializing LCM...")
	LcmAddressUDP := "udpm://" + LcmAddress
	lc, err := lcm.Create(LcmAddressUDP)
	if err != nil {
		logger.Panic("Failed to initialize LCM")
	}

	for {
		// Creates a can frame of socket-CAN connection
		frame, err := socketcan.ReadFrame(conn)
		if err != nil {
			log.Panic("Failed to identify CAN-Frame", err)
		}

		// JSON marshal CAN frame
		canPublish, err := frame.MarshalJSON()
		if err != nil {
			log.Fatal("CAN Marshalling failed")
		}

		// Publish marshaled CAN frame in corresponding channel
		if frame.ID == dataspeedcan.Descriptor.Steering_Report.ID {
			if err := lc.Publish("canSteeringReport", canPublish); err != nil {
				log.Fatal("Failed to publish on LCM", zap.Error(err))
			}
		}

		if frame.ID == dataspeedcan.Descriptor.WheelSpeed_Report.ID {
			if err := lc.Publish("canWheelSpeedReport", canPublish); err != nil {
				log.Fatal("Failed to publish on LCM", zap.Error(err))
			}
		}

		if frame.ID == dataspeedcan.Descriptor.Gyro_Report.ID {
			if err := lc.Publish("canGyroReport", canPublish); err != nil {
				log.Fatal("Failed to publish on LCM", zap.Error(err))
			}
		}

		if frame.ID == dataspeedcan.Descriptor.Accel_Report.ID {
			if err := lc.Publish("canAccelReport", canPublish); err != nil {
				log.Fatal("Failed to publish on LCM", zap.Error(err))
			}
		}
	}
}
