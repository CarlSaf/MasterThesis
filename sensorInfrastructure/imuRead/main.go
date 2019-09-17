package main

import (
	"log"
	"os"
	"os/signal"
	// import misc Einride Repos
)

const (
	LcmAddress     = "239.255.76.67:7667"
	ImuChannelName = "IMUlog"
)

func main() {
	log.Println("Initializing logging...")
	loggingConfig := zap.NewDevelopmentConfig()
	loggingConfig.EncoderConfig.EncodeLevel = zapcore.CapitalColorLevelEncoder
	logger, err := loggingConfig.Build()
	if err != nil {
		log.Panic(errors.Wrap(err, "failed to initialize logging"))
	}

	// Sets up a lcm network
	logger.Info("Initializing LCM...")
	LcmAddressUDP := "udpm://" + LcmAddress
	lc, err := lcm.Create(LcmAddressUDP)
	if err != nil {
		logger.Panic("Failed to initialize LCM", zap.Error(err))
	}

	// Reads IMU-data from usb-port
	port, err := serial.OpenPort(&serial.Config{
		Name:     "/dev/ttyUSB0",
		Baud:     xsens.DefaultSerialBaudRate,
		Size:     xsens.MinLengthOfMessage,
		StopBits: xsens.DefaultSerialStopBits,
	})
	defer func() {
		if err := port.Close(); err != nil {
			panic(err)
		}
	}()
	if err != nil {
		logger.Panic("Failed to open Xsens serial port", zap.Error(err))
	}

	// Writes xsens data
	sc := xsens.NewMessageScanner(port)
	interruptChan := make(chan os.Signal, 1)
	signal.Notify(interruptChan, os.Interrupt)
	if _, err := port.Write(xsens.NewMessage(xsens.MessageIdentifierGotoMeasurement, nil)); err != nil {
		panic(err)
	}

	var data xsens.MeasurementData

loop:
	// scans for xsens message
	for sc.Scan() {
		select {
		case <-interruptChan:
			break loop
		default:
		}
		msg := sc.Message()
		if msg.Identifier() == xsens.MessageIdentifierMTData2 {
			if err := data.UnmarshalMTData2(msg); err != nil {
				log.Panic(err)
			}
			// Publishes message to lcm on the specified channel name
			if err := lc.Publish(ImuChannelName, msg); err != nil {
				log.Fatal("failed to publish on LCM")
			}
		}
	}
	if err := sc.Err(); err != nil {
		panic(err)
	}
}
