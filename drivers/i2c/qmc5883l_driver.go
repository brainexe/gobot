package i2c

import (
	"fmt"
	"log"
	"math"
	"sort"
	"time"
)

const (
	qmc5883lDebug          = false
	qmc5883lDefaultAddress = 0x0D // default I2C Address for QMC5883L
)

const (
	qmc5883lRegX        = 0x00 // Output Data Registers for X
	qmc5883lRegY        = 0x02 // Output Data Registers for Y
	qmc5883lRegZ        = 0x04 // Output Data Registers for Z
	qmc5883lRegStatus   = 0x06 // Status Register
	qmc5883lRegCtrl1    = 0x09 // Control Register 1
	qmc5883lRegCtrl2    = 0x0A // Control Register 2
	qmc5883lRegSetReset = 0x0B // Set/Reset Period Register

	qmc5883lCtrl1_Mode_Standby    = 0x00
	qmc5883lCtrl1_Mode_Continuous = 0x01
	qmc5883lCtrl1_ODR_10Hz        = 0x00 // Output Data Rate 10Hz
	qmc5883lCtrl1_ODR_50Hz        = 0x04 // Output Data Rate 50Hz
	qmc5883lCtrl1_ODR_100Hz       = 0x08 // Output Data Rate 100Hz
	qmc5883lCtrl1_ODR_200Hz       = 0x0C // Output Data Rate 200Hz
	qmc5883lCtrl1_RNG_2G          = 0x00 // Range 2 Gauss
	qmc5883lCtrl1_RNG_8G          = 0x10 // Range 8 Gauss
	qmc5883lCtrl1_OSR_512         = 0x00 // Over Sample Ratio 512
	qmc5883lCtrl1_OSR_256         = 0x40 // Over Sample Ratio 256
	qmc5883lCtrl1_OSR_128         = 0x80 // Over Sample Ratio 128
	qmc5883lCtrl1_OSR_64          = 0xC0 // Over Sample Ratio 64

	qmc5883lCtrl2_SOFT_RST = 0x80 // Soft reset
	qmc5883lCtrl2_ROL_PNT  = 0x40 // Roll-over function
	qmc5883lCtrl2_INT_ENB  = 0x01 // Interrupt Pin Enabling
)

type QMC5883LDriver struct {
	*Driver
	odr  uint8
	rng  uint8
	osr  uint8
	mode uint8
}

var qmc5883lODRBits = map[uint8]uint8{
	10:  qmc5883lCtrl1_ODR_10Hz,
	50:  qmc5883lCtrl1_ODR_50Hz,
	100: qmc5883lCtrl1_ODR_100Hz,
	200: qmc5883lCtrl1_ODR_200Hz,
}

var qmc5883lRNGBits = map[uint8]uint8{
	2: qmc5883lCtrl1_RNG_2G,
	8: qmc5883lCtrl1_RNG_8G,
}

var qmc5883lOSRBits = map[uint16]uint8{
	512: qmc5883lCtrl1_OSR_512,
	256: qmc5883lCtrl1_OSR_256,
	128: qmc5883lCtrl1_OSR_128,
	64:  qmc5883lCtrl1_OSR_64,
}

func NewQMC5883LDriver(c Connector, options ...func(Config)) *QMC5883LDriver {
	q := &QMC5883LDriver{
		Driver: NewDriver(c, "QMC5883L", qmc5883lDefaultAddress),
		odr:    50,
		rng:    2,
		osr:    qmc5883lOSRBits[512],
		mode:   qmc5883lCtrl1_Mode_Continuous,
	}
	q.afterStart = q.initialize

	for _, option := range options {
		option(q)
	}

	return q
}

func WithQMC5883LODR(val uint8) func(Config) {
	return func(c Config) {
		d, ok := c.(*QMC5883LDriver)
		if ok {
			if err := qmc5883lValidateODR(val); err != nil {
				panic(err)
			}
			d.odr = val
		} else if qmc5883lDebug {
			log.Printf("Trying to set ODR for non-QMC5883LDriver %v", c)
		}
	}
}

func WithQMC5883LRNG(val uint8) func(Config) {
	return func(c Config) {
		d, ok := c.(*QMC5883LDriver)
		if ok {
			if err := qmc5883lValidateRNG(val); err != nil {
				panic(err)
			}
			d.rng = val
		} else if qmc5883lDebug {
			log.Printf("Trying to set RNG for non-QMC5883LDriver %v", c)
		}
	}
}

func WithQMC5883LOSR(val uint16) func(Config) {
	return func(c Config) {
		d, ok := c.(*QMC5883LDriver)
		if ok {
			if err := qmc5883lValidateOSR(val); err != nil {
				panic(err)
			}
			d.osr = qmc5883lOSRBits[val]
		} else if qmc5883lDebug {
			log.Printf("Trying to set OSR for non-QMC5883LDriver %v", c)
		}
	}
}

func (q *QMC5883LDriver) Read() (x float64, y float64, z float64, err error) {
	q.mutex.Lock()
	defer q.mutex.Unlock()

	const numReadings = 10 // Number of readings to average
	var sumX, sumY, sumZ int32

	for i := 0; i < numReadings; i++ {
		xr, yr, zr, err := q.readRawData()
		if err != nil {
			return 0, 0, 0, err
		}
		sumX += int32(xr)
		sumY += int32(yr)
		sumZ += int32(zr)
		time.Sleep(10 * time.Millisecond) // Short delay between readings
	}

	avgX := float64(sumX) / numReadings
	avgY := float64(sumY) / numReadings
	avgZ := float64(sumZ) / numReadings

	scale := 12000.0 // for 2G range
	if q.rng == 8 {
		scale = 3000.0 // for 8G range
	}

	return avgX / scale, avgY / scale, avgZ / scale, nil
}

func (q *QMC5883LDriver) Heading() (float64, error) {
	x, y, _, err := q.Read() // Using filtered Read
	if err != nil {
		return 0, err
	}

	heading := math.Atan2(y, x)

	// Convert to degrees
	headingDegrees := heading * 180 / math.Pi

	// Normalize to 0-360
	if headingDegrees < 0 {
		headingDegrees += 360
	}

	return headingDegrees, nil
}

func (q *QMC5883LDriver) readRawData() (x int16, y int16, z int16, err error) {
	data := make([]byte, 6)
	if err = q.connection.ReadBlockData(qmc5883lRegX, data); err != nil {
		return
	}

	x = int16(uint16(data[1]) | uint16(data[0])<<8)
	y = int16(uint16(data[3]) | uint16(data[2])<<8)
	z = int16(uint16(data[5]) | uint16(data[4])<<8)

	return
}

func (q *QMC5883LDriver) initialize() error {
	// Soft reset
	if err := q.connection.WriteByteData(qmc5883lRegCtrl2, qmc5883lCtrl2_SOFT_RST); err != nil {
		return err
	}

	// Wait a bit after reset
	time.Sleep(5 * time.Millisecond)

	// Configure Control Register 1
	ctrl1 := q.mode | qmc5883lODRBits[q.odr] | qmc5883lRNGBits[q.rng] | q.osr
	if err := q.connection.WriteByteData(qmc5883lRegCtrl1, ctrl1); err != nil {
		return err
	}

	// Configure Control Register 2 (optional, adjust as needed)
	ctrl2 := uint8(qmc5883lCtrl2_ROL_PNT) // Enable pointer roll-over
	return q.connection.WriteByteData(qmc5883lRegCtrl2, ctrl2)
}

func qmc5883lValidateODR(odr uint8) error {
	if _, ok := qmc5883lODRBits[odr]; ok {
		return nil
	}

	keys := make([]int, 0, len(qmc5883lODRBits))
	for k := range qmc5883lODRBits {
		keys = append(keys, int(k))
	}
	sort.Ints(keys)

	return fmt.Errorf("ODR must be one of: %v", keys)
}

func qmc5883lValidateRNG(rng uint8) error {
	if _, ok := qmc5883lRNGBits[rng]; ok {
		return nil
	}

	keys := make([]int, 0, len(qmc5883lRNGBits))
	for k := range qmc5883lRNGBits {
		keys = append(keys, int(k))
	}
	sort.Ints(keys)

	return fmt.Errorf("RNG must be one of: %v", keys)
}

func qmc5883lValidateOSR(osr uint16) error {
	if _, ok := qmc5883lOSRBits[osr]; ok {
		return nil
	}

	keys := make([]int, 0, len(qmc5883lOSRBits))
	for k := range qmc5883lOSRBits {
		keys = append(keys, int(k))
	}
	sort.Ints(keys)

	return fmt.Errorf("OSR must be one of: %v", keys)
}
