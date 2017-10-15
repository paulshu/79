package main

import (
	"C"
)
import "math"

func Round(f float64, n int) float64 {
	pow10_n := math.Pow10(n)
	return math.Trunc((f+0.5/pow10_n)*pow10_n) / pow10_n
}

//export GoAdd
func GoAdd(a, b float64) int {
	var abc []float64
	var c float64
	// hinge_y := 0.0
	c = Round((a + b), 1)
	abc = append(abc, a, b, c)
	return len(abc)
}

func main() {}

// Required but ignored
