package main

import (
	"fmt"
	"math"
)

type Vertex struct {
	vector_og_on_xz, vector_ob_on_xz float64
}

func Formula_calculation(a_x, a_y, a_z float64) float64 {
	var vector_og_on_xz, vector_ob_on_xz, vector_ab, vector_ob_on_xz_x_angle, vector_og_on_xz_x_angle, o_centre_gravity_x, o_centre_gravity_z, o_gate_b1_x, o_gate_b1_z float64
	var vector_ob1_length_x, vector_ob1_length_y, vector_ob1_length_z, vector_ab1_length_x, vector_ab1_length_y, vector_ab1_length_z float64
	centre_gravity_x := 3850.0
	centre_gravity_z := 853.576
	hinge_x := 3470.5
	hinge_y := 0.0
	hinge_z := 1240.0
	open_angle := 89.0
	b_x := 3711.0
	b_y := -659.67
	b_z := 745.25
	const PI = 3.141592653589793

	vector_og_on_xz = math.Sqrt((centre_gravity_x-hinge_x)*(centre_gravity_x-hinge_x) + (centre_gravity_z-hinge_z)*(centre_gravity_z-hinge_z))
	// 向量OG在XZ平面的投影长度
	vector_ob_on_xz = math.Sqrt((b_x-hinge_x)*(b_x-hinge_x) + (b_z-hinge_z)*(b_z-hinge_z))
	// 向量OB在XZ平面的投影长度
	vector_ab = math.Sqrt((a_x-b_x)*(a_x-b_x) + (a_y-b_y)*(a_y-b_y) + (a_z-b_z)*(a_z-b_z))
	// 向量AB的长度,即撑杆关门长度
	vector_ob_on_xz_x_angle = (180 / PI) * math.Atan((b_z-hinge_z)/(b_x-hinge_x))
	// 向量OB在XZ平面与X轴的夹角（°）
	vector_og_on_xz_x_angle = (180 / PI) * math.Atan((centre_gravity_z-hinge_z)/(centre_gravity_x-hinge_x))
	// 向量OG在XZ平面与X轴的夹角（°）
	o_centre_gravity_x = vector_og_on_xz*math.Cos((open_angle-math.Abs(vector_og_on_xz_x_angle))*PI/180) + hinge_x
	// 开门重心X坐标
	o_centre_gravity_z = vector_og_on_xz*math.Sin((open_angle-math.Abs(vector_og_on_xz_x_angle))*PI/180) + hinge_z
	// 开门重心z坐标

	o_gate_b1_x = vector_ob_on_xz*math.Cos((open_angle-math.Abs(vector_ob_on_xz_x_angle))*PI/180) + hinge_x
	// 开门尾门连接点B1 X坐标
	o_gate_b1_z = vector_ob_on_xz*math.Sin((open_angle-math.Abs(vector_ob_on_xz_x_angle))*PI/180) + hinge_z
	// 开门尾门连接点B1 z坐标
	vector_ob1_length_x = o_gate_b1_x - hinge_x
	// 向量OB1的长度（X方向分量）

	vector_ob1_length_y = math.Abs(b_y - hinge_y)
	// 向量OB1的长度（Y方向分量）
	vector_ob1_length_z = o_gate_b1_z - hinge_z
	// 向量OB1的长度（Z方向分量）

	vector_ab1_length_x = o_gate_b1_x - a_x
	// 向量AB1的长度（X方向分量）
	vector_ab1_length_y = math.Abs(b_y - a_y)
	// 向量AB1的长度（Y方向分量）

	vector_ab1_length_z = o_gate_b1_z - a_z
	// 向量AB1的长度（Z方向分量）
	return vector_og_on_xz_x_angle
}

func main() {
	var abc float64
	abc = Formula_calculation(3465.1, -575.1, 1134.1)
	fmt.Printf("%v \n", abc)
	// fmt.Printf("vector_ob_on_xz_x_angle= %v vector_ab= %v \n",
	// 	vector_og_on_xz, vector_ab)
}
